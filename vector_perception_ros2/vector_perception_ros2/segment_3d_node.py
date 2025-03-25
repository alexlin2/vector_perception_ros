#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from vector_perception.segmentation import Sam2DSegmenter
from vector_perception.common import depth_to_point_cloud, fit_cuboid, visualize_fit
import tf2_ros
import struct
import torch

# Import our custom messages
from vector_perception_msgs.msg import Object3D, Object3DArray

class Segment3DNode(Node):
    def __init__(self):
        super().__init__('segment_3d_node')
        
        # Declare parameters
        self.declare_parameter('color_image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('enable_analysis', False)
        self.declare_parameter('min_analysis_interval', 4.0)
        
        # Get parameters
        color_topic = self.get_parameter('color_image_topic').value
        depth_topic = self.get_parameter('depth_image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        depth_info_topic = self.get_parameter('depth_camera_info_topic').value
        enable_tracking = self.get_parameter('enable_tracking').value
        enable_analysis = self.get_parameter('enable_analysis').value
        min_analysis_interval = self.get_parameter('min_analysis_interval').value
        
        # Initialize components
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize segmenter
        self.segmenter = Sam2DSegmenter(
            model_path='FastSAM-s.pt',
            use_tracker=enable_tracking,
            use_analyzer=enable_analysis,
            min_analysis_interval=min_analysis_interval if enable_analysis else 0.0
        )
        
        # Create synchronized subscribers
        self.color_sub = Subscriber(self, Image, color_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)
        self.info_sub = Subscriber(self, CameraInfo, info_topic)
        self.depth_info_sub = Subscriber(self, CameraInfo, depth_info_topic)

        self.enable_tracking = enable_tracking
        self.enable_analysis = enable_analysis
        
        # Create synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub, self.depth_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.image_callback)
        
        # Create publishers
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'segmented_objects', 10)
        self.viz_pub = self.create_publisher(Image, 'detection_visualization', 10)
        
        # Add new publisher for Object3DArray
        self.objects_pub = self.create_publisher(Object3DArray, 'detected_objects', 10)

    def generate_color_from_id(self, track_id):
        """Generate a consistent color for a given tracking ID."""
        np.random.seed(track_id)
        color = np.random.randint(0, 255, 3)
        np.random.seed(None)
        return color

    def create_point_cloud_msg(self, points, header):
        """Create a PointCloud2 message from points."""
        fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
        ]

        pc2_msg = point_cloud2.create_cloud(header, fields, points)
        return pc2_msg

    def image_callback(self, color_msg, depth_msg, info_msg, depth_info_msg):
        """Process synchronized color and depth images."""
        try:
            # Convert messages to OpenCV format
            color_img = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            
            # Get camera matrices
            camera_matrix = np.array(info_msg.k).reshape(3, 3)
            depth_camera_matrix = np.array(depth_info_msg.k).reshape(3, 3)
            
            # Convert depth image from mm to meters if needed
            if depth_img.dtype == np.uint16:
                depth_img = depth_img.astype(np.float32) / 1000.0
            
            # Run SAM segmentation
            masks, bboxes, target_ids, probs, names = self.segmenter.process_image(color_img)
            
            # Run image analysis if enabled
            if self.enable_analysis and self.enable_tracking:
                self.segmenter.run_analysis(color_img, bboxes, target_ids)
                names = self.segmenter.get_object_names(target_ids, names)
            
            # Initialize visualization image
            viz_img = color_img.copy()
            
            # First draw the segmentation results
            viz_img = self.segmenter.visualize_results(
                viz_img, 
                masks, 
                bboxes, 
                target_ids, 
                probs, 
                names
            )
            
            # Create Object3DArray message
            objects_msg = Object3DArray()
            objects_msg.header = color_msg.header
            
            # Process each detected object
            for mask, bbox, target_id, prob, name in zip(masks, bboxes, target_ids, probs, names):
                # Convert mask to numpy if needed
                if hasattr(mask, 'cpu'):
                    mask = mask.cpu().numpy()
                mask = mask.astype(bool)
                
                # Get masked depth image
                masked_depth = depth_img.copy()
                masked_depth[~mask] = 0
                
                # Convert to point cloud
                points = depth_to_point_cloud(masked_depth, depth_camera_matrix)
                
                # Skip if no points
                if len(points) == 0:
                    continue
                
                # Create Object3D message
                obj_msg = Object3D()
                obj_msg.header = color_msg.header
                obj_msg.target_id = target_id
                obj_msg.probability = float(prob)
                obj_msg.name = name if name else ""
                
                # Convert mask to Image message
                mask_img = (mask.astype(np.uint8) * 255)
                obj_msg.mask = self.bridge.cv2_to_imgmsg(mask_img, encoding='mono8')
                
                # Set bounding box
                obj_msg.bbox = [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])]
                
                # Convert points to PointCloud2
                obj_msg.point_cloud = self.create_point_cloud_msg(points, color_msg.header)
                
                # Generate color for visualization
                rgb = self.generate_color_from_id(target_id)
                obj_msg.color = ColorRGBA(r=float(rgb[0])/255.0, g=float(rgb[1])/255.0, b=float(rgb[2])/255.0, a=1.0)
                
                # Fit cuboid
                cuboid_params = fit_cuboid(points)
                if cuboid_params is not None:
                    # Draw cuboid on visualization
                    viz_img = visualize_fit(
                        viz_img,
                        cuboid_params,
                        camera_matrix
                    )
                
                # Add object to array
                objects_msg.objects.append(obj_msg)
            
            # Publish object array if we have objects
            if objects_msg.objects:
                self.objects_pub.publish(objects_msg)
                
                # Create a combined point cloud for backward compatibility
                all_points = []
                all_colors = []
                
                for obj in objects_msg.objects:
                    # Extract points from each object's point cloud
                    cloud_points = point_cloud2.read_points_list(
                        obj.point_cloud, field_names=('x', 'y', 'z')
                    )
                    
                    # Use the object's color
                    obj_color = [
                        int(obj.color.r * 255),
                        int(obj.color.g * 255),
                        int(obj.color.b * 255)
                    ]
                    
                    all_points.append(cloud_points)
                    all_colors.append(obj_color)
                
                # Create combined point cloud message (for backward compatibility)
                if all_points:
                    combined_pc_msg = self.create_combined_point_cloud_msg(all_points, all_colors, color_msg.header)
                    self.pointcloud_pub.publish(combined_pc_msg)
            
            # Publish visualization
            viz_msg = self.bridge.cv2_to_imgmsg(viz_img, encoding='bgr8')
            viz_msg.header = color_msg.header
            self.viz_pub.publish(viz_msg)

            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            
        except Exception as e:
            self.get_logger().error(f'Error processing images: {str(e)}')

    def create_combined_point_cloud_msg(self, points_list, colors_list, header):
        """Create a combined PointCloud2 message from points and colors."""
        all_points = []
        
        for points, color in zip(points_list, colors_list):
            for point in points:
                x, y, z = point
                r, g, b = color
                rgb = struct.unpack('f', struct.pack('BBBB', r, g, b, 255))[0]
                all_points.append([x, y, z, rgb])

        fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='rgb', offset=12, datatype=point_cloud2.PointField.FLOAT32, count=1)
        ]

        pc2_msg = point_cloud2.create_cloud(header, fields, all_points)
        return pc2_msg

    def cleanup(self):
        """Cleanup resources before node shutdown."""
        if hasattr(self, 'segmenter'):
            self.segmenter.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = Segment3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()