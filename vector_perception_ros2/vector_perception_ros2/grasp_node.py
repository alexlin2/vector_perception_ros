#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import numpy as np
import struct
from scipy.spatial.transform import Rotation

from gsnet import AnyGrasp
from vector_perception_msgs.msg import Object3DArray

class AnyGraspSimpleNode(Node):
    def __init__(self):
        super().__init__('anygrasp_node')

        # Fixed parameters
        self.process_all_objects = True  # Process all detected objects
        self.checkpoint_path = os.path.join(os.path.dirname(__file__), 'checkpoint_detection.tar')

        # Initialize components
        self.bridge = CvBridge()

        # Initialize AnyGrasp
        self.init_anygrasp()

        # Create synchronized subscribers for objects and color image
        self.objects_sub = Subscriber(self, Object3DArray, 'detected_objects')
        self.color_sub = Subscriber(self, Image, '/camera/color/image_raw')

        # Create synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.objects_sub, self.color_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)

        # Create publishers
        self.grasp_markers_pub = self.create_publisher(MarkerArray, '/grasp/markers', 10)
        self.grasp_poses_pub = self.create_publisher(PoseStamped, '/grasp/pose', 10)
        self.all_grasp_poses_pub = self.create_publisher(PoseArray, '/grasp/poses', 10)
        self.combined_cloud_pub = self.create_publisher(PointCloud2, '/grasp/pointcloud', 10)

        self.get_logger().info('AnyGrasp simple node initialized')

    def init_anygrasp(self):
        """Initialize AnyGrasp with fixed configuration parameters."""
        try:
            # Create config object similar to what AnyGrasp expects
            class Config:
                def __init__(self, checkpoint_path):
                    self.checkpoint_path = checkpoint_path
                    self.max_gripper_width = 0.1
                    self.gripper_height = 0.05
                    self.top_down_grasp = False
                    self.debug = False

            cfg = Config(self.checkpoint_path)

            # Initialize AnyGrasp
            self.anygrasp = AnyGrasp(cfg)
            self.anygrasp.load_net()
            self.get_logger().info('AnyGrasp initialized successfully')

            # Note about coordinate systems
            self.get_logger().info('Using AnyGrasp coordinates directly for ROS')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize AnyGrasp: {str(e)}')
            self.anygrasp = None

    # We don't need object size calculation anymore as we process all objects

    def sync_callback(self, objects_msg, color_msg):
        """Process synchronized Object3DArray and color image messages."""
        try:
            if not objects_msg.objects:
                self.get_logger().info("No objects detected")
                return

            if self.anygrasp is None:
                self.get_logger().error('AnyGrasp not initialized')
                return

            # Convert color image to OpenCV format
            color_img = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            self.get_logger().info(f"Processing {len(objects_msg.objects)} objects")

            # Use all detected objects
            selected_objects = objects_msg.objects

            # Create a combined point cloud
            combined_points = []

            # Get frame from message header
            frame_id = objects_msg.header.frame_id
            stamp = objects_msg.header.stamp

            # Create a combined mask for all selected objects
            if selected_objects:
                first_mask = self.bridge.imgmsg_to_cv2(selected_objects[0].mask, desired_encoding='mono8')
                img_height, img_width = first_mask.shape
                combined_mask = np.zeros((img_height, img_width), dtype=np.uint8)
            else:
                return

            # Process each selected object
            for obj in selected_objects:
                # Get point cloud
                pc_points = []

                # Convert PointCloud2 to list of points
                for p in point_cloud2.read_points(
                    obj.point_cloud,
                    field_names=('x', 'y', 'z')
                ):
                    pc_points.append([p[0], p[1], p[2]])

                # Add points to combined list
                combined_points.extend(pc_points)

                # Get object mask
                mask = self.bridge.imgmsg_to_cv2(obj.mask, desired_encoding='mono8')

                # Add to combined mask
                combined_mask = np.maximum(combined_mask, mask)

            if not combined_points:
                self.get_logger().warning('No points in combined point cloud')
                return

            # Convert to numpy array
            points = np.array(combined_points, dtype=np.float32)

            # Apply mask to color image to get colors for each point
            # First convert mask to boolean
            mask_bool = combined_mask > 0

            # Get masked color image
            masked_color = color_img.copy()
            masked_color[~mask_bool] = 0

            # For simplicity, project points back to image space to get colors
            # This is approximate and could be improved with proper point cloud RGB data
            # Extract all pixel colors from the masked image
            mask_indices = np.where(mask_bool)
            pixel_colors = masked_color[mask_indices]

            # In cases where we have more or fewer mask pixels than points,
            # we'll need to handle it by duplicating or sampling
            if len(pixel_colors) < len(points):
                # If we have fewer colors than points, duplicate colors
                indices = np.random.choice(len(pixel_colors), size=len(points))
                pixel_colors = pixel_colors[indices]
            elif len(pixel_colors) > len(points):
                # If we have more colors than points, sample colors
                indices = np.random.choice(len(pixel_colors), size=len(points), replace=False)
                pixel_colors = pixel_colors[indices]

            # Convert to float format expected by AnyGrasp (0-1 range)
            colors = pixel_colors.astype(np.float32) / 255.0

            # Alternatively, just use a solid color if the above approach is problematic
            # colors = np.ones_like(points) * np.array([0.5, 0.5, 0.5])

            # Define workspace limits (can be adjusted based on camera frame)
            xmin, xmax = points[:, 0].min() - 0.2, points[:, 0].max() + 0.2
            ymin, ymax = points[:, 1].min() - 0.2, points[:, 1].max() + 0.2
            zmin, zmax = points[:, 2].min() - 0.2, max(points[:, 2].max() + 0.2, 1.0)

            # Ensure a reasonable workspace size
            lims = [xmin, xmax, ymin, ymax, zmin, zmax]

            # Publish combined pointcloud for visualization
            combined_pc_msg = self.create_point_cloud_msg(points, colors, objects_msg.header)
            self.combined_cloud_pub.publish(combined_pc_msg)

            # Log information for debugging
            self.get_logger().info(f"Points shape: {points.shape}, Colors shape: {colors.shape}")

            # Call AnyGrasp to get grasp poses
            try:
                self.get_logger().info("Calling AnyGrasp to compute grasps...")
                gg, cloud = self.anygrasp.get_grasp(
                    points,
                    colors,
                    lims=lims,
                    apply_object_mask=False,
                    dense_grasp=False,  # Use dense grasp to get more candidates
                    collision_detection=True
                )
                self.get_logger().info(f"Found {len(gg)} grasp candidates")
            except Exception as e:
                self.get_logger().error(f"AnyGrasp error: {str(e)}")
                return

            if len(gg) == 0:
                self.get_logger().warning('No grasps detected after collision detection')
                return

            # Sort grasps by score and apply non-maximum suppression
            gg = gg.nms().sort_by_score()

            # Select top N grasps - show more since we're processing all objects
            num_grasps = min(50, len(gg))
            top_grasps = gg[:num_grasps]

            # Publish grasp markers
            self.publish_grasp_markers(top_grasps, frame_id, stamp)

            # Publish all grasp poses as array
            self.publish_all_grasps(top_grasps, frame_id, stamp)
            
            # Publish best grasp pose
            if len(top_grasps) > 0:
                self.publish_best_grasp(top_grasps[0], frame_id, stamp)

        except Exception as e:
            self.get_logger().error(f'Error processing objects: {str(e)}')

    def create_point_cloud_msg(self, points, colors, header):
        """Create a PointCloud2 message from points and colors."""
        all_points = []

        for i in range(len(points)):
            x, y, z = points[i]
            r, g, b = colors[i] * 255  # Scale colors to 0-255
            rgb = struct.unpack('f', struct.pack('BBBB', int(r), int(g), int(b), 255))[0]
            all_points.append([x, y, z, rgb])

        fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='rgb', offset=12, datatype=point_cloud2.PointField.FLOAT32, count=1)
        ]

        pc2_msg = point_cloud2.create_cloud(header, fields, all_points)
        return pc2_msg

    def publish_grasp_markers(self, grasps, frame_id, stamp):
        """Publish visualization markers for grasps."""
        marker_array = MarkerArray()

        for i, grasp in enumerate(grasps):
            # Get grasp parameters directly from AnyGrasp
            translation = grasp.translation
            rotation = grasp.rotation_matrix
            width = grasp.width
            score = grasp.score

            # Create marker for gripper base
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = translation[0]
            marker.pose.position.y = translation[1]
            marker.pose.position.z = translation[2]

            # Convert rotation matrix to quaternion
            q = self.rotation_matrix_to_quaternion(rotation)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]

            # Set marker size (gripper base)
            marker.scale.x = 0.02
            marker.scale.y = width
            marker.scale.z = 0.02

            # Color based on score (green to red)
            marker.color.r = max(0.0, min(1.0, 1.0 - score))
            marker.color.g = max(0.0, min(1.0, score))
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

            # Add approach direction marker (arrow)
            approach_marker = Marker()
            approach_marker.header.frame_id = frame_id
            approach_marker.header.stamp = stamp
            approach_marker.id = i + 1000  # Use different ID range
            approach_marker.type = Marker.ARROW
            approach_marker.action = Marker.ADD
            approach_marker.pose.position.x = translation[0]
            approach_marker.pose.position.y = translation[1]
            approach_marker.pose.position.z = translation[2]
            approach_marker.pose.orientation = marker.pose.orientation

            # Arrow length and width
            approach_marker.scale.x = 0.05  # Arrow length
            approach_marker.scale.y = 0.01  # Arrow width
            approach_marker.scale.z = 0.01  # Arrow height

            # Color slightly different from gripper
            approach_marker.color.r = max(0.0, min(1.0, 1.0 - score))
            approach_marker.color.g = max(0.0, min(1.0, score))
            approach_marker.color.b = 0.5
            approach_marker.color.a = 0.8

            marker_array.markers.append(approach_marker)

        self.grasp_markers_pub.publish(marker_array)

    def rotation_matrix_to_quaternion(self, R):
        """Convert 3x3 rotation matrix to quaternion using scipy."""
        # Create a Rotation object from the rotation matrix
        rot = Rotation.from_matrix(R)

        # Get quaternion (x, y, z, w)
        quat = rot.as_quat()

        # Return in the format expected by ROS (x, y, z, w)
        return quat

    def publish_all_grasps(self, grasps, frame_id, stamp):
        """Publish all grasp poses as a PoseArray."""
        # Create PoseArray message
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = frame_id
        pose_array_msg.header.stamp = stamp
        
        # Add each grasp pose to the array
        for grasp in grasps:
            # Get grasp parameters
            translation = grasp.translation
            rotation = grasp.rotation_matrix
            
            # Create Pose message
            pose = Pose()
            pose.position.x = translation[0]
            pose.position.y = translation[1]
            pose.position.z = translation[2]
            
            # Convert rotation matrix to quaternion
            q = self.rotation_matrix_to_quaternion(rotation)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            # Add to array
            pose_array_msg.poses.append(pose)
        
        # Publish pose array
        self.all_grasp_poses_pub.publish(pose_array_msg)

    def publish_best_grasp(self, grasp, frame_id, stamp):
        """Publish the best grasp pose."""
        # Get grasp parameters
        translation = grasp.translation
        rotation = grasp.rotation_matrix

        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = stamp

        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]

        # Convert rotation matrix to quaternion
        q = self.rotation_matrix_to_quaternion(rotation)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        # Publish pose
        self.grasp_poses_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AnyGraspSimpleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()