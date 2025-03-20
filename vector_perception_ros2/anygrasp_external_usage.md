# Using AnyGrasp SDK in External Python Applications

This guide explains how to integrate AnyGrasp's grasp detection functionality into your own Python applications outside the AnyGrasp SDK repository.

## 1. Prerequisites

- Python 3.6-3.10
- PyTorch 1.7.1+ with CUDA 11.x/12.1
- MinkowskiEngine v0.5.4
- Other dependencies from requirements.txt in the AnyGrasp SDK
- Valid AnyGrasp license files

## 2. Installation Steps

1. **Install Dependencies**:
   ```bash
   pip install torch==1.10.1 # or your compatible PyTorch version
   # Install MinkowskiEngine following their instructions
   pip install -r /path/to/anygrasp_sdk/requirements.txt
   ```

2. **Install PointNet2 module**:
   ```bash
   cd /path/to/anygrasp_sdk/pointnet2
   python setup.py install
   ```

3. **Set Up License**: 
   - Register for a license using your machine ID:
     ```bash
     cd /path/to/anygrasp_sdk/license_registration
     ./license_checker -f
     ```
   - Apply for a license at https://forms.gle/XVV3Eip8njTYJEBo6 using your machine ID
   - Once received, extract the license files to a directory

4. **Prepare Binary Files**:
   - Copy the appropriate version of `gsnet.so` to your project directory:
     ```bash
     cp /path/to/anygrasp_sdk/grasp_detection/gsnet_versions/gsnet.cpython-{your-python-version}-x86_64-linux-gnu.so /your/project/directory/gsnet.so
     ```
   - Copy the appropriate version of `lib_cxx.so`:
     ```bash
     cp /path/to/anygrasp_sdk/license_registration/lib_cxx_versions/lib_cxx.cpython-{your-python-version}-x86_64-linux-gnu.so /your/project/directory/lib_cxx.so
     ```

5. **Copy the Checkpoint File**:
   ```bash
   cp /path/to/anygrasp_sdk/log/checkpoint_detection.tar /your/project/directory/
   ```

## 3. Using AnyGrasp in Your Code

Here's a minimal example showing how to use AnyGrasp's grasp detection:

```python
import os
import argparse
import numpy as np
from PIL import Image
import torch

# Import AnyGrasp (ensure gsnet.so is in your directory or Python path)
from gsnet import AnyGrasp
from graspnetAPI import GraspGroup

# Define configuration
class Config:
    def __init__(self):
        self.checkpoint_path = 'checkpoint_detection.tar'  # Path to your checkpoint file
        self.max_gripper_width = 0.1
        self.gripper_height = 0.03
        self.top_down_grasp = True
        self.debug = False

# Initialize AnyGrasp
config = Config()
anygrasp = AnyGrasp(config)
anygrasp.load_net()

# Example: Process your point cloud data
def detect_grasps(point_cloud, color_data, limits=None):
    """
    Detect grasps in a point cloud
    
    Args:
        point_cloud: numpy array of shape (N, 3) with XYZ coordinates
        color_data: numpy array of shape (N, 3) with RGB values (0-1)
        limits: optional workspace limits [xmin, xmax, ymin, ymax, zmin, zmax]
    
    Returns:
        gg: GraspGroup object containing detected grasps
        cloud: Open3D point cloud object
    """
    # Set default limits if None
    if limits is None:
        xmin, xmax = -0.5, 0.5
        ymin, ymax = -0.5, 0.5
        zmin, zmax = 0.0, 1.0
        limits = [xmin, xmax, ymin, ymax, zmin, zmax]
    
    # Call AnyGrasp's grasp detection
    gg, cloud = anygrasp.get_grasp(
        point_cloud,
        color_data,
        lims=limits,
        apply_object_mask=True,  # Filter grasps by objectness mask
        dense_grasp=False,       # Less dense but higher quality grasps
        collision_detection=True # Enable collision detection
    )
    
    # Process results
    if len(gg) > 0:
        gg = gg.nms().sort_by_score()
    
    return gg, cloud

# Example: Convert depth image to point cloud
def depth_to_point_cloud(depth_img, color_img, fx, fy, cx, cy, depth_scale=1000.0):
    """
    Convert depth image to point cloud
    
    Args:
        depth_img: numpy array with depth values
        color_img: numpy array with RGB values (0-255)
        fx, fy: focal lengths
        cx, cy: principal point
        depth_scale: depth scale factor
    
    Returns:
        points: numpy array with XYZ coordinates
        colors: numpy array with RGB values (0-1)
    """
    # Create pixel coordinate maps
    xmap, ymap = np.arange(depth_img.shape[1]), np.arange(depth_img.shape[0])
    xmap, ymap = np.meshgrid(xmap, ymap)
    
    # Calculate 3D coordinates
    points_z = depth_img / depth_scale
    points_x = (xmap - cx) / fx * points_z
    points_y = (ymap - cy) / fy * points_z
    
    # Filter valid points
    mask = (points_z > 0) & (points_z < 1.0)
    points = np.stack([points_x, points_y, points_z], axis=-1)
    points = points[mask].astype(np.float32)
    
    # Get colors
    color_img = color_img.astype(np.float32) / 255.0
    colors = color_img[mask].astype(np.float32)
    
    return points, colors
```

## 4. Directory Structure Requirements

Your project directory should have the following structure:

```
your_project/
├── gsnet.so                 # AnyGrasp binary module
├── lib_cxx.so               # License handling module
├── checkpoint_detection.tar # Trained model checkpoint
├── license/                 # License files directory
│   ├── licenseCfg.json
│   ├── your_name.public_key
│   ├── your_name.signature
│   └── your_name.lic
└── your_script.py           # Your own application code
```

## 5. Important Notes

1. The `gsnet.so` and `lib_cxx.so` files must match your Python version
2. The license files must be valid for your machine
3. Make sure your Python environment can find the required modules
4. AnyGrasp requires GPU acceleration with CUDA
5. For optimal results, use the default settings for grasp detection:
   - `apply_object_mask=True`
   - `dense_grasp=False`
   - `collision_detection=True`

## 6. Troubleshooting

- **License Issues**: Verify your machine ID and check that your license files are correctly placed
- **Import Errors**: Ensure paths to `gsnet.so` and `lib_cxx.so` are correct
- **No Detections**: Check that your point cloud data is within expected range and format
- **Runtime Errors**: Verify compatibility between your Python version and the .so files

For more details, refer to the original demo code in the AnyGrasp SDK repository.