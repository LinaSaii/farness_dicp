# farness_dicp — Doppler-ICP ROS 2 Node

##  Overview
This package implements a **Doppler-ICP LiDAR frame stitching pipeline** in ROS 2.  
It loads sequential LiDAR frames, aligns them using a Doppler-constrained ICP algorithm, and publishes stitched clouds, poses, and motion dynamics for real-time visualization.

**Core Features**
- Preprocessing: filtering, downsampling, normal estimation  
- Doppler-ICP alignment with velocity constraints  
- Stitched point cloud publication (`/stitched_cloud`)  
- Current pose estimation (`/icp_pose`)  
- Trajectory history (`/icp_trajectory`)  
- Linear acceleration & angular velocity estimation  

---

##  Project Structure

ws_new/
├── src/
│ └── farness_dicp/
│ ├── config/
│ │ └── dicp_params.yaml # Node parameters
│ ├── farness_dicp/
│ │ ├── init.py
│ │ └── main.py # Main ROS 2 node (Doppler-ICP)
│ ├── launch/
│ │ └── dicp_stitcher.launch.py # Launch file
│ ├── resource/
│ │ └── farness_dicp
│ ├── test/ # Unit & integration tests
│ ├── package.xml
│ ├── setup.py
│ ├── setup.cfg
│ └── README.md

---

##  Installation

1. Clone the repository into your ROS 2 workspace:
   
   cd ~/ws_new/src
   git clone <your_repo_url> farness_dicp

2. Build the package:

   cd ~/ws_new
   colcon build --packages-select farness_dicp
   source install/setup.bash

3. Confirm installation:

   ros2 pkg list | grep farness_dicp

##   Usage
+ Option 1 - Run directly with parameters
You can run the node directly and pass parameters from the command line.  
For example, to load CSV frames from `/home/farness/Téléchargements/csv_point_clouds`:


ros2 run farness_dicp farness_dicp_node \
  --ros-args \
  -p frames_directory:=/home/farness/Téléchargements/csv_point_clouds \
  -p publish_rate:=10

Make sure the directory contains .csv LiDAR frames with the format:

x, y, z, v_radial


+ option 2 - Launch via launch file

ros2 launch farness_dicp dicp_stitcher.launch.py

## Visualization in Foxglove Studio

This package comes with a preconfigured **Foxglove Studio layout** (`voyant lidar.json`) >
To visualize the stitched point cloud, pose, and trajectory:

1. **Run the Doppler-ICP node**:
   
   ros2 run farness_dicp farness_dicp_node \
     --ros-args \
     -p frames_directory:=/home/farness/Téléchargements/csv_point_clouds

2. **Start the Foxglove bridge in another terminal**
ros2 run foxglove_bridge foxglove_bridge --port 8765

3. **Open Foxglove Studio**
- Go to Foxglove Studio
- Add a new connection : WebSocket.
- Connect to: ws://localhost:8765
- Load the provided layout file: Go to File → Open Layout…
- Select: <your_workspace>/ws_new/src/farness_dicp/foxglove/voyant_lidar.json


4. **The layout will automatically show**

- 3D panel: /stitched_cloud (stitched point cloud) and /icp_trajectory (trajectory history)

- Pose marker: /icp_pose (current estimated pose)

- Plots: /linear_acceleration and /angular_velocity (x, y, z)

- Grid layer for spatial reference

## Published Topics

/stitched_cloud : sensor_msgs/PointCloud2

/icp_pose ; geometry_msgs/PoseStamped

/icp_trajectory : geometry_msgs/PoseArray

/linear_acceleration : geometry_msgs/Vector3Stamped

/angular_velocity : geometry_msgs/Vector3Stamped

## Parameters (config/dicp_params.yaml)

frames_directory : path to LiDAR frame CSV/inputs

publish_rate : frequency of publishing (Hz)

downsample_factor : voxel/downsampling factor

max_iterations : max ICP iterations

icp_tolerance : convergence threshold

lambda_doppler_start / end : Doppler constraint weights

reject_outliers : enable/disable outlier filtering

max_corr_distance : nearest-neighbor threshold

min_inliers : minimum correspondences for stability

(See dicp_params.yaml for the full list of tunable parameters.)

## Testing


colcon test --packages-select farness_dicp
colcon test-result --verbose

## Development Notes

-The Doppler-ICP stitching pipeline has been implemented and validated using
CSV files with the structure:
                             x, y, z, v_radial
-Each file corresponds to a single LiDAR frame.
-The stitched cloud is published in ROS 2, where v_radial is currently mapped into
the intensity field for visualization.

+ Identified Limitations

-The number of stitched frames grows indefinitely, which can lead to memory
saturation.
-The CSV format only supports v_radial, and richer LiDAR data (vx, vy, vz,
intensity,timestamp) is not yet exploited.

+ Next Steps

*Frame Limiting Parameter
-Introduce a parameter n to restrict the number of stitched frames kept in memory.
*Raw Data Integration :
-Modify the code so that it accepts .bin files as input instead of CSV directly:

==> A conversion module will be implemented inside the pipeline to automatically
transform .bin frames into the extended CSV forma

## License

MIT License © 2025 Farness AI