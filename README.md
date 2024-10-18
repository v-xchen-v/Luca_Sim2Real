# Luca_Transformation
A trajectory in generated with RL in simulator. -> A trajectory of real world the robot and directly apply.

## Changelog
- **0.1.0** (2024-10-17)
    - Initial release with core features: setting up coordinate frames in grasp task.

## Progress Overview

### Completed Features
- [x] Core functionality (e.g., Transformation between coordinate frames)
- [x] Compute T_table_to_camera with a checkerboard image.
- [x] Basic input/output handling
- [x] 2D plotting of landmarks

### Ongoing Development
- [ ] Add 3D plotting (`plot3d_landmarks`)
- [ ] Implement constraints (e.g., Max velocity/acceleration)
- [ ] Finalize PD controller gains tuning  
- [ ] Optimize transformations for points and axes

### Planned Features
- [ ] Integration with ROS topics (`/rm_driver/JointPos`)
- [ ] Refine force/angle handling in Isaac Sim
- [ ] Add unit tests and documentation for camera-world-robot transformations  
- [ ] Example scripts showcasing common workflows (in `example/` folder)

## Folder Structure
/robotics_project/
|
├── /camera_operations/               # Folder for camera operations
│   ├── __init__.py                   # Make it a Python package
│   ├── camera_capture.py             # Script to capture images from a camera
<!-- │   ├── camera_manager.py             # Manage multiple cameras
│   ├── video_stream.py               # Handle video streams from cameras -->
│   └── utils.py                      # Helper functions for camera operations
│
├── /calibration/              # Contains calibration-related files and utilities
│   ├── __init__.py
│   ├── calibration_data_MMDD/ # Backup histrical data 
│   ├── calibration_data/      # Store calibration results (extrinsics, intrinsics, etc.)
|   |   ├── /camera_intrinsics/
|   |   │   ├── dist.npy
|   |   │   └── mtx.npy
|   |   ├── /table_to_camera/ # Table-to-camera transformation, should be update often since that the historical data is useless once the table or calibration board is moved.
|   |   │   ├── table_to_camera.npy
|   |   │   ├── corner_visualization.npy
|   |   │   └── reprojection_error.txt
|   |   ├── /camera_to_robot/ # Precomputed camera-to-robot transformation
|   |   │   ├── cam2base_4x4.npy
|   |   │   └── reprojection_error.txt # Optional, the reprojection error of eye hand calibration
│   │   └── calibration_report.txt
│   ├── checkerboard_utils.py  # Utilities for handling checkerboard patterns
│   ├── calibration_precomputed_data_loader.py  # Utilities for loading precomputed calibration data when setup robot
│   ├── calibration_data_loader.py  # Utilities for loading calibration data computed during sim2real
<!-- │   └── calibrate.py           # Main script for running calibration -->
│   └── calibrate_board_to_camera.py  # Script to compute the transformation matrix
│
├── /coordinates/              # Contains coordinate frames and transformations
│   ├── __init__.py
│   ├── frames.py              # Definitions of coordinate frames (camera, robot, objects)
│   ├── transformations.py     # Transformation matrix creation and utilities
│   └── visualization.py       # Visualization of frames and transformations
│
├── /robot/                    # Robot-specific logic
│   ├── __init__.py
│   ├── robot_control.py        # Robot control logic (move joints, grasp objects)
│   └── robot_kinematics.py     # Robot kinematics calculations (forward/inverse)
│
├── /trajectory_processing/           # Sim2Real trajectory processing
│   ├── __init__.py                   # Makes it a package
│   ├── trajectory_adapter.py         # Adapt simulated trajectories for real robot
│   ├── trajectory_smoothing.py       # Smooth trajectories to avoid abrupt movements
│   ├── trajectory_utils.py           # Helper functions (e.g., loading trajectories)
│   └── test_trajectory.py            # Test trajectory processing logic
|
├── /scripts/                   # Standalone scripts
│   ├── run_calibration.py      # Example of how to run the calibration
│   ├── visualize_transforms.py # Example of visualization of all frames
│   ├── use_precomputed_data.py # Example of using calibration data
│   └── execute_trajectory.py   # New script to execute processed trajectory
│
├── /examples/                 # Examples folder for standalone demos
│   ├── vis_transforms.py      # Example of using transformation matrices
│   └── example_visualization.py  # Example of visualizing multiple frames
|
├── /debug/                    # debug folder for testing and troubleshooting
│   ├── debug_transformations.py  # Script to print and validate transformations
│   ├── debug_visualization.py    # Script to visualize frames with extra logging
│   └── debug_robot.py             # Script to test robot control commands and log errors
│
├── /tests/                    # Tests for calibration, transformations, and robot logic
│   ├── test_calibration.py
│   ├── test_transformations.py
│   └── test_robot_control.py
│
├── /docs/                     # Documentation (setup guides, API docs, etc.)
│   └── calibration_guide.md
│
├── requirements.txt           # List of dependencies (e.g., numpy, matplotlib, pytransform3d)
└── README.md                  # Overview of the project
