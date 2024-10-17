# Luca_Transformation

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
│
├── /calibration/              # Contains calibration-related files and utilities
│   ├── __init__.py
│   ├── calibration_data/      # Store calibration results (extrinsics, intrinsics, etc.)
│   │   ├── camera_intrinsics.yaml
│   │   ├── extrinsics.yaml
│   │   └── calibration_report.txt
│   ├── checkerboard_utils.py  # Utilities for handling checkerboard patterns
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
├── /scripts/                  # Standalone scripts
│   ├── run_calibration.py     # Example of how to run the calibration
│   └── visualize_transforms.py # Example of visualization of all frames
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
