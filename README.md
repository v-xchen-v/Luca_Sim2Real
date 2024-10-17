# Luca_Transformation

## Changelog
- **0.1.0** (2024-10-17)
    - Initial release with core features: setting up coordinate frames in grasp task.


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
│   └── calibrate.py           # Main script for running calibration
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
├── /scripts/                  # Standalone scripts and examples
│   ├── run_calibration.py     # Example of how to run the calibration
│   └── visualize_transforms.py # Example of visualization of all frames
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
