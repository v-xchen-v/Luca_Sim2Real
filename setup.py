from setuptools import setup

setup(
    name='my_package',
    version='0.1.0',
    packages=['luca_transformation'],
    install_requires=[
        'pytransform3d',
        'opencv-python',
        'pyrealsense2',
        'torch', # only for load sim trajectory data, not necessary for the main functionality
    ],
    python_requires='>=3.8.10',
)