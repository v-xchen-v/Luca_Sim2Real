from setuptools import setup

setup(
    name='luca_real2sim',
    version='0.1.0',
    packages=['luca_transformation'],
    install_requires=[
        'pytransform3d',
        'opencv-python',
        'pyrealsense2',
        'torch', # only for load sim trajectory data, not necessary for the main functionality,
        "open3d",
        'pyglet==1.5.27'
    ],
    python_requires='>=3.8.10',
)