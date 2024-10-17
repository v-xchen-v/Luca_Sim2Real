from setuptools import setup

setup(
    name='my_package',
    version='0.1.0',
    packages=['luca_transformation'],
    install_requires=[
        'pytransform3d',
    ],
    python_requires='>=3.8',
)