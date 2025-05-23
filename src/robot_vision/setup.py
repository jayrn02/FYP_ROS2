from setuptools import setup, find_packages

setup(
    name='robot_vision',
    version='0.1',
    packages=find_packages(),
    install_requires=[
        # ...existing requirements...
    ],
    entry_points={
        'console_scripts': [
            'calculate_camera_pose = robot_vision.calculate_camera_pose:main',
            'calculate_camera_pose_v2 = robot_vision.calculate_camera_pose_v2:main',  # Add this line
        ],
    },
)