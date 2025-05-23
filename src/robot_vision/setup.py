from setuptools import find_packages, setup

package_name = 'robot_vision'

setup(
    name=package_name,
    version='0.1.0',  # Updated version for clarity
    packages=find_packages(exclude=['test']),  # Standard way to find packages
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # If you have launch files, they should be installed via CMakeLists.txt
        # For example: install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/launch)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jayrn',
    maintainer_email='jayrn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'], # Uncomment if you have tests
    entry_points={
        'console_scripts': [
            'calculate_camera_pose_v2 = robot_vision.script.calculate_camera_pose_v2:main',
            'pixel_to_3d_node = robot_vision.robot_vision.pixel_to_3d:main'  # Adjusted path
        ]
    }
)