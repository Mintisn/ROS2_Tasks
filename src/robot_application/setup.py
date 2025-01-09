from setuptools import find_packages, setup

package_name = 'robot_application'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zychen',
    maintainer_email='3257161455@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "waypoint=robot_application.waypoint:main",
            "waypoint2=robot_application.waypoint2:main",
            "init_robot_pose=robot_application.init_robot_pose:main",
            "init_robot_pose1=robot_application.init_robot_pose1:main",
            "init_robot_pose2=robot_application.init_robot_pose2:main",
            "init_pose=robot_application.init_pose:main",
            "init_pose2=robot_application.init_pose2:main",
        ],
    },
)
