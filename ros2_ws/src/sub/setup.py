from setuptools import find_packages, setup

package_name = 'sub'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cam=sub.cam:main",
            "lidar=sub.lidar:main",
            "imu=sub.imu:main",
            "utm=sub.utm:main",
            "lon=sub.lon:main",
            "lat=sub.lat:main",
            "pcd_sub=sub.pcd_sub:main",
            "lidar_clu=sub.lidar_clu:main",
            "PointCloudSubscriber=sub.PointCloudSubscriber:main",
            "webcam=sub.webcam:main",
            "camrviz=sub.camrviz:main"
            
            
            
        ],
    },
)
