from setuptools import setup

package_name = 'novatelzed_data_collection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autodrive',
    maintainer_email='autodrive@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_collection = novatelzed_data_collection.gps_collection:main',
            'ins_collection = novatelzed_data_collection.ins_collection:main',
            'ins_imu_collection = novatelzed_data_collection.ins_imu_collection:main',
            'zed_pose_collection = novatelzed_data_collection.zed_pose_collection:main',
            'zed_path_map_collection = novatelzed_data_collection.zed_path_map_collection:main',
        ],
    },
)
