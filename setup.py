from setuptools import setup

package_name = 'pioneer3dx'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/pioneer3dx_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/pioneer3dx_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/pioneer3dx_webots.urdf']))
data_files.append(('share/' + package_name + '/models', ['models/pioneer3dx.urdf']))
data_files.append(('share/' + package_name + '/rviz', ['rviz/urdf_config.rviz']))
data_files.append(('share/' + package_name + '/config', ['config/ekf.yaml']))
data_files.append(('share/' + package_name, ['package.xml']))
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leanhchien',
    maintainer_email='leanhchien1903@gmail.com',
    description='Slam algorithm on pioneer-3dx platform in webots robot simulations',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_control = pioneer3dx.slam_control:main',
            'mapping_robot = pioneer3dx.mapping:main'
        ],
    },
)
