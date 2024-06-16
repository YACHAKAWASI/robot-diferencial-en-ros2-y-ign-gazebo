import os
from glob import glob
from setuptools import setup

package_name = 'robot_description'

def generate_paths(dir_name):
    paths = glob(dir_name + '/**', recursive=True)
    paths = [path for path in paths if os.path.isfile(path)]
    paths = sorted(list(set(['/'.join(path.split('/')[0:-1]) for path in paths])))
    paths = [(os.path.join('share', package_name, path), glob(path + '/*.*')) for path in paths]
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
    ] + generate_paths(dir_name='models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'display_odom = robot_description.display_odom:main',
            'display_scan = robot_description.display_scan:main',
            'display_imu = robot_description.display_imu:main',
        ],
    },
)