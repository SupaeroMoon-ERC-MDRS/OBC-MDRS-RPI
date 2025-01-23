from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'sim_mdrs'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    py_modules=[],
    data_files=[
        # Include launch, config, and urdf directories
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'urdf/meshes'), glob('urdf/meshes/*.stl')),
        (os.path.join('share', package_name, 'urdf/arm_meshes'), glob('urdf/arm_meshes/*.stl')),
        (os.path.join('lib', package_name), glob('sim_mdrs/udpcanpy.cpython-310-x86_64-linux-gnu.so')),
        (os.path.join('lib', package_name), glob('sim_mdrs/comms.dbc')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    # author='Your Name',
    # author_email='your_email@example.com',
    description='The sim_mdrs package for robot control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyboard_control = sim_mdrs.keyboard_control:main',
            'keyboard_arm = sim_mdrs.keyboard_arm:main',
            'remote_drive = sim_mdrs.remote_messages:main',
            'diff_controller = sim_mdrs.diff_controller:main'
        ],
    },

)
