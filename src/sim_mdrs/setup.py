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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'urdf/meshes'), glob('urdf/meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your_email@example.com',
    description='The sim_mdrs package for robot control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control = sim_mdrs.keyboard_control:main',
            'diff_controller = sim_mdrs.diff_controller:main'
        ],
    },

)
