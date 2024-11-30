from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'bb8'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'urdf2'), glob('urdf2/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='parul',
    maintainer_email='prsingh@utexas.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_something = bb8.scripts.run_something:main',
            'spawn_bb8 = bb8.scripts.spawn_bb8:main',
            'test_command = bb8.scripts.test_command:main',
            'test_pendulum_init = bb8.scripts.test_pendulum_init:main',
            'test_diffdrive_init = bb8.scripts.test_diffdrive_init:main',
            'test_opti_1 = bb8.scripts.test_opti_1:main',
            'test_opti_2 = bb8.scripts.test_opti_2:main',
            'test_opti_3 = bb8.scripts.test_opti_3:main',
            'test_opti_5 = bb8.scripts.test_opti_5:main',
            'test_opti_6 = bb8.scripts.test_opti_6:main',
        ],
    },
)
