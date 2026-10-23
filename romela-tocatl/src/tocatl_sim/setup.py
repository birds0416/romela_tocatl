from glob import glob

from setuptools import find_packages, setup

package_name = 'tocatl_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py']),
        ('share/' + package_name + '/config', ['config/sim.yaml']),
        ('share/' + package_name + '/model', ['model/tocatl.xml']),
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matthew Hutchinson',
    maintainer_email='matthewhutchinson@ucla.edu',
    description='MuJoCo simulation of the TOCATL hexapod with keyboard walking',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mujoco_sim_node = tocatl_sim.mujoco_sim_node:main',
            'gait_node = tocatl_sim.gait_node:main',
            'teleop_keyboard = tocatl_sim.teleop_keyboard:main',
            'record_walk = tocatl_sim.record_walk:main',
            'build_model = tocatl_sim.model_builder:main',
        ],
    },
)
