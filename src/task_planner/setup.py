from glob import glob
from setuptools import setup

package_name = 'task_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, 'vlm_grounder', 'vlm_grounder.src'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/pddl', glob('pddl/*.pddl')),
        ('share/' + package_name + '/pddl/hardware', glob('pddl/hardware/*.pddl')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/resource', glob('resource/*.png')),
        ('share/' + package_name + '/resource/descriptions', glob('resource/descriptions/*.txt')),
        ('share/' + package_name, ['README_vlm_pipeline.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yigit Yildirim',
    maintainer_email='yigyil@gmail.com',
    description='Fast Downward planning and UR10 plan submission to SEED.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = task_planner.planner_node:main',
            'vlm_grounder_node = task_planner.vlm_grounder_node:main',
            'two_objects_planner_node = task_planner.two_objects_planner_node:main',
            'pddl_to_seed = task_planner.pddl_to_seed:main',
            'pddl_to_seed_hardware = task_planner.hardware_to_seed:main',
            'pddl_to_seed_two_objects = task_planner.two_objects_to_seed:main',
        ],
    },
)
