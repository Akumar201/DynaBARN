from setuptools import setup
from glob import glob
import os

package_name = 'dynabarn_turtlebot'

launch_files = glob(os.path.join('launch', '*.launch.py'))
urdf_files   = glob(os.path.join('urdf', '*'))  # keep if you have URDFs

def data_files_for_tree(root_dir, dest_prefix):
    entries = []
    for dirpath, _, filenames in os.walk(root_dir):
        if filenames:
            dest = os.path.join(dest_prefix, os.path.relpath(dirpath, root_dir))
            sources = [os.path.join(dirpath, f) for f in filenames]
            entries.append((dest, sources))
    return entries

data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    (f'share/{package_name}', ['package.xml']),
    (f'share/{package_name}/launch', launch_files),
    (f'share/{package_name}/urdf', urdf_files),
]
# Install models recursively, preserving subfolders like tb3_burger_cam/
data_files += data_files_for_tree('models', f'share/{package_name}/models')

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,  # <-- use the one we built above
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ankit Kumar',
    maintainer_email='ankitkumar3.1428@gmail.com',
    description='Dynabarn TurtleBot demos',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'wander = dynabarn_turtlebot.wander:main',
            'end_to_end = dynabarn_turtlebot.end_to_end:main',
            'imagedetector = dynabarn_turtlebot.imagedetector:main',
        ],
    },
)