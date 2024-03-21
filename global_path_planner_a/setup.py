from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'global_path_planner_a'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['global_path_planner_a']),
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
)
