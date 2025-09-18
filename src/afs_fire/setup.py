from setuptools import setup, find_packages
import os

package_name = 'afs_fire'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/fire_ca.launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/fire_ca.params.yaml']),
    ],
    install_requires=['setuptools','numpy'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Deterministic cellular-automaton fire spread node for AFS sim',
    license='Apache-2.0',
    entry_points={'console_scripts': ['fire_ca_node = afs_fire.fire_ca_node:main']},
)
