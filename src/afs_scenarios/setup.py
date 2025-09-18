from setuptools import setup
import os

package_name = 'afs_scenarios'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         ['launch/run_scenario.launch.py']),
        (os.path.join('share', package_name, 'scenarios'),
         ['scenarios/simple_hill.yaml', 'scenarios/windy_dual_ignitions.yaml']),
    ],
    install_requires=['setuptools','PyYAML'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Scenario orchestration for AFS simulation',
    license='Apache-2.0',
)
