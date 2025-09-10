from setuptools import setup
package_name = 'afs_clock'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_clock.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Deterministic simulation clock for AFS',
    license='Apache-2.0',
    entry_points={'console_scripts': ['sim_clock = afs_clock.clock_node:main']},
)
