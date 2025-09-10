from setuptools import setup
package_name = 'afs_world'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/world.launch.py']),
        ('share/' + package_name + '/config', ['config/map_40x40.png']),
    ],
    install_requires=['setuptools', 'Pillow', 'numpy'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='World publisher for AFS simulation (PNG → OccupancyGrid)',
    license='Apache-2.0',
    entry_points={'console_scripts': ['map_pub = afs_world.map_pub:main']},
)
