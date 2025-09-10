from setuptools import setup
package_name = 'afs_env'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/env_from_rasters.launch.py']),
        ('share/' + package_name + '/config', [
            'config/summer_red.tiff',
            'config/summer_nir.tiff',
            'config/winter_red.tiff',
            'config/winter_nir.tiff',
            'config/dem.tif',
        ]),
    ],
    install_requires=['setuptools','numpy','scipy','rasterio','opencv-python-headless'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Environment layers from rasters for AFS sim',
    license='Apache-2.0',
    entry_points={'console_scripts': ['env_from_rasters = afs_env.env_from_rasters:main']},
)
