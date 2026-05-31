import os
from glob import glob
from setuptools import setup

package_name = 'table_plane_extractor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='v4r',
    author_email='v4r@todo.todo',
    description='Service for table plane extraction',
    license='Apache-2.0',

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
         (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    
    entry_points={
    'console_scripts': [
        'table_objects_extractor_srv = table_plane_extractor.table_objects_extractor_srv:main',
        'table_plane_extractor_srv = table_plane_extractor.table_plane_extractor_srv:main',
        'table_objects_extractor_as = table_plane_extractor.table_objects_extractor_as:main',
    ],}
)