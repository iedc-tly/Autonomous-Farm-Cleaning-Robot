from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'web_app_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'static'),
         glob(os.path.join(package_name, 'static', '*'))),
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn', 'websockets'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Web app bridge node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'app_node = web_app_node.app_node:main',
        ],
    },
)
