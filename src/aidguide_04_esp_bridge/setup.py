import os
from glob import glob
from setuptools import find_packages, setup
package_name = 'aidguide_04_esp_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mimi',
    maintainer_email='mvladev@epsg.upv.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
   entry_points={
    'console_scripts': [
        'esp_bridge = aidguide_04_esp_bridge.esp_bridge:main',
        'esp32_reader = aidguide_04_esp_bridge.esp32_event_reader:main',
        'nav2_supervisor = aidguide_04_esp_bridge.nav2_supervisor:main',
        'test_nav_behavior = aidguide_04_esp_bridge.test_nav_behavior:main',
        'esp_reader_uart = aidguide_04_esp_bridge.esp_reader_uart:main',
    ],
},
)
