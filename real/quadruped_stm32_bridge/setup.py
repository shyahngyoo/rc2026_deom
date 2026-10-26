from glob import glob
from setuptools import find_packages, setup
name = 'quadruped_stm32_bridge'
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]),
        ('share/'+name, ['package.xml', 'README.md']),
        ('share/'+name+'/launch', glob('launch/*.launch.py')),
        ('share/'+name+'/config', glob('config/*.yaml')),
        ('share/'+name+'/protocol', glob('protocol/*.json'))],
    install_requires=['setuptools', 'pyserial'], zip_safe=True,
    maintainer='rc2026', maintainer_email='robot@example.com', license='Apache-2.0',
    description='Single-owner framed STM32 bridge with default-disabled physical output',
    tests_require=['pytest'], entry_points={'console_scripts':[
        'cybergear_bridge=quadruped_stm32_bridge.cybergear_node:main',
        'serial_gateway=quadruped_stm32_bridge.serial_gateway:main',
        'nav_velocity_adapter=quadruped_stm32_bridge.nav_velocity_adapter:main',
        'telemetry_adapter=quadruped_stm32_bridge.telemetry_adapter:main']})
