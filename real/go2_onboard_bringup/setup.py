from glob import glob
from setuptools import find_packages, setup
setup(name='go2_onboard_bringup',version='0.1.0',packages=find_packages(exclude=['test']),
 data_files=[('share/ament_index/resource_index/packages',['resource/go2_onboard_bringup']),
 ('share/go2_onboard_bringup',['package.xml','README.md']),
 ('share/go2_onboard_bringup/launch',glob('launch/*.launch.py')),
 ('share/go2_onboard_bringup/config',glob('config/*'))],
 install_requires=['setuptools','PyYAML'],zip_safe=True,maintainer='rc2026',
 maintainer_email='robot@example.com',description='Measured external profiles and non-actuating onboard diagnostics',
 license='Apache-2.0',tests_require=['pytest'],entry_points={'console_scripts':[
 'operator_panel=go2_onboard_bringup.operator_panel:main',
 'configure_robot=go2_onboard_bringup.configure:main',
 'doctor=go2_onboard_bringup.doctor:main',
 'doctor_node=go2_onboard_bringup.doctor_node:main']})
