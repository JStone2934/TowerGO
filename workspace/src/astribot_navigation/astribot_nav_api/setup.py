from setuptools import setup, find_packages
setup(name='astribot_nav_api', version='0.1.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/astribot_nav_api']),('share/astribot_nav_api',['package.xml'])],
 install_requires=['setuptools'], entry_points={'console_scripts':['navigate = astribot_nav_api.navigator:main']})
