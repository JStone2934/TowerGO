from setuptools import setup, find_packages
setup(name='astribot_nav_sim', version='0.1.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/astribot_nav_sim']),('share/astribot_nav_sim',['package.xml'])],
 install_requires=['setuptools'], entry_points={'console_scripts':['simulator = astribot_nav_sim.simulator:main', 'experiment = astribot_nav_sim.experiment:main']})
