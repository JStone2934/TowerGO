from setuptools import setup, find_packages
setup(name='astribot_nav_bridge', version='0.1.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/astribot_nav_bridge']),('share/astribot_nav_bridge',['package.xml'])],
 install_requires=['setuptools'], entry_points={'console_scripts':['command_guard = astribot_nav_bridge.command_guard:main', 'prepare_bag = astribot_nav_bridge.bag_tools:main', 'inspect_state_bag = astribot_nav_bridge.inspect_state_bag:main']})
