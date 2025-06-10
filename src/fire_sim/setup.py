from setuptools import find_packages, setup

package_name = 'fire_sim'

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
    maintainer='rebwar',
    maintainer_email='rebwar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'sim_node = fire_sim.sim_node:main',
        'dashboard_node = fire_sim.dashboard_node:main',
    ],
},

)
