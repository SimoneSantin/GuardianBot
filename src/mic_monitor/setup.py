from setuptools import find_packages, setup

package_name = 'mic_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mic_monitor.launch.py']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lyra',
    maintainer_email='lyra@todo.todo',
    description='ROS2 microphone noise monitor node',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'noise_monitor = mic_monitor.noise_node:main',  
            'list_mics = mic_monitor.list_mics:main',
        ],
    },
)
