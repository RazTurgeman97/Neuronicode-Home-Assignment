from setuptools import find_packages, setup

package_name = 'situational_awareness'

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
    maintainer='raz',
    maintainer_email='RazTurgeman97@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'awareness_node = situational_awareness.awareness_node:main',
        ],
    },
    options={
        'build_scripts': {
            'executable': '/home/raz/projects/Neuronicode-Home-Assignment/ros2_ws/venv/bin/python3'
        }
    },
)
