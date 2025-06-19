from setuptools import find_packages, setup

package_name = 'neuronics_sa'

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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_mapper_node = neuronics_sa.neuronics_sa.fusion_mapper_node:main',
            'fusion_node = fast_sensor_fusion.fast_sensor_fusion:main',
            'fusion_node_Q = neuronics_sa.neuronics_sa.fusion_node_Q:main',
        ],
    },
)
