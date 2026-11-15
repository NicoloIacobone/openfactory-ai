from setuptools import find_packages, setup

package_name = 'openfactory_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/perception.yaml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py',
                                               'launch/ring_perception.launch.py',
                                               'launch/cube_perception.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nico',
    maintainer_email='nicolo.iacobone@gmail.com',
    description='Geometric RGB-D perception: object hypotheses in the input bin, the Rubik\'s cube in its nest.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scene_perception = openfactory_perception.scene_perception_node:main',
            'cube_perception = openfactory_perception.cube_perception_node:main',
        ],
    },
)
