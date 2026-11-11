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
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nico',
    maintainer_email='etaspam12@gmail.com',
    description='Geometric RGB-D perception: object hypotheses in the input bin from depth and TF.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scene_perception = openfactory_perception.scene_perception_node:main',
        ],
    },
)
