from setuptools import find_packages, setup

package_name = 'openfactory_manipulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/pick_place.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nico',
    maintainer_email='etaspam12@gmail.com',
    description='Pick and place at a known or perceived object pose: object selection, MoveIt 2, gripper.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_place_once = openfactory_manipulation.pick_place_once:main',
            'pick_perceived_once = openfactory_manipulation.pick_perceived_once:main',
        ],
    },
)
