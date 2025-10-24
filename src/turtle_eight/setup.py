from setuptools import find_packages, setup

package_name = 'turtle_eight'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bartosz Kondratowicz',
    maintainer_email='bartkon2006@gmail.com',
    description='ROS 2 package for a turtlesim infinity loop.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'infinity_loop_node = turtle_eight.infinity_loop_node:main'
        ],
    },
)

