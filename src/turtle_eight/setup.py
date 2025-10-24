from setuptools import find_packages, setup

package_name = 'turtle_loop'

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
    maintainer='Your Name', # Zmień na swoje imię
    maintainer_email='your.email@example.com', # Zmień na swój email
    description='ROS 2 package for a turtlesim infinity loop.',
    license='Apache-2.0', # Możesz zmienić licencję
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'infinity_loop_node = turtle_loop.infinity_loop_node:main'
        ],
    },
)

