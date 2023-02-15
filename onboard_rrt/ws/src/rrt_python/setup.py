from setuptools import setup

package_name = 'rrt_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Justin Cooper',
    maintainer_email='coope263@purdue.edu',
    description='RRT_python package to run with Turtlesim ',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'default = rrt_python.default_turtle:main',
            'rrt = rrt_python.rrt_python:main',
        ],
    },
)
