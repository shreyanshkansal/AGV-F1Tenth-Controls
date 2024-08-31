from setuptools import setup

package_name = 'f1tenth_stanley_controller'

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
    maintainer='root',
    maintainer_email='ninaad.desai1212@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stanley_controller = f1tenth_stanley_controller.controller_node:main',
            'get_waypoints = f1tenth_stanley_controller.get_waypoints:main'
        ],
    },
)
