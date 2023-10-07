from setuptools import find_packages, setup

package_name = 'my_pkg_py'

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
    maintainer='frivas',
    maintainer_email='rivascf@gmail.com',
    description='Paquete de prueba para ROS2 Humble (Python)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test = my_pkg_py.my_node:main"
        ],
    },
)
