from setuptools import find_packages, setup

package_name = 'tb3ctrl_py'

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
    maintainer='arrusr',
    maintainer_email='rivascf@gmail.com',
    description='Nodo demo para dibujar un circulo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go_cle_py = tb3ctrl_py.go_in_circle:main',
            'tb3_robot_state = tb3ctrl_py.tb3_robot_state:main'
        ],
    },
)
