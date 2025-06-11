from setuptools import find_packages, setup

package_name = 'autonomus_turtle'

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
    maintainer='sameer',
    maintainer_email='sameerdnadig@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "draw_square=autonomus_turtle.square:main",
            "ping_pong=autonomus_turtle.ping_pong1:main",
            "ping_pong1=autonomus_turtle.ping_pong2:main"

        ],
    },
)
