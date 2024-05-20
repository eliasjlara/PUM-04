from setuptools import find_packages, setup

package_name = 'lidar'

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
    maintainer='albin',
    maintainer_email='18600349+thulavall@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_reader = lidar.lidar_reader:main',
            'lidar_to_image = lidar.lidar_to_image:main',
        ],
    },
)
