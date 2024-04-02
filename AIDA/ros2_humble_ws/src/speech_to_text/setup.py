from setuptools import find_packages, setup

package_name = 'speech_to_text'

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
            'faster_whisper_node = speech_to_text.faster_whisper_node:main',
            'receive_str_result = speech_to_text.receive_str_result:main',
        ],
    },
)
