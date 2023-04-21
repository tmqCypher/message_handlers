from setuptools import setup

package_name = 'robosub_messagers'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Quinn Cypher',
    maintainer_email='tmqCypher.git@gmail.com',
    description='Message composers/parsers for the robosub',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_composer = robosub_messagers.TelemetryComposer:main',
            'command_parser = robosub_messagers.CommandParser:main',
        ],
    },
)
