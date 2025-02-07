from setuptools import setup

package_name = 'solution_subscriber_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tailai Cheng', 
    maintainer_email='go34beq@mytum.de',  
    description='A package to subscribe to left and right task solutions.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'solution_subscriber = solution_subscriber_pkg.solution_subscriber:main',
            'marker_publisher = solution_subscriber_pkg.marker_publisher:main'
        ],
    },
)

