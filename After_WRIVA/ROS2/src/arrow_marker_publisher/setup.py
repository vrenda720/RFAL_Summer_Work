from setuptools import find_packages, setup

package_name = 'arrow_marker_publisher'

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
    maintainer='rfal',
    maintainer_email='rfal@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'LA_arrow = arrow_marker_publisher.arrow_marker_node_LA:main',
            'AV_arrow = arrow_marker_publisher.arrow_marker_node_AV:main',
            'AV_arrow_curve = arrow_marker_publisher.arrow_marker_node_AV_Curve:main',
        ],
    },
)
