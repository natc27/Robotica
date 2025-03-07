from setuptools import setup

package_name = 'droidcam_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tuemail@example.com',
    description='Publisher de imágenes usando DroidCam para ROS 2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'droidcam_publisher = droidcam_publisher.droidcam_publisher_node:main',
        ],
    },
)



