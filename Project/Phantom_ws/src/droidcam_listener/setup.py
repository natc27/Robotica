from setuptools import find_packages, setup

package_name = 'droidcam_listener'

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
    maintainer='julip',
    maintainer_email='37085956+jdpulidoca@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
    'console_scripts': [
        'droidcam_listener = droidcam_listener.droidcam_listener:main',
    ],
    },
)
