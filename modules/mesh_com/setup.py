from setuptools import setup

package_name = 'mesh_com'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/mesh_com.launch']),
        ('share/bin', ['scripts/mesh.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sad',
    maintainer_email='sad@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'mesh_subscriber = mesh_com.mesh_subscriber:main',
        'mesh_publisher = mesh_com.mesh_publisher:main',
        ],
    },
)
