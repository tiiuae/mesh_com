from setuptools import setup

package_name = 'mesh_com'
submodules = "mesh_com/src"
rid_submodules = "mesh_com/utm_ussp_service"
rid_tx_submodules = "mesh_com/utm_ussp_service/transport/"
astm_submodules = "mesh_com/utm_ussp_service/astm"

setup(
    name=package_name,
    version='0.5.1',
    packages=[package_name, submodules, rid_submodules, rid_tx_submodules, astm_submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/mesh_com.launch']),
        ('share/bin', ['../../common/scripts/mesh-ibss.sh']),
        ('share/bin', ['../../common/scripts/mesh-11s.sh']),
        ('share/' + package_name, ['mesh_com/utm_ussp_service/ussp.yaml']),
       # ('share/bin', ['../../common/scripts/mesh-11s-mr.sh']),
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
        'mesh_executor = mesh_com.mesh_executor:main',
        'rid_ussp_service = mesh_com.utm_ussp_service.rid_location_subscriber:main',
        'rid_queue_test = mesh_com.utm_ussp_service.rid_queue_test:main'
        ],
    },
)
