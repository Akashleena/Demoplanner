from setuptools import find_packages, setup

package_name = 'warehouse_demo_py'

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
    maintainer='aleenatron',
    maintainer_email='akash13leena@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'add_warehouse_scene = warehouse_demo_py.add_warehouse_scene:main',
	'plan_warehouse_pick = warehouse_demo_py.plan_warehouse_pick:main',
    'warehouse_picker = warehouse_demo_py.warehouse_picker:main',
    ],
},
)
