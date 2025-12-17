from setuptools import find_packages, setup
import os           # <-- ADDED
from glob import glob # <-- ADDED

package_name = 'yolo_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # This installs your best.onnx file into install/yolo_pkg/share/yolo_pkg/models/
        (os.path.join('share', package_name, 'models'), 
         glob(os.path.join('models', '*.onnx'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaize',
    maintainer_email='kaize@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo_pkg.yolo_detector:main', 
            'pursuit_controller = yolo_pkg.pursuit_controller:main',
        ],
    },
)
