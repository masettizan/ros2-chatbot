from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'chat'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taylorem',
    maintainer_email='emilytaylorr22@gmail.com',
    description='a chatbot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'talker = chat.user_text_input:main',
		'listener = chat.user_text_listener:main',
		'diction = chat.user_diction:main',
        'openai_chatbot = chat.openai_chatbot:main',
        ],
    },
)
