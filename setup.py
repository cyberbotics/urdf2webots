import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name='urdf2webots',
    version='0.2.2',
    # scripts=[
    #     'urdf2webots.py',
    #     'gazebo_materials.py',
    #     'math_utils.py',
    #     'parserURDF.py',
    #     'writeProto.py'
    # ],
    author="Cyberbotics",
    author_email="support@cyberbotics.com",
    description="A converter between URDF and PROTO files.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/cyberbotics/urdf2webots",
    packages=setuptools.find_packages(),
    license='Apache License, Version 2.0',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development'
    ],
    install_requires=[
#       "pycollada >= 0.6",
#       "Pillow"
    ]
 )
