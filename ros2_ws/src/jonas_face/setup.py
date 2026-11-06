from setuptools import setup

package_name = "jonas_face"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MiTo Olortegui Huaman",
    maintainer_email="molortegui@utec.edu.pe",
    description="Cara animada de Jonas",
    license="MIT",
    entry_points={
        "console_scripts": [
            "face = jonas_face.face_node:main",
            "demo = jonas_face.eyes:main",
        ],
    },
)
