from setuptools import setup

package_name = "jonas_web"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_data={package_name: ["static/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="MiTo Olortegui Huaman",
    maintainer_email="molortegui@utec.edu.pe",
    description="Interfaz web de Jonas",
    license="MIT",
    entry_points={"console_scripts": ["server = jonas_web.server:main"]},
)
