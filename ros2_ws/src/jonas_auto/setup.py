from setuptools import setup

package_name = "jonas_auto"

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
    description="Modo automático de Jonas",
    license="MIT",
    entry_points={"console_scripts": ["auto = jonas_auto.auto_node:main", "sim = jonas_auto.sim:main",
                                     "sim_node = jonas_auto.sim_node:main"]},
)
