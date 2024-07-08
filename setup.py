from setuptools import find_packages, setup
from pathlib import Path

package_name = "commander"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="edvard",
    maintainer_email="mail@edvardsire.com",
    description="TODO: Package description",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            f"{path.stem} = {package}.{path.stem}:main"
            for package in find_packages()
            for path in Path(package).iterdir()
            if path.stem != "__init__"
        ],
    },
)
