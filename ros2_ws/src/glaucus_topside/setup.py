from setuptools import setup

package_name = "glaucus_topside"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Glaucus I team",
    maintainer_email="team@example.com",
    description="Topside nodes: mission manager, coverage mapper, hull mapper (stretch).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_manager = glaucus_topside.mission_manager:main",
            "coverage_mapper = glaucus_topside.coverage_mapper:main",
            "hull_mapper = glaucus_topside.hull_mapper:main",
        ],
    },
)
