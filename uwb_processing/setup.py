from setuptools import find_packages, setup


package_name = "uwb_processing"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="uwb_processing", exclude=["test"]),
    package_dir={"": "uwb_processing"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "scipy",
        "matplotlib",
        "opencv-python",
        "pyyaml",
    ],
    zip_safe=True,
    maintainer="OpenAI Codex",
    maintainer_email="codex@openai.com",
    description="Offline UWB radar processing, detection, and spectral analysis tools.",
    license="Proprietary",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "uwb_run_session = uwb_processing.run_session:main",
            "uwb_batch_eval = uwb_processing.batch_eval:main",
        ],
    },
)
