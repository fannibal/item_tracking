#!/usr/bin/env python
# coding: utf-8

import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="item_tracking",
    version="1.0.10",
    author="Fabrice POIRIER",
    author_email="fabrice.poirier@ensta-bretagne.org",
    description="A tracking algorithm for general purpose",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/fannibal/item_tracking.git",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: POSIX",
    ],
)
