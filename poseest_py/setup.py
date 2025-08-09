#!/usr/bin/env python3
"""
Setup script for poseest Python package.
This package provides Python bindings for runway pose estimation using a Julia/C library.
"""

import os
import shutil
import platform
from pathlib import Path
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext


class BundleNativeLibraries(build_ext):
    """Custom build command to bundle native libraries."""
    
    def run(self):
        """Bundle the native libraries with the Python package."""
        # Source directory containing the Julia compiled libraries
        source_dir = Path(__file__).parent.parent / "RunwayLibCompiled"
        
        # Target directory in the package
        package_dir = Path(self.build_lib) / "poseest" / "native"
        
        if source_dir.exists():
            print(f"Bundling native libraries from {source_dir}")
            
            # Copy the entire RunwayLibCompiled structure
            if package_dir.exists():
                shutil.rmtree(package_dir)
            
            shutil.copytree(source_dir, package_dir)
            print(f"Native libraries bundled to {package_dir}")
        else:
            print(f"Warning: Source directory {source_dir} not found!")
            print("Make sure to build the Julia library first using 'make' in the juliac directory")
        
        # Call the original build_ext
        super().run()


if __name__ == "__main__":
    setup(
        cmdclass={
            'build_ext': BundleNativeLibraries,
        },
    )