"""
Native library bundling and loading for poseest.

This module handles the detection and loading of the bundled Julia/C libraries.
"""

import os
import sys
import ctypes
import platform
from pathlib import Path
from typing import Optional

def get_library_path() -> Path:
    """
    Get the path to the bundled native libraries.
    
    Returns:
        Path to the directory containing the native libraries
        
    Raises:
        RuntimeError: If the native libraries cannot be found
    """
    # Get the directory where this module is located
    module_dir = Path(__file__).parent
    
    # Check if libraries exist in the expected location
    runway_lib_dir = module_dir / "RunwayLibCompiled"
    lib_dir = runway_lib_dir / "lib"
    if not lib_dir.exists():
        raise RuntimeError(
            f"Native libraries not found at {lib_dir}. "
            "Make sure the package was built correctly."
        )
    
    return runway_lib_dir


def find_shared_library(name: str, search_paths: list[Path]) -> Optional[Path]:
    """
    Find a shared library by name in the given search paths.
    
    Args:
        name: Base name of the library (e.g., "poseest")  
        search_paths: List of directories to search
        
    Returns:
        Path to the library if found, None otherwise
    """
    # Determine file extension based on platform
    system = platform.system().lower()
    if system == "windows":
        extensions = [".dll"]
    elif system == "darwin":
        extensions = [".dylib", ".so"]
    else:  # Linux and others
        extensions = [".so"]
    
    # Try different naming conventions
    prefixes = ["lib", ""]
    
    for search_path in search_paths:
        for prefix in prefixes:
            for ext in extensions:
                # Try exact name first
                lib_name = f"{prefix}{name}{ext}"
                lib_path = search_path / lib_name
                if lib_path.exists():
                    return lib_path
                
                # Try versioned names (e.g., libposeest.so.1)
                for version in ["1", "1.0"]:
                    versioned_name = f"{lib_name}.{version}"
                    versioned_path = search_path / versioned_name
                    if versioned_path.exists():
                        return versioned_path
    
    return None


def load_poseest_library() -> ctypes.CDLL:
    """
    Load the poseest shared library.
    
    Returns:
        Loaded ctypes library object
        
    Raises:
        RuntimeError: If the library cannot be found or loaded
    """
    native_dir = get_library_path()
    
    # Search paths for the library
    search_paths = [
        native_dir / "lib",
        native_dir / "lib" / "julia",
    ]
    
    # Find the library
    lib_path = find_shared_library("poseest", search_paths)
    if lib_path is None:
        raise RuntimeError(
            f"Could not find libposeest shared library in {search_paths}. "
            "Available files: " + str(list((native_dir / "lib").glob("*")))
        )
    
    # Set up environment for library loading
    setup_library_environment(native_dir)
    
    try:
        # Load the library
        library = ctypes.CDLL(str(lib_path))
        return library
    except Exception as e:
        raise RuntimeError(f"Failed to load library {lib_path}: {e}")


def setup_library_environment(native_dir: Path) -> None:
    """
    Set up the environment variables needed for the library to work.
    
    Args:
        native_dir: Path to the native library directory
    """
    # Set JULIA_DEPOT_PATH to point to our bundled artifacts
    julia_depot = native_dir / "share" / "julia"
    if julia_depot.exists():
        os.environ["JULIA_DEPOT_PATH"] = str(julia_depot)
    
    # Add library directories to the library path
    lib_dir = native_dir / "lib"
    julia_lib_dir = lib_dir / "julia"
    
    # Platform-specific library path setup
    system = platform.system().lower()
    
    if system == "windows":
        # On Windows, add to PATH
        current_path = os.environ.get("PATH", "")
        new_paths = [str(lib_dir), str(julia_lib_dir)]
        for path in new_paths:
            if path not in current_path:
                os.environ["PATH"] = f"{path}{os.pathsep}{current_path}"
                
    else:
        # On Unix-like systems, set LD_LIBRARY_PATH as backup
        # (RPATH should handle this, but just in case)
        current_path = os.environ.get("LD_LIBRARY_PATH", "")
        new_paths = [str(lib_dir), str(julia_lib_dir)]
        for path in new_paths:
            if path not in current_path:
                if current_path:
                    os.environ["LD_LIBRARY_PATH"] = f"{path}{os.pathsep}{current_path}"
                else:
                    os.environ["LD_LIBRARY_PATH"] = path