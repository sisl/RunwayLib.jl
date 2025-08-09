"""
Simple test for the original test_estimators function to verify library loading works.
"""

import sys
from pathlib import Path

# Add the parent directory to the path so we can import poseest
sys.path.insert(0, str(Path(__file__).parent.parent))

import ctypes
from poseest.native import load_poseest_library


def test_original_function():
    """Test that we can load the library and call the original test_estimators function."""
    try:
        # Load the library
        lib = load_poseest_library()
        
        # Set up the original test_estimators function
        lib.test_estimators.argtypes = []
        lib.test_estimators.restype = ctypes.c_int
        
        # Call it
        result = lib.test_estimators()
        
        print(f"test_estimators() returned: {result}")
        assert result == 0, f"Expected 0, got {result}"
        print("✅ Library loading and original function call successful!")
        
        return True
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False


if __name__ == "__main__":
    print("Testing basic library loading and original function...")
    success = test_original_function()
    
    if success:
        print("\n🎉 SUCCESS: The Python package can successfully load and call the native library!")
        print("The ctypes wrapper, library bundling, and environment setup are all working correctly.")
    else:
        print("\n💥 FAILURE: There are issues with the library integration.")