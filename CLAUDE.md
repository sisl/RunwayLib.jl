## Package Structure

- This package has three main components. The Julia code (`src`), the compiled library (`juliac`), and some python bindings code (`poseest_py`). 
- To use the Julia functionality from python, you first need to recompile the Julia code to a C-library using the Makefile in the `juliac` directory. Notice that the library is then symlinked to the python `native` code directory.
- In particular, PYTHON TESTS WILL NOT PASS IF YOU DON'T RECOMPILE THE LIBRARY after fixing an issue in the julia code.
