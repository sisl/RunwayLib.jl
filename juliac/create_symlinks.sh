#!/bin/bash

# Create symlinks in julia/ directory pointing to libraries in julia/lib/ and julia/lib/julia/

# First, create symlinks for libraries in julia/lib/
echo "Creating symlinks for libraries in julia/lib/"
for lib in julia/lib/*; do
    if [ -f "$lib" ] || [ -L "$lib" ]; then
        libname=$(basename "$lib")
        if [ ! -e "julia/$libname" ]; then
            echo "Creating symlink for $libname"
            ln -s "lib/$libname" "julia/$libname"
        else
            echo "Symlink for $libname already exists"
        fi
    fi
done

# Then, create symlinks for libraries in julia/lib/julia/
echo "Creating symlinks for libraries in julia/lib/julia/"
for lib in julia/lib/julia/*; do
    if [ -f "$lib" ] || [ -L "$lib" ]; then
        libname=$(basename "$lib")
        if [ ! -e "julia/$libname" ]; then
            echo "Creating symlink for $libname"
            ln -s "lib/julia/$libname" "julia/$libname"
        else
            echo "Symlink for $libname already exists"
        fi
    fi
done

echo "Symlink creation complete."