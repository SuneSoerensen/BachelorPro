#!/bin/bash
echo "Removing all cmake-generated files"
rm -r bin/
rm -r CMakeFiles/
rm -r libs/
rm Makefile
rm cmake_install.cmake
rm CMakeCache.txt
