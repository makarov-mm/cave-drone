#!/bin/sh
# MinGW-w64 build (native on Windows via MSYS2, or cross-compile on Linux)
CXX=${CXX:-x86_64-w64-mingw32-g++}
mkdir -p build
$CXX -O2 -std=c++17 -Wall -Wextra src/*.cpp -o build/CaveDroneSim.exe \
    -lopengl32 -lgdi32 -luser32 -static -mwindows
