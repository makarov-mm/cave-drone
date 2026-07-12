@echo off
rem Run from a "x64 Native Tools Command Prompt for VS"
if not exist build mkdir build
cl /nologo /O2 /EHsc /std:c++17 /W4 src\*.cpp /Fo:build\ /Fe:build\CaveDroneSim.exe ^
   /link opengl32.lib gdi32.lib user32.lib /SUBSYSTEM:WINDOWS
