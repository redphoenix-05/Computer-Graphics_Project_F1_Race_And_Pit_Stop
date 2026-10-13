@echo off
echo Compiling OpenGL Project...
g++ -std=c++17 -O2 src/main.cpp src/glad.c -Iinclude -Llib -lglfw3dll -lopengl32 -lglu32 -lgdi32 -lwinmm -o main.exe
if %ERRORLEVEL% EQU 0 (
    echo [SUCCESS] Build succeeded! Running main.exe...
    main.exe
) else (
    echo [ERROR] Build failed!
)

