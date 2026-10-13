CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Iinclude
LDFLAGS = -Llib -lglfw3dll -lopengl32 -lglu32 -lgdi32 -lwinmm

SRC = src/main.cpp src/glad.c
TARGET = main.exe

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)

run: all
	./$(TARGET)

clean:
	del /f /q $(TARGET) 2>nul || rm -f $(TARGET)
