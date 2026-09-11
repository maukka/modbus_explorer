# ==============================================================================
# Cross-Platform C++ Makefile with Auto-Dependency Tracking
# Designed for MinGW / MSYS64 environments on Windows
# ==============================================================================

# 1. Compiler and flags with strict warnings and optimizations
CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor \
            -Wunused -Woverloaded-virtual -Wformat=2 -O2

# 2. Path definitions
# MSYS64 libmodbus- library path (adjust if necessary)
MODBUS_INC := C:/msys64/ucrt64/include/modbus
MODBUS_LIB := C:/msys64/ucrt64/lib

SRC_DIR    := src
INC_DIR    := include
BUILD_DIR  := build

# 3.Target executable name and object files
TARGET     := $(BUILD_DIR)/testi.exe

# Search automatically all .cpp files in the src/ directory
SRCS       := $(wildcard $(SRC_DIR)/*.cpp)

# Create corresponding .o files in the build/ directory
OBJS       := $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(SRCS))

# Automatically generate dependency files (.d) for each .o file
DEPS       := $(OBJS:.o=.d)

# Search for include directories and link libraries
INCLUDES   := -I$(MODBUS_INC) -I$(INC_DIR)
LDFLAGS    := -L$(MODBUS_LIB)
LDLIBS     := -lmodbus

# 4. Operating system detection for cross-platform compatibility
ifeq ($(OS),Windows_NT)
    RM := rm -rf
else
    RM := rm -rf
endif

# ==============================================================================
# Commands and rules
# ==============================================================================

# Default target: build the executable
all: $(TARGET)

# Program linking: link all object files into the final executable
$(TARGET): $(OBJS)
	@echo [LINK] Linkitetään valmis ohjelma: $@
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(OBJS) -o $(TARGET) $(LDFLAGS) $(LDLIBS)
	@echo [SUCCESS] Compilation and linking completed successfully. You can find the executable at: $(TARGET)

# Source compilation: compile each .cpp file into a corresponding .o file in the build/ directory	
# -MMD -MP flags generate dependency files (.d) for each .o file
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@echo [COMPILE] $< -\> $@
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -MMD -MP -c $< -o $@

# Debug-mode build: compile with debug flags and no optimizations
# Käyttö: make debug
debug: CXXFLAGS := -std=c++17 -g -O0 -DDEBUG_MODE -Wall -Wextra -Wpedantic
debug: clean $(TARGET)

# Clean	remove build artifacts and the build directory
clean:
	@echo [CLEAN] Remove build artifacts...
	$(RM) $(BUILD_DIR)

# Use auto-dependency tracking to include .d files for each .o file
-include $(DEPS)

.PHONY: all clean debug