# Project
BIN_DIR := bin
TARGET := $(BIN_DIR)/fp

# Layout
SRC_DIR := src
INC_DIR := include
BUILD_DIR := build

# Gurobi 12.0.3
GRB_DIR := gurobi1203/linux64
GRB_INC := $(GRB_DIR)/include
GRB_LIB := $(GRB_DIR)/lib

# Toolchain
CXX := g++
CXXFLAGS := -O3 -std=c++17 -I$(INC_DIR) -I$(GRB_INC) -MMD -MP
LDFLAGS := -L$(GRB_LIB) -Wl,-rpath,'$$ORIGIN/../$(GRB_LIB)'
LIBS := -lgurobi_c++ -lgurobi120 -lpthread -lm

# Sources and Objects
SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))
DEPS := $(OBJS:.o=.d)

# Targets
.PHONY: all clean release

all: $(TARGET)

$(TARGET): $(OBJS)
	@mkdir -p $(BIN_DIR)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LIBS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

release: CXXFLAGS := -O3 -DNDEBUG -std=c++17 -I$(INC_DIR) -I$(GRB_INC) -MMD -MP
release: clean all

clean:
	rm -r $(BUILD_DIR) $(BIN_DIR)

-include $(DEPS)
