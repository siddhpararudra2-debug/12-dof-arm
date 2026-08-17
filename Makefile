BUILD_DIR ?= build
CMAKE_FLAGS ?= -DCMAKE_BUILD_TYPE=Release

.PHONY: all install build test sim hardware hybrid clean lint format

all: build

install:
	cmake -S . -B $(BUILD_DIR) $(CMAKE_FLAGS)
	cmake --build $(BUILD_DIR) --target install

build:
	cmake -S . -B $(BUILD_DIR) $(CMAKE_FLAGS)
	cmake --build $(BUILD_DIR) -j2

test: build
	ctest --test-dir $(BUILD_DIR) --output-on-failure

sim: build
	./$(BUILD_DIR)/aether12_demo

hardware:
	@echo "Physical hardware is intentionally disabled until a reviewed transport adapter is configured."
	@echo "Use the RealHardware adapter as a fail-safe boundary; do not bypass SafetyManager."

hybrid:
	@echo "Hybrid orchestration is an integration concern; use the C++ core with real state and simulated planning."

clean:
	rm -rf $(BUILD_DIR)

lint:
	cmake -S . -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=Debug
	cmake --build $(BUILD_DIR) --clean-first -j2

format:
	@command -v clang-format >/dev/null && clang-format -i include/aether12/*.hpp src/*.cpp tests/*.cpp examples/*.cpp || echo "clang-format is not installed; formatting left unchanged."
