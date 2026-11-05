# Thin wrappers over the CMake presets (CMakePresets.json). `make check` is what every commit must pass.
JOBS ?= $(shell sysctl -n hw.ncpu 2>/dev/null || nproc)

.PHONY: check release asan venv clean

# The Python environment the presets build the module for (Python_EXECUTABLE), with pytest.
PYTHON ?= python3.11
VENV_STAMP = .venv/.requirements-installed
venv: $(VENV_STAMP)
$(VENV_STAMP): requirements-dev.txt
	test -x .venv/bin/python || $(PYTHON) -m venv .venv
	.venv/bin/pip install -q -r requirements-dev.txt
	touch $@

# Debug build with warnings as errors, then every test (GoogleTest and pytest).
check: $(VENV_STAMP)
	cmake --preset dev >/dev/null
	cmake --build --preset dev -j $(JOBS)
	ctest --preset dev -j $(JOBS)

# The same tests on the optimised build.
release: $(VENV_STAMP)
	cmake --preset release >/dev/null
	cmake --build --preset release -j $(JOBS)
	ctest --preset release -j $(JOBS)

# The C++ tests under AddressSanitizer and UndefinedBehaviorSanitizer.
asan:
	cmake --preset asan >/dev/null
	cmake --build --preset asan -j $(JOBS)
	ctest --preset asan -j $(JOBS)

clean:
	rm -rf build
