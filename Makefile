# Thin wrappers over the CMake presets (CMakePresets.json). `make check` is what every commit must pass.
JOBS ?= $(shell sysctl -n hw.ncpu 2>/dev/null || nproc)

.PHONY: check release asan bench coverage gcc-check maps check-maps venv clean

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

# Optimised build, then every benchmark 5 times; Google Benchmark reports the mean, median, stddev and CV of each.
# BENCH_ARGS passes more flags, e.g. BENCH_ARGS=--benchmark_filter=Distances.
bench:
	cmake --preset release >/dev/null
	cmake --build --preset release -j $(JOBS) --target pathfinder_bench
	./build/release/bench/pathfinder_bench --benchmark_repetitions=5 --benchmark_report_aggregates_only=true $(BENCH_ARGS)

# The C++ tests under AddressSanitizer and UndefinedBehaviorSanitizer.
asan:
	cmake --preset asan >/dev/null
	cmake --build --preset asan -j $(JOBS)
	ctest --preset asan -j $(JOBS)

# Line, function, region and branch coverage of the library (src/, include/) by the GoogleTest suite and the CLI
# tests, with Clang's source-based coverage: each test process writes its own profile, they are merged, and the
# report reads the test binary and the CLI together. (The Python module isn't instrumented.)
LLVM_COV ?= $(shell xcrun --find llvm-cov 2>/dev/null || command -v llvm-cov)
LLVM_PROFDATA ?= $(shell xcrun --find llvm-profdata 2>/dev/null || command -v llvm-profdata)
coverage:
	cmake --preset coverage >/dev/null
	cmake --build --preset coverage -j $(JOBS)
	rm -rf build/coverage/profiles
	LLVM_PROFILE_FILE=$(CURDIR)/build/coverage/profiles/%p.profraw ctest --preset coverage -j $(JOBS) >/dev/null
	$(LLVM_PROFDATA) merge -sparse build/coverage/profiles/*.profraw -o build/coverage/tests.profdata
	$(LLVM_COV) report -instr-profile=build/coverage/tests.profdata build/coverage/tests/pathfinder_tests \
	  -object build/coverage/pathfinder $(CURDIR)/src $(CURDIR)/include

# This project's own sources compiled with GCC (Homebrew's g++-15 here), Debug and Release, warnings as errors:
# CI's GCC job catches warnings Clang doesn't give, and this finds most of them before a push (scripts/gcc_check.sh).
gcc-check: $(VENV_STAMP)
	scripts/gcc_check.sh

# The MovingAI MAPF benchmark maps and random scenarios, downloaded into data/ and checked against pinned SHA-256s
# (scripts/fetch_maps.py). They aren't committed; the tests that read them skip when they're missing.
maps:
	python3 -I scripts/fetch_maps.py

# Every benchmark scenario through `pathfinder check` (optimised build): a valid instance of all its entries, every
# goal reachable, every stated optimal length the octile distance (scripts/check_maps.sh).
check-maps:
	cmake --preset release >/dev/null
	cmake --build --preset release -j $(JOBS) --target pathfinder_cli
	scripts/check_maps.sh build/release/pathfinder data

clean:
	rm -rf build
