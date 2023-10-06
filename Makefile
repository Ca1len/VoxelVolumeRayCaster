
.PHONY: prepare
prepare:
	@mkdir -p build

.PHONY: cmake
cmake: prepare
	@cd build && \
		cmake .. && \
		mv compile_commands.json ..

.PHONY: build
build:
	@cd build && make -j$(shell nproc)

.PHONY: run
run:
	@./build/RayCaster

.PHONY: clean
clean:
	@rm -rf build/*
