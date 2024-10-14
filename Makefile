.PHONY: all
all: build

BUILD ?= "build"

.PHONY: build
build:
	mkdir -p ${BUILD}
	cd ${BUILD}; cmake ../src
	cd ${BUILD}; make

.PHONY: clean
clean:
	rm -rf ${BUILD}