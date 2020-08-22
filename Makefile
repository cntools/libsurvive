all: build

build:
	mkdir -p bin
	cd bin && cmake .. && cmake --build . 

install: build
	cd bin && cmake --build . --target install

clean:
	rm -rf bin

.PHONY: build install clean
