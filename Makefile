all: build

build:
	mkdir -p bin
	cd bin && cmake .. && cmake --build . -j 4

install: build
	cd bin && cmake --build . --target install

clean:
	rm -rf bin

