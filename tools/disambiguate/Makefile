disambiguate: disambiguate.c
	gcc -g -o $@ $^ -lm

test: disambiguate
	./disambiguate | head -n 100
