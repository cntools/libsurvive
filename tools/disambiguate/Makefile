disambiguate: disambiguate.c ../../src/disambiguator.c
	gcc -Wall -Wextra -Werror -g -o $@ $^ -lm

test: disambiguate
	./disambiguate | head -n 100
