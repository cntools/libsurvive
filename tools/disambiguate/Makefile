disambiguate: disambiguate.c ../../src/disambiguator.c
	gcc -I../../src -Wall -Wextra -Werror -g -o $@ $^ -lm

test: disambiguate
	./disambiguate | head -n 100

disambiguate2: disambiguate2.c
	gcc -Wall -Wextra -Werror -g -o $@ $^

test2: disambiguate2
	./disambiguate2 | head -n 100
