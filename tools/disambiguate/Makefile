disambiguate: disambiguate.c
	gcc -Wall -Wextra -Werror -g -o $@ $^ -lm

test: disambiguate
	./disambiguate | head -n 100
