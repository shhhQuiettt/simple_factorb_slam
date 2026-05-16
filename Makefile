CFLAGS = -Wall -Wextra -O2 -ggdb -pedantic
LDFLAGS = -lm -lraylib
CC = gcc


sim: sim.c
	$(CC) $(CFLAGS) -o sim sim.c $(LDFLAGS)

run: sim
	./sim

