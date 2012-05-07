.PHONY: clean, mrproper
.SUFFIXES:

CC = gcc
EXEC = opt_q
DEBUG = yes
LIBS = -lm

ifeq ($(DEBUG),yes)
	CFLAGS = -g -W -Wall
else
	CFLAGS = 
endif

all : $(EXEC)
ifeq ($(DEBUG),yes)
	@echo "Generation in debug mode"
else
	@echo "Generate in release mode"
endif

opt_q: opt_q.o
	$(CC) $^ -o $(EXEC) $(LIBS)

%.o : %.c
	$(CC) -c $< -o $@ $(CFLAGS)

exec: 
	@echo "Executing program with default args..."
	@sleep 0.4
	./$(EXEC) 1 4
clean:
	rm -rf *.o

mrproper: clean
	rm -rf programme_pi
