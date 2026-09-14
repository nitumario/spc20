# Host-side control-logic harness for charger.c + mppt.c (v0.41).
# Compiles the REAL firmware sources against stubbed hardware and drives
# them with a simple plant model. Not an electrical simulation.
#   make && ./harness
SRC   := ../..
CFLAGS := -std=c11 -Wall -Wextra -Wno-unused-function -DSPCBOARDAPI_H -I$(SRC) -I.

harness: charger.o mppt.o stubs.o harness.o
	$(CC) $^ -lm -o $@

charger.o: $(SRC)/charger.c stubs.h
	$(CC) $(CFLAGS) -include stubs.h -c $< -o $@
mppt.o: $(SRC)/mppt.c stubs.h
	$(CC) $(CFLAGS) -include stubs.h -c $< -o $@
stubs.o: stubs.c stubs.h
	$(CC) $(CFLAGS) -c $< -o $@
harness.o: harness.c stubs.h
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o harness
.PHONY: clean
