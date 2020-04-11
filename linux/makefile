CFLAGS=-c -Wall -O2 -I../src
LIBS = -lm -lpthread

all: libBitBang_I2C.a

libBitBang_I2C.a: BitBang_I2C.o
	ar -rc libBitBang_I2C.a BitBang_I2C.o ;\
	sudo cp libBitBang_I2C.a /usr/local/lib ;\
	sudo cp ../src/BitBang_I2C.h /usr/local/include

BitBang_I2C.o: BitBang_I2C.c ../src/BitBang_I2C.h ../src/BitBang_I2C.cpp
	$(CC) $(CFLAGS) -D_LINUX_ BitBang_I2C.c

clean:
	rm *.o libBitBang_I2C.a
