all: adxl345

adxl345:
	clang -O3 -Wl,-s -Wall -Werror -o adxl345 adxl345.c

clean:
	rm -f adxl345
