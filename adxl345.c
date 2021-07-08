#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <dev/iicbus/iic.h>

#define die(s) { \
		perror(s); \
		exit(EXIT_FAILURE); \
	}

#define die4(func, cmd, name, addr) { \
		(void)fprintf(stderr, "%s(%s,%s,%02x): %s\n", \
			func, cmd, name, addr, strerror(errno)); \
		exit(EXIT_FAILURE); \
	}

static void adxl345_read(int fd, uint8_t addr, void *buf, uint16_t len)
{
	struct iic_msg msg[2];
	struct iic_rdwr_data rdwr;

	msg[0].slave = 0x3B;
	msg[0].flags = IIC_M_WR;
	msg[0].len = sizeof(addr);
	msg[0].buf = &addr;

	msg[1].slave = 0x3B;
	msg[1].flags = IIC_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;

	rdwr.msgs = msg;
	rdwr.nmsgs = 2;
	if (ioctl(fd, I2CRDWR, &rdwr) < 0)
		die4("ioctl", "I2CRDWR", "read", addr);
}

static void adxl345_write(int fd, uint8_t addr, uint8_t value)
{
	uint8_t buf[2];
	struct iic_msg msg;
	struct iic_rdwr_data rdwr;

	buf[0] = addr;
	buf[1] = value;

	msg.slave = 0x3A;
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;

	rdwr.msgs = &msg;
	rdwr.nmsgs = 1;
	if (ioctl(fd, I2CRDWR, &rdwr) < 0)
		die4("ioctl", "I2CRDWR", "write", addr);
}

#define ADXL345_DEVID       0x00
#define ADXL345_BW_RATE     0x2C
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_FIFO_CTL    0x38
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_DATA_X0     0x32

typedef struct {
	short x;
	short y;
	short z;
} ADXL345_ACCELERATION;

int main(int argc, char *argv[])
{
	int fd;
	uint8_t devid;
	struct timespec base, now;
	ADXL345_ACCELERATION acc;

	if ((fd = open("/dev/iic1", O_RDWR)) < 0)
		die("open");

	adxl345_read(fd, ADXL345_DEVID, &devid, sizeof(devid));
	(void)printf("devid=%2x\n", devid);

	adxl345_write(fd, ADXL345_BW_RATE, 0x0A); // 100Hz
	adxl345_write(fd, ADXL345_DATA_FORMAT, 0x00);
	adxl345_write(fd, ADXL345_FIFO_CTL, 0x00);
	adxl345_write(fd, ADXL345_POWER_CTL, 0x08);

	(void)clock_gettime(CLOCK_MONOTONIC, &base);
	for (;;) {
		adxl345_read(fd, ADXL345_DATA_X0, &acc, sizeof(acc));
		(void)printf("\r(%6d,%6d,%6d)", acc.x, acc.y, acc.z);

		base.tv_nsec += 10 * 1000 * 1000; // 10msec <= 100Hz
		if (1000 * 1000 * 1000 <= base.tv_nsec) {
			base.tv_nsec -= 1000 * 1000 * 1000;
			base.tv_sec++;
		}

		(void)clock_gettime(CLOCK_MONOTONIC, &now);
		if (now.tv_nsec < base.tv_nsec) {
			now.tv_nsec += 1000 * 1000 * 1000 - base.tv_nsec;
			now.tv_sec -= 1 + base.tv_sec;
		} else {
			now.tv_nsec -= base.tv_nsec;
			now.tv_sec -= base.tv_sec;
		}
		if (now.tv_sec < 0)
			continue;

		(void)nanosleep(&now, NULL);
	}

	if (close(fd) < 0)
		die("close");

	return (0);
}
