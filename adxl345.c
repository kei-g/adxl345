#include <errno.h>
#include <fcntl.h>
#include <signal.h>
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

static void gpio_init(void);
static void gpio_exit(void);
static void gpio_signal(int sig);
static void NJU3714_write(uint16_t value);

#define GPIO_HI             7
#define GPIO_LO             10

#define NJU3714_CLR(v)      GPIO[(v)] = 1 << 22
#define NJU3714_STB(v)      GPIO[(v)] = 1 << 27
#define NJU3714_CLK(v)      GPIO[(v)] = 1 << 17
#define NJU3714_DAT(v)      GPIO[(v)] = 1 <<  4
#define NJU3714_ALL(v)      GPIO[(v)] = 1 << 4 | 1 << 17 | 1 << 27 | 1 << 22

static volatile uint32_t *GPIO = NULL;

static void gpio_init(void)
{
	int fd;
	void *addr;

	if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
		die("open");

	addr = mmap(
			NULL,
			4096,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			fd,
			0x20200000
		);
	if ((int)addr == -1)
		die("mmap");

	if (close(fd) < 0)
		die("close");

	GPIO = (volatile uint32_t *)addr;
	NJU3714_ALL(GPIO_LO);

	atexit(gpio_exit);
	signal(SIGINT, gpio_signal);
}

static void gpio_exit(void)
{
	void *addr;

	if (!GPIO)
		return;

	NJU3714_write(0);
	NJU3714_CLR(GPIO_LO);

	addr = (void *)GPIO;
	GPIO = NULL;

	if (munmap(addr, 4096) < 0)
		(void)perror("munmap");
}

static void gpio_signal(int sig)
{
	(void)printf("\n");
	exit(0);
}

static void NJU3714_write(uint16_t value)
{
	int i;
	NJU3714_STB(GPIO_HI);
	for (i = 0; i < 12; i++) {
		NJU3714_DAT(value & (1 << i) ? GPIO_HI : GPIO_LO);
		NJU3714_CLK(GPIO_HI);
		NJU3714_CLK(GPIO_LO);
	}
	NJU3714_STB(GPIO_LO);
}

#define die4(func, cmd, name, addr) { \
		(void)fprintf(stderr, "%s(%s,%s,%02x): %s\n", \
			func, cmd, name, addr, strerror(errno)); \
		exit(EXIT_FAILURE); \
	}

static void adxl345_read(int fd, uint8_t addr, uint8_t *buf, uint16_t len)
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

int main(argc, argv)
	int argc;
	char *argv[];
{
	int fd;
	uint8_t devid, buf[6];
	struct timespec base, now;
	short x, y, z;

	if ((fd = open("/dev/iic1", O_RDWR)) < 0)
		die("open");

	adxl345_read(fd, ADXL345_DEVID, &devid, sizeof(devid));
	(void)printf("devid=%2x\n", devid);

	adxl345_write(fd, ADXL345_BW_RATE, 0x0A); // 100Hz
	adxl345_write(fd, ADXL345_DATA_FORMAT, 0x00);
	adxl345_write(fd, ADXL345_FIFO_CTL, 0x00);
	adxl345_write(fd, ADXL345_POWER_CTL, 0x08);

	gpio_init();

	NJU3714_CLR(GPIO_HI);

	(void)clock_gettime(CLOCK_MONOTONIC, &base);
	for (;;) {
		adxl345_read(fd, ADXL345_DATA_X0, buf, sizeof(buf));
		x = *(short *)&buf[0];
		y = *(short *)&buf[2];
		z = *(short *)&buf[4];

#if 0
		(void)printf("(%6d,%6d,%6d)\n", x, y, z);
#else
		x = (192 - x) / 48;
		if (x < 0)
			x = 0;
		if (7 < x)
			x = 7;
		NJU3714_write(1 << x);
#endif

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
