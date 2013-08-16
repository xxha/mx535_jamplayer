/* By Yanjun Luo, for MX27 gpio test with application software. */

#ifndef __MX27_GPIO_H__
#define __MX27_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#include <stdio.h>
#include <stdarg.h>
#include <memory.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>

#define	PORT_1						0x1
#define	PORT_2						0x2
#define	PORT_3						0x3
#define	PORT_4						0x4
#define	PORT_5						0x5
#define	PORT_6						0x6
#define	PORT_7 						0x7

#define	OUTPUT						0x1
#define INPUT						0x0

#if 0
#define MUX_IO_P        29
#define MUX_IO_I        24
#define _MX27_BUILD_PIN(gp,gi) (((gp) << MUX_IO_P) | ((gi) << MUX_IO_I))

typedef enum iomux_pins {
	CPLD_JTAG_TCK = _MX27_BUILD_PIN(5, 13);
	CPLD_JTAG_TMS = _MX27_BUILD_PIN(5, 14);
	CPLD_JTAG_TDI = _MX27_BUILD_PIN(4, 14);
	CPLD_JTAG_TDO = _MX27_BUILD_PIN(5, 18);
} iomux_pin_name_t;
#endif

extern int setgpiodir(unsigned int port, unsigned int pin, unsigned int dir);
extern int setgpiodata(unsigned int port, unsigned int pin, unsigned int data);
extern int getgpiodata(unsigned int port, int pin, unsigned int * data);
extern int setpullup(unsigned int port, int pin, unsigned int pull);
extern int gpio_init();
extern int gpio_exit();


#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* __MX27_GPIO_H__ */
