/* By Lindell Xu, for MX535 gpio test with application software. */

#include "gpio.h"

/* Memory mapping definitions */
#define GPIO_MAP_SIZE 0x80000UL
#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

#define IOMUX_MMAP_ADDR	0x53FA8000
#define MMAP_BASE_ADDR	0x53F84000

#define	PORT_OFFSET_INC		0x4000
#define HIGH_PORT_SHIFT 	0x44000

#define PORT1_OFFSET_ADDR	0x0000
#define PORT2_OFFSET_ADDR	0x4000
#define PORT3_OFFSET_ADDR	0x8000
#define PORT4_OFFSET_ADDR	0xC000
#define PORT5_OFFSET_ADDR	0x58000
#define PORT6_OFFSET_ADDR	0x5C000
#define PORT7_OFFSET_ADDR	0x60000

#define GPIO_REG_DR 		0X00
#define GPIO_REG_GDIR 		0X04
#define GPIO_REG_PSR 		0x08
#define GPIO_REG_ICR1 		0x0C
#define GPIO_REG_ICR2 		0x10
#define GPIO_REG_IMR 		0x14
#define GPIO_REG_ISR 		0x18
#define GPIO_REG_EDGE_SEL 	0x1C

#define JTAG_TCK_PATA_DA_2_GPIO8_PORT7 		0x298
#define JTAG_TDO_PATA_DATA0_GPIO0_PORT2 	0x2A4
#define JTAG_TMS_PATA_DATA1_GPIO1_PORT2 	0x2A8
#define JTAG_TMS_PATA_DATA2_GPIO2_PORT2 	0x2AC

#define MUX_MODE_GPIO 		0x1

volatile void *map_base = NULL;
volatile void *iomux_map_base = NULL;
int fd = 0;

int config_iomux_gpio_mode()
{
	iomux_map_base = mmap((void *)0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, IOMUX_MMAP_ADDR & ~MAP_MASK);
	if(iomux_map_base == (void *) -1){
		printf("Error mapping iomux memory\n");
		return -1;
	}

	*((unsigned long*)(iomux_map_base + JTAG_TCK_PATA_DA_2_GPIO8_PORT7)) = MUX_MODE_GPIO;
	*((unsigned long*)(iomux_map_base + JTAG_TDO_PATA_DATA0_GPIO0_PORT2)) = MUX_MODE_GPIO;
	*((unsigned long*)(iomux_map_base + JTAG_TMS_PATA_DATA1_GPIO1_PORT2)) = MUX_MODE_GPIO;
	*((unsigned long*)(iomux_map_base + JTAG_TMS_PATA_DATA2_GPIO2_PORT2)) = MUX_MODE_GPIO;

	if(munmap((void *)iomux_map_base, MAP_SIZE) == -1){
		printf("Error unmapping iomux memory\n");
		return -1;
	}

	return 0;
}

int gpio_init()
{
	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1){
		printf("Error opening /dev/mem\n");
		return -1;
	}

	if(config_iomux_gpio_mode() == -1) {
		printf("config iomux gpio failed");
		close(fd);
		return -1;
	}

	/* Map one page */
	map_base = mmap((void *)0, GPIO_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, MMAP_BASE_ADDR & ~MAP_MASK);
	if(map_base == (void *) -1){
		printf("Error mapping memory\n");
		close(fd);
		return -1;
	}
}


int gpio_exit()
{
	/* Unmap previously mapped page */
	if(munmap((void *)map_base, MAP_SIZE) == -1){
		printf("Error unmapping memory\n");
		close(fd);
		return -1;
	}

	close(fd);
}

int setgpiodir(unsigned int port, unsigned int pin, unsigned int dir)
{
	if((port < 1)||(port > 7)||(pin < 0)||(pin > 31)) return -1;

	if (port < 5) {
		if(dir) {   /* dir=1 output */
			*((unsigned long*)(map_base+(port-1)*PORT_OFFSET_INC+GPIO_REG_GDIR)) |= (0x1<<pin);
		} else {    /* dir=0 input */
			*((unsigned long*)(map_base+(port-1)*PORT_OFFSET_INC+GPIO_REG_GDIR)) &= (~(0x1<<pin));
		}
	} else {
		if(dir) {
			*((unsigned long*)(map_base+port*PORT_OFFSET_INC+HIGH_PORT_SHIFT+GPIO_REG_GDIR)) |= (0x1<<pin);
		} else {
			*((unsigned long*)(map_base+port*PORT_OFFSET_INC+HIGH_PORT_SHIFT+GPIO_REG_GDIR)) &= (~(0x1<<pin));
		}
	}

	return 0;
}

int setgpiodata(unsigned int port, unsigned int pin, unsigned int data)
{
	if((port < 1)||(port > 7)||(pin < 0)||(pin > 31)) return -1;

	if (port < 5) {
		if(data){
			*((unsigned long*)(map_base+(port-1)*PORT_OFFSET_INC+GPIO_REG_DR)) |= (0x1<<pin);
		}else{
			*((unsigned long*)(map_base+(port-1)*PORT_OFFSET_INC+GPIO_REG_DR)) &= (~(0x1<<pin));
		}
	} else {
		if(data) {
			*((unsigned long*)(map_base+port*PORT_OFFSET_INC+HIGH_PORT_SHIFT+GPIO_REG_DR)) |= (0x1<<pin);
		} else {
			*((unsigned long*)(map_base+port*PORT_OFFSET_INC+HIGH_PORT_SHIFT+GPIO_REG_DR)) &= (~(0x1<<pin));
		}
	}

	return 0;
}

int getgpiodata(unsigned int port, int pin, unsigned int * data)
{
	unsigned int temp = 0;

	if((port < 1)||(port > 7)||(pin < 0)||(pin > 31)) return -1;

	if (port < 5) {
		temp = *((unsigned long*)(map_base+(port-1)*PORT_OFFSET_INC+GPIO_REG_PSR));
	} else {
		temp = *((unsigned long*)(map_base+port*PORT_OFFSET_INC+HIGH_PORT_SHIFT+GPIO_REG_PSR));
	}

	if(temp & (0x1<<pin))
		*data = 1;
	else
		*data = 0;

	return 0;
}


