/* By Yanjun Luo, for MX27 gpio test with application software. */

#include "gpio.h"

/* Memory mapping definitions */
#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

#define MMAP_BASE_ADDR	0x10015000


#define	PORT_OFFSET_INC		0x100

#define PORTA_OFFSET_ADDR	0x000
#define PORTB_OFFSET_ADDR	0x100
#define PORTC_OFFSET_ADDR	0x200
#define PORTD_OFFSET_ADDR	0x300
#define PORTE_OFFSET_ADDR	0x400
#define PORTF_OFFSET_ADDR	0x500

#define	GPIO_REG_DDIR		0x00
#define	GPIO_REG_OCR1		0x04
#define	GPIO_REG_OCR2		0x08
#define	GPIO_REG_ICONFA1	0x0C
#define	GPIO_REG_ICONFA2	0x10
#define	GPIO_REG_ICONFB1	0x14
#define	GPIO_REG_ICONFB2	0x18
#define	GPIO_REG_DR		0x1C
#define	GPIO_REG_GIUS		0x20
#define	GPIO_REG_SSR		0x24
#define	GPIO_REG_ICR1		0x28
#define	GPIO_REG_ICR2		0x2C
#define	GPIO_REG_IMR		0x30
#define	GPIO_REG_ISR		0x34
#define	GPIO_REG_GPR		0x38
#define	GPIO_REG_SWR		0x3C
#define	GPIO_REG_PUEN		0x40


volatile void *map_base = NULL; 
int fd = 0;

int gpio_init()
{
	int i, rc, pin, dest_addr, id, retval, flag = 0;
	char *current, action, port;
	unsigned long readval, writeval;

	unsigned int temp = 0, temp1 = 0;;
	
	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1){
		printf("Error opening /dev/mem\n");
		return -1;
	}

  /* Map one page */
	map_base = mmap((void *)0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, MMAP_BASE_ADDR & ~MAP_MASK);
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
	if((port < 0)||(port > 5)||(pin < 0)||(pin > 31)) return -1;
		
	*((unsigned long*)(map_base+port*PORT_OFFSET_INC+GPIO_REG_GIUS)) |= (0x1<<pin);

	if(dir){
		*((unsigned long*)(map_base+port*PORT_OFFSET_INC+GPIO_REG_DDIR)) |= (0x1<<pin);
		if(pin > 15){
			*((unsigned long*)(map_base+port*PORT_OFFSET_INC+GPIO_REG_OCR2)) |= (0x3<<((pin-16)*2));
		}else{
			*((unsigned long*)(map_base+port*PORT_OFFSET_INC+GPIO_REG_OCR1)) |= (0x3<<(pin*2));
		}
	}else{
		*((unsigned long*)(map_base+port*PORT_OFFSET_INC+GPIO_REG_DDIR)) &= (~(0x1<<pin));
		if(pin > 15){
			*((unsigned long*)(map_base+port*PORT_OFFSET_INC+GPIO_REG_ICONFA2)) &= (~(0x3<<((pin-16)*2)));
			*((unsigned long*)(map_base+port*PORT_OFFSET_INC+GPIO_REG_ICONFB2)) &= (~(0x3<<((pin-16)*2)));
		}else{
			*((unsigned long*)(map_base+port*PORT_OFFSET_INC+GPIO_REG_ICONFA1)) &= (~(0x3<<(pin*2)));
			*((unsigned long*)(map_base+port*PORT_OFFSET_INC+GPIO_REG_ICONFB1)) &= (~(0x3<<(pin*2)));
		}
	}

	return 0;
}

int setgpiodata(unsigned int port, unsigned int pin, unsigned int data)
{
	if((port < 0)||(port > 5)||(pin < 0)||(pin > 31)) return -1;

	if(data){
		*((unsigned long*)(map_base+port*PORT_OFFSET_INC+GPIO_REG_DR)) |= (0x1<<pin);
	}else{
		*((unsigned long*)(map_base+port*PORT_OFFSET_INC+GPIO_REG_DR)) &= (~(0x1<<pin));
	}

	return 0;
}

int getgpiodata(unsigned int port, int pin, unsigned int * data)
{
	unsigned int temp = 0;
	
	if((port < 0)||(port > 5)||(pin < 0)||(pin > 31)) return -1;

	temp = *((unsigned long*)(map_base+port*PORT_OFFSET_INC+GPIO_REG_SSR));

	if(temp & (0x1<<pin)) *data = 1;
	else *data = 0;
			
	return 0;
}

int setpullup(unsigned int port, int pin, unsigned int pull)
{
	if((port < 0)||(port > 5)||(pin < 0)||(pin > 31)) return -1;
		
	if(pull)
		*((unsigned long*)(map_base+port*PORT_OFFSET_INC+GPIO_REG_PUEN)) |= (0x1<<pin);
	else
		*((unsigned long*)(map_base+port*PORT_OFFSET_INC+GPIO_REG_PUEN)) &= (~(0x1<<pin));

	return 0;
}




