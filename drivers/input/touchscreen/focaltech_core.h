#ifndef __LINUX_ftxxxx_TS_H__
#define __LINUX_ftxxxx_TS_H__

#include <linux/version.h>
#define __devinit        __section(.devinit.text) __cold notrace
#define __devinitdata    __section(.devinit.data)
#define __devinitconst   __section(.devinit.rodata)
#define __devexit        __section(.devexit.text) __exitused __cold notrace
#define __devexitdata    __section(.devexit.data)
#define __devexitconst   __section(.devexit.rodata)

/* -- dirver configure -- */
#define AUTO_CLB_NEED                              1
#define FTS_UPGRADE_LOOP	5


#define CFG_MAX_TOUCH_POINTS	10

#define PRESS_MAX	0xFF
#define FT_PRESS	0x08
#define FTS_I2C_ADDR  (0x70>>1)
#define FTS_NAME	"ft5x26"

#define FTS_I2C_ADDR  (0x70>>1)
#define TOUCH_MAX_X	1280
#define TOUCH_MAX_Y	800
#define FTS_INT_PIN	341
#define FTS_INT_PIN_NAME	"ftxxxx-int"
#define FTS_RESET_PIN	408
#define FTS_RESET_PIN_NAME	"ftxxxx-reset"

#define FT_MAX_ID	0x0F
#define FT_TOUCH_STEP	4
#define FT_TOUCH_X_H_POS		3
#define FT_TOUCH_X_L_POS		4
#define FT_TOUCH_Y_H_POS		5
#define FT_TOUCH_Y_L_POS		6
#define FT_TOUCH_EVENT_POS		3
#define FT_TOUCH_ID_POS			5

#define POINT_READ_BUF	(3 + FT_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

#define FT5506                  0x08
#define FT5826                  0x59
#define FTS_CHIP_ID_FT5826      0x59

#define FT_TOUCH_VENDOR_AVD     0x38
#define FT_TOUCH_VENDOR_163     0x5C
/*register address*/
#define FTS_REG_CHIP_ID      0xA3    //chip ID 
#define FTS_REG_FW_VER		0xA6
#define FTS_REG_POINT_RATE	0x88
#define FTS_REG_THGROUP	    0x80
#define FTS_REG_VENDOR_ID	0xA8

#define FTS_ENABLE_IRQ	1
#define FTS_DISABLE_IRQ	0
#define FTS_RESET_GPIO       128

int fts_i2c_Read(struct i2c_client *client, char *writebuf, int writelen,
		    char *readbuf, int readlen);
int fts_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);

void fts_reset_tp(int HighOrLow);

void ftxxxx_Enable_IRQ(struct i2c_client *client, int enable);
/* The platform data for the Focaltech ftxxxx touchscreen driver */
struct fts_platform_data {
	uint32_t gpio_irq;		// IRQ port
	uint32_t irq_cfg;
	
	uint32_t gpio_wakeup;		// Wakeup support
	uint32_t wakeup_cfg;

	uint32_t gpio_reset;		// Reset support
	uint32_t reset_cfg;

	int screen_max_x;
	int screen_max_y;
	int pressure_max;
};

#endif
