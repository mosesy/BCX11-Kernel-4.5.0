#ifndef _LINUX_ACPI_GPIO_H_
#define _LINUX_ACPI_GPIO_H_

#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>

/**
 * struct acpi_gpio_info - ACPI GPIO specific information
 * @gpioint: if %true this GPIO is of type GpioInt otherwise type is GpioIo
 * @active_low: in case of @gpioint, the pin is active low
 */
struct acpi_gpio_info {
	bool gpioint;
	bool active_low;
};


#ifdef CONFIG_GPIO_ACPI

int acpi_get_gpio(char *path, int pin);
struct gpio_desc *acpi_get_gpiod_by_index(struct acpi_device *adev,
					  const char *propname, int index,
					  struct acpi_gpio_info *info);
#else /* CONFIG_GPIO_ACPI */

static inline struct gpio_desc *
acpi_get_gpiod_by_index(struct device *dev, int index,
			struct acpi_gpio_info *info)
{
	return ERR_PTR(-ENOSYS);
}

#endif /* CONFIG_GPIO_ACPI */

static inline int acpi_get_gpio_by_index(struct device *dev, int index,
					 struct acpi_gpio_info *info)
{
	struct acpi_device *adev = ACPI_COMPANION(dev);
	struct gpio_desc *desc = acpi_get_gpiod_by_index(adev, NULL, index, info);

	if (IS_ERR(desc))
		return PTR_ERR(desc);
	return desc_to_gpio(desc);
}


#endif /* _LINUX_ACPI_GPIO_H_ */
