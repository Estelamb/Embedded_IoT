#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

// LED from the board devicetree (standard alias: led0 for led0)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

int main(void)
{
    printk("Embedded Platforms and Communications for IoT\n");
    printk("        ETSIST - UPM - MUIoT 2025-2026       \n\n");
	
    printk("    Board LED toggle (single thread: main)   \n");

    // Ensure the GPIO device is ready
    if (!device_is_ready(led0.port)) {
        printk("Error: LED device not ready\n");
        return 0;
    }

    if (!device_is_ready(led2.port)) {
        printk("Error: LED device not ready\n");
        return 0;
    }

    // Configure as output and start LOW (off)
    if (gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE) < 0) {
        printk("Error: configuring led0\n");
        return 0;
    }

    if (gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("Error: configuring LED2\n");
        return 0;
    }

    // Toggle loop: change state every 1 second
    while (1) {
        printk("Toggle!!\n");
        gpio_pin_toggle_dt(&led0);
        gpio_pin_toggle_dt(&led2);
        k_sleep(K_SECONDS(1));
    }
}
