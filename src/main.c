#include <zephyr/kernel.h>          
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>   
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>


#define ERROR -1

#define STACK_SIZE

#define LEDPRIORITY 1
volatile uint32_t ledPeriod = 1000;
K_THREAD_STACK_DEFINE(ledStack, STACK_SIZE);
struct k_thread ledData;
k_tid_t ledID;
void ledThread(void *argA, void *argB, void *argC);

#define BUTTONPRIORITY
volatile buttonPeriod = 1000;
K_THREAD_STACK_DEFINE(buttonStack, STACK_SIZE);
struct k_thread buttonData;
k_tid_t ledID;
void buttonThread(void *argA, void *argB, void *argC);


#define I2CPRIORITY 1
volatile uint16_t ic2Period = 1000;
K_THREAD_STACK_DEFINE(i2cStack, STACK_SIZE);
struct k_thread i2cData;
k_tid_t i2cID;
void i2cThread(void *argA, void *argB, void *argC);

#define uartPriority 1
K_THREAD_STACK_DEFINE(uartStack, STACK_SIZE);
struct k_thread uartData;
k_tid_t uartID;

#define RXBUFFERSIZE 100
#define TXBUFFERSIZE 100
#define RXMAXTIMOUT 1000

const struct uart_config uartConfig = {
    .baudrate = 115200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
}

void uartThread(void *argA, void *argB, void *argC);


