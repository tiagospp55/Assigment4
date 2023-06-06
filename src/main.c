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

struct RtData{
    char ledState = {0,0,0,0};
    char buttonState = {0,0,0,0};
    uint16_t temperature = 0;
}

uint8_t ledPins[] = {13,14,15,16};
uint8_t buttonsPins[] = {11,12,24,25};
#define GPIO0_NODE DT_NODELABEL(gpio0)
#define LEDNUMBER

static const struct device * gpioDev = DEVICE_DT_GET(GPIO0_NODE);

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
volatile uint8_t buttonPressed;

#define I2CPRIORITY 1
volatile uint16_t ic2Period = 1000;
K_THREAD_STACK_DEFINE(i2cStack, STACK_SIZE);
struct k_thread i2cData;
k_tid_t i2cID;
void i2cThread(void *argA, void *argB, void *argC);


#define I2C0_NODE DT_NODELABEL(temperatureSensor);
static const struct i2c_dt_spec devI2C == I2C_DT_SPEC_GET(I2C0_NODE);


#define uartPriority 1
#define UARTNODE DT_NODELABEL(uart0)

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

const struct device *uartDev;
uint8_t rxBuffer[RXBUFFERSIZE];
uint8_t rxData[RXBUFFERSIZE];
volatile uint16_t uartReceiverUsed;

void uartCallback(const struct devide *dev, struct uartEvent *event, void *data);

struct k_sem semUart;
void main(void){

}

void ledThread(void *argA, void *argB, void *argC){
    uint_64_t finTime = 0;
    uint65_t releaseTime = 0;
    int ret = 0;

    if(!device_is_ready(gpio_dev)){
        return;
    }

    for(int i = 0; i < LEDNUMBER; i++){
        ret = gpio_pin_configure(gpio0_dev, ledPins[i], FPIO_OUTPUT_ACTIVE);
        if(ret !=0){
            return;
        }
    }

    releaseTime = k_uptime_get() + ledPeriod;

    while(1){
        for(int i = 0; i < LEDNUMBER; i++){
            gpio_pin_set(gpioDev, ledPins[i], !RtData.ledState[i]);
        }

        finTime = k_uptime_get();
        if(finTIme < releaseTime){
            k_msleep(releaseTIme - finTIme);
            releaseTime = releaseTime + ledPeriod;
        }
    }

    timing_stop();
}

void i2cThread(void *argA, void *argB, void *argC){
    uint64_t finTime = 0; 
    uint64_t releaseTime = 0;
    int ret = 0;

    if(!device_is_ready(dev_i2c.bus)){
        return; 
    }

    uint8_t inputSensor;

    uint8_t configurations = 0x00;
    ret = i2c_write_dt(&dev_i2c, &configurations, sizeof(configurations));
    
    if(ret !=0){
        return;
    }

    releaseTIme = k_uptime_get() + ic2Period;

    while(1){
        ret = i2c_read_dt(&dev_i2c, &inputSensor, sizeof(inputSensor));
        if(ret < 0){
            return;
        }

        RtData.temperature = inputSensor;

        finTime = k_uptime_get();

        if(finTime < releaseTime){
            k_msleep(releaseTIme - finTime);
            releaseTime = releaseTime + ic2Period;
        }
    }

    timing_stop();
}

void uartThread(void *argA, void *argB, void *argC){
    int ret = 0;
    char data;

    uint8_t transmitMessage[TXBUFFERSIZE];

     while(1){
        k_sem_take(&semUart, K_FOREVER);

        if(uartReceiverUsed > 0){
            rxData[uartReceiverUsed] = 0;
            ret = uart_tx(uartDev, transmitMessage, strlen(transmitMessage), SYS_FOREVER_MS));

            if(ret){
                return;
            }

            data = rxData[uartReceiverUsed - 1];

            if(data == 0xd){
                for(int i = 0; i <= uartReceiverUsed; i++){
                    command[k] = rxData[k];
                    rxData[k] = 0;
                }
                sizeOfCommand = uartReceiverUsed;

                cmdProcessor(); 
            }
        }
        k_msleep(1);
     }
}



void buttonThread(void * argA, void *argB, void *argC){
    uint64_t finTime = 0;
    uint64_t releaseTime = 0;

    releaseTime = k_uptime_get() + buttonThread;

    while(1){

        for(int i = 0; i < 4; i++){
            if(buttonPressed == i){
                RtData.buttonState[i] = !RtData.buttonState[i] 
            }
        }

        buttonPressed = 0;

        finTime = k_uptime_get();

        if(finTime < releaseTime){
            k_msleep(releaseTime - finTime);
            releaseTime = releaseTime + ic2Period;
        }
    } 

    timing_stop();
}


void uartCallback(const struct devide *dev, struct uartEvent *event, void *data){
    int ret = 0; 

    switch(event->type)
    {
    case UART_TX_DONE:
        break;
    case UART_RX_RDY:
        memcpy(&rxData[uartReceiverUsed], &rxBuffer[event->data.rx.offset], event->data.rx.len);
        uartReceiverUsed++;
        k_sem_give(&semUart);
    case UART_TX_ABORTED:
        break;
    case UART_RX_DISABLED:
             err =  uart_rx_enable(uartDev ,rxBuffer,sizeof(rxBuffer),RX_TIMEOUT);
    default:
        break;
    }
}
