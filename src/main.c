
#include <zephyr/kernel.h>          /* for kernel functions*/
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>   /* for timing services */
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <zephyr/timing/timing.h>
#define ERROR -1

#define SOF_SYM '#'
#define EOF_SYM 0x0d

struct RtData{
    char ledState; 
    char buttonState;
    uint16_t temperature;
};

struct RtData RtData = {
    .ledState = {0,0,0,0},
    .buttonState = {0,0,0,0},
    .temperature = 0
};


#define NUMBERBUTTONS 4
#define NUMBERLED 4
int ledPins[NUMBERLED] = {13,14,15,16};
int buttonsPins[NUMBERBUTTONS] = {11,12,24,25};


#define GPIO0_NODE DT_NODELABEL(gpio0)


static const struct device * gpioDev = DEVICE_DT_GET(GPIO0_NODE);

#define STACK_SIZE  1000
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
k_tid_t buttonID;
void buttonThread(void *argA, void *argB, void *argC);
volatile uint8_t buttonPressedName;
static struct gpio_callback buttonCallbackData;

#define I2CPRIORITY 1
volatile uint16_t ic2Period = 1000;
K_THREAD_STACK_DEFINE(i2cStack, STACK_SIZE);
struct k_thread i2cData;
k_tid_t i2cID;
void i2cThread(void *argA, void *argB, void *argC);


#define I2C0_NODE DT_NODELABEL(temperatureSensor)
static const struct i2c_dt_spec devI2C = I2C_DT_SPEC_GET(I2C0_NODE);


#define UARTPRIORITY 1
#define UART_NODE DT_NODELABEL(uart0)

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
};

void uartThread(void *argA, void *argB, void *argC);

const struct device *uartDev;
uint8_t rxBuffer[RXBUFFERSIZE];
uint8_t rxData[RXBUFFERSIZE];
volatile uint16_t uartReceiverUsed;

void uartCallback(const struct device *dev, struct uart_event *event, void *data);

struct k_sem semUart;

#define rxTimeout
volatile char command[20];
unsigned char sizeOfCommand = 0;
void main(void){
    int err = 0;
    int ret = 0;

    uartDev = device_get_binding(DT_LABEL(UART_NODE));

    if(uartDev == NULL){
        return;
    }

    err = uart_configure(uartDev, &uartConfig);

    err =  uart_rx_enable(uartDev ,rxBuffer ,sizeof(rxBuffer),rxTimeout);

    if (err) {
        return;
    }

    if (!device_is_ready(gpioDev)) { 
        return; 
    }

    for(int i=0; i<sizeof(buttonsPins); i++) {
		ret = gpio_pin_configure(gpioDev, buttonsPins[i], GPIO_INPUT | GPIO_PULL_UP);
		if (ret < 0)  {
            return; 
        }
    }

    for(int i=0; i<sizeof(buttonsPins); i++) {
		ret = gpio_pin_interrupt_configure(gpioDev, buttonsPins[i], GPIO_INT_EDGE_TO_ACTIVE );
		if (ret < 0) { 
            return; 
        }
	}

    uint32_t pinmask = 0;
	for(int i=0; i<sizeof(buttonsPins); i++) {
		pinmask |= BIT(buttonsPins[i]);
	}

	gpio_init_callback(&buttonCallbackData, buttonPressed, pinmask);	
	
	
	gpio_add_callback(gpioDev, &buttonCallbackData);

  
    k_sem_init(&semUart, 0, 1);

   
	ledID = k_thread_create(&ledData, ledStack,K_THREAD_STACK_SIZEOF(ledStack), ledThread,NULL, NULL, NULL, LEDPRIORITY, 0, K_NO_WAIT);

   
	buttonID = k_thread_create(&buttonData, buttonStack,K_THREAD_STACK_SIZEOF(buttonStack), buttonThread,NULL, NULL, NULL,BUTTONPRIORITY, 0, K_NO_WAIT);

  
    i2cID = k_thread_create(&i2cData, i2cStack, K_THREAD_STACK_SIZEOF(i2cStack), i2cThread,NULL, NULL, NULL, I2CPRIORITY, 0, K_NO_WAIT);

   
    uartID = k_thread_create(&uartData, uartStack,K_THREAD_STACK_SIZEOF(uartStack), uartThread,NULL, NULL, NULL, UARTPRIORITY, 0, K_NO_WAIT);

    return;
}

void ledThread(void *argA, void *argB, void *argC){
    uint64_t finTime = 0;
    uint64_t releaseTime = 0;
    int ret = 0;

    if(!device_is_ready(gpioDev)){
        return;
    }

    for(int i = 0; i <= NUMBERLED; i++){
        ret = gpio_pin_configure(gpioDev, ledPins[i], GPIO_OUTPUT_ACTIVE);
        if(ret !=0){
            return;
        }
    }

    releaseTime = k_uptime_get() + ledPeriod;

    while(1){
        for(int i = 0; i < NUMBERLED; i++){
            gpio_pin_set(gpioDev, ledPins[i], !RtData.ledState[i]);
        }

        finTime = k_uptime_get();
        if(finTime < releaseTime){
            k_msleep(releaseTime - finTime);
            releaseTime = releaseTime + ledPeriod;
        }
    }

    timing_stop();
}

void i2cThread(void *argA, void *argB, void *argC){
    uint64_t finTime = 0; 
    uint64_t releaseTime = 0;
    int ret = 0;

    if(!device_is_ready(devI2C.bus)){
        return; 
    }

    uint8_t inputSensor;

    uint8_t configurations = 0x00;
    ret = i2c_write_dt(&devI2C, &configurations, sizeof(configurations));
    
    if(ret !=0){
        return;
    }

    releaseTime = k_uptime_get() + ic2Period;

    while(1){
        ret = i2c_read_dt(&devI2C, &inputSensor, sizeof(inputSensor));
        if(ret < 0){
            return;
        }

        RtData.temperature = inputSensor;

        finTime = k_uptime_get();

        if(finTime < releaseTime){
            k_msleep(releaseTime - finTime);
            releaseTime = releaseTime + ic2Period;
        }
    }

    timing_stop();
}

void uartThread(void *argA, void *argB, void *argC){
    int ret = 0;
    char data;
    int k;
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

                commandProcessor(); 
            }
        }
        k_msleep(1);
     }
}



void buttonThread(void * argA, void *argB, void *argC){
    int64_t finTime = 0;
    int64_t releaseTime = 0;

    releaseTime = k_uptime_get() + buttonThread;
    int i;
    while(1){

        for( i = 0; i < 4; i++){
            if(buttonPressedName == i){
                RtData.buttonState[i] = !RtData.buttonState[i];
            }
        }

        buttonPressedName = 0;

        finTime = k_uptime_get();

        if(finTime < releaseTime){
            k_msleep(releaseTime - finTime);
            releaseTime = releaseTime + ic2Period;
        }
    } 

    timing_stop();
}


void uartCallback(const struct device *dev, struct uart_event *event, void *data){
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
             ret=  uart_rx_enable(uartDev ,rxBuffer,sizeof(rxBuffer),rxTimeout);
    default:
        break;
    }
}


static buttonPressed(struct device * dev, struct gpio_callback *cb, uint32_t pins){
    int button;
    for(int i = 0; i < NUMBERBUTTONS; i++){
        if(BIT(buttonsPins[i]) & pins){
            button = buttonsPins[i];
        }
    }

    buttonPressedName = button;
}

// cOMEÇA COM # ACABA EM \R
int commandProcessor(void){
    uint16_t i = 0;
    char frequency[3] = {0,0,0};
    uint16_t freq = 0;
    uint8_t error = 0;

    uint8_t receivedMessage[TXBUFFERSIZE];

    if(sizeOfCommand == 0){
        return ERROR;
    }

    for(i = 0; i < sizeOfCommand; i++){
        if(command[i] == SOF_SYM){
            i++;
            break;
        }
    }

    if(i < sizeOfCommand){
        if(command[i]== 'T'){
            sprintf(receivedMessage, "T: %d\n\r", RtData.temperature);
            error = uart_tx(uartDev, receivedMessage, strlen(receivedMessage), SYS_FOREVER_MS);
            if(error){
                return ERROR;
            }
        }
        if(command[i] == 'L'){
            if(command[i+1] < '1' || command[i+1] > '4' || command[i+2] != '1' || command[i+2] > '0'){
                return ;
            }

            RtData.ledState[ command[i+1]- '0' + 1] = command[i+2] -'0' ;
            sprintf(receivedMessage,"LED: %d now is %d \n\r",command[i+1], command[i+2]);
            error = uart_tx(uartDev, receivedMessage, strlen(receivedMessage), SYS_FOREVER_MS);
            if (error) { 
                return ; 
            }
        }
        if(command[i] == 'B'){
                if(command[i+1] < '1' || command[i+1] > '4'){
                    return ERROR;
                }
                 sprintf(receivedMessage,"Button[%d] = %d \n\r",command[i+2] - '0', RtData.buttonState[command[i+2] - '0'-1]);  
                error = uart_tx(uartDev, receivedMessage, strlen(receivedMessage), SYS_FOREVER_MS);
                if (error) {
                    return; 
                }
        }
      /*  if(command[i] == 'F'){
            if(command[i+1] == B){

            }
        }
*/    }

    return 0;
}

