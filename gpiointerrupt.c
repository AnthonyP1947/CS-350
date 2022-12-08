/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/I2C.h>

/* Driver configuration */
#include "ti_drivers_config.h"


/*
 *  ======== UART Driver Stuff ========
 */
#define DISPLAY(x) UART_write(uart, &output, x);

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
UART_Handle uart;

void initUART(void)
{
    UART_Params uartParams;
    // Init the driver
    UART_init();
    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}
// ------------------------------- UART END -----------------------------



/*
 *  ======== I2C Driver Stuff ========
 */
// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}

sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    // Init the driver
    I2C_init();
    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"));
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"));
    }

    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress));
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"));
    }
}

int16_t readTemp(void)
{
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor %d\n\r", i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }
    return temperature;
}
// ---------------------------------- I2C END ------------------------------------------



/*
 *  ======== Timer Driver Stuff ========
 */
// Driver Handles - Global variables
Timer_Handle timer0;
volatile unsigned char TimerFlag = 0;

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

void initTimer(void)
{
    Timer_Params params;
    // Init the driver
    Timer_init();
    // Configure the driver
    Timer_Params_init(&params);
    //Every 100 milliseconds
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}
// ---------------------------------- Timer End ------------------------------------------



/*
 *  ======== GPIO Interrupt Stuff ========
 */
/* : Add rest of gpiointerrupt.c file default structure, including
   gpioButtonFxn0(), gpioButtonFxn1(), mainThread(), etc., including
   final scaffolding from the PDF for the mainThread(), duplicated below:*/

/* Init buttons before callback */
    int leftButton = 0;                     // Left button for decrement
    int rightButton = 0;                    // Right button for increment

    //button interupts
    void gpioButtonFxn0(uint_least8_t index)
    {
        /* Button function */
        leftButton = 1;
    }
    void gpioButtonFxn1(uint_least8_t index)
    {
        /* Button function */
        rightButton = 1;
    }
    void *mainThread(void *arg0)
    {
        /* Call driver init functions */
        GPIO_init();

        /* Configure the red LED */
        GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

        /* Configure BUTTON0 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Turn on user LED */
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

        /* Enable interrupts */
        GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
        if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
            /* Configure BUTTON1 pin */
            GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

            /* Install Button callback */
            GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

            /* Enable interrupts */
            GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
        }

    /* Call driver init functions */

    initUART(); // The UART must be initialized before calling initI2C()
    initI2C();
    initTimer();

    /* Init variables*/
    unsigned long buttonCheckTime = 200;                              // Init elapsed time for button
    unsigned long tempCheckTime = 500;                                // Init elapsed time for temp
    unsigned long displayCheckTime = 1000;                             // Init elapsed time for display
    const unsigned long buttonCheckPeriod = 200;                    // Init constant button period (200ms)
    const unsigned long tempCheckPeriod = 500;                      // Init constant temp period (500ms)
    const unsigned long displayCheckPeriod = 1000;                  // Init constant display period (1000ms)
    const unsigned long timerPeriod = 100;                          // Init constant timer period/GCD (100ms)
    int setpoint_F = 25;                                              // Init setpoint (default thermostat value)
    int heat = 0;                                                   // Init heat (0 is off, 1 is on)
    int seconds = 0;                                                // Init seconds
    int temperature = 0;
    // Loop Forever
    // The student should add flags (similar to the timer flag) to the button handlers
    /*  output to the server (via UART) should be formatted as
     *  AA    = ASCII decimal value of room temperature (00 - 99) degrees Celsius
     *  BB    = ASCII decimal value of set-point temperature (00-99) degrees Celsius
     *  S     = 0 if heat is off, 1 if heat is on
     *  CCCC  = decimal count of seconds since board has been reset
     *  <%02d,%02d,%d,%04d> = temperature, set-point, heat, seconds
     */
    while(1)
    {
        readTemp();
      // Every 200ms check the button flags
        if (buttonCheckTime >= buttonCheckPeriod)
        {
            if (rightButton == 1)
            {
                setpoint_F += 1;
                rightButton = 0;
            }
            if (leftButton == 1)
            {
                setpoint_F -= 1;
                leftButton = 0;
            }
        }
      // Every 500ms read the temperature and update the LED
        if (tempCheckTime >= tempCheckPeriod)
        {
            temperature = readTemp();
            if (temperature < setpoint_F)
            {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                heat = 1;
            }
            else
            {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                heat = 0;
            }
        }

      // Every second output the following to the UART
      if (displayCheckTime >= displayCheckPeriod)
      {
          // <%02d,%02d,%d,%04d> = temperature, setpoint, heat, seconds
          DISPLAY( snprintf(output, 64, "<%02d, %02d, %d, %04d>\n\r", temperature, setpoint_F, heat, seconds ));
          ++seconds;
      }

      // Refer to zyBooks - "Converting different-period tasks to C"
      // Remember to configure the timer period
      while (!TimerFlag){} // Wait for the timer period
      TimerFlag = 0;       // Lower flag raised by timer
      buttonCheckTime += timerPeriod;                               // Increment buttonCheckTime by 100ms
      tempCheckTime += timerPeriod;                               // Increment tempCheckTime by 100ms
      displayCheckTime += timerPeriod;                            // Increment displayCheckTime by 100ms
    }
}

