/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
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
 *  ======== main_tirtos.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/PWM.h>

/* Example/Board Header files */
#include "Board.h"

#define TASKSTACKSIZE      1024

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];
Semaphore_Struct semStruct;
Semaphore_Handle saSem;


/*  ======== taskFxn ======== */
Void taskFxn(UArg arg0, UArg arg1)
{
    int i;
    uint8_t         txBuffer[16];
    uint8_t         rxBuffer[16];
    uint8_t         deviceAddress;
    uint8_t         registerAddress;
    uint8_t         registerValue;
    uint8_t         rxValue;

    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;

    /* 100 msecond delay */
    uint32_t del_time = 100000;

    /* init I2C driver */
    I2C_init();

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

    ///////////////////////////////////////////////////////////////
    /*start pwm on pin 12*/
    /*
    PWM_Handle pwm1 = NULL;
    PWM_Params params;

    PWM_init();

    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_FRACTION;
    params.dutyValue = 0;
    params.periodUnits = PWM_PERIOD_HZ;
    params.periodValue = 6e6;
    pwm1 = PWM_open(Board_PWM2, &params);
    if (pwm1 == NULL) {
        while (1);
    }

    PWM_start(pwm1);
    PWM_setDuty( pwm1, PWM_DUTY_FRACTION_MAX/2);
    */
    ///////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////
    /*codec reset pin high*/
    GPIO_write(Board_GPIO_PIN15, 1);


    ///////////////////////////////////////////////////////////////
    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;

    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        /*failed init for i2c bus*/
        while (1);
    }

    deviceAddress = 0x95>>1;
    registerAddress = 0x01>>1;
    registerValue = 0xFF;

    /* Point to the T ambient register and read its 2 bytes */
    txBuffer[0] = registerAddress;
    txBuffer[1] = registerValue;
    i2cTransaction.slaveAddress = deviceAddress;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;


    for( i = 0; i < 200; i++)
    {
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            rxValue = rxBuffer[0];
        }
        else
        {
            /* I2C Bus fault */
        }

        /* Sleep for del_time seconds */
        usleep(del_time);
        GPIO_toggle(Board_GPIO_LED0);
    }

    /* Deinitialise I2C */
    I2C_close(i2c);
}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;
    Semaphore_Params semParams;

    /* Call driver init functions */
    Board_initGeneral();
    GPIO_init();


    /*codec reset pin low*/
    GPIO_write(Board_GPIO_PIN15, 0);

    /* Construct BIOS objects */
    Task_Params_init(&taskParams);
    taskParams.priority = 1;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)taskFxn, &taskParams, NULL);

    /* Construct a Semaphore object to be use as a resource lock, inital count 1 */
    Semaphore_Params_init(&semParams);
    Semaphore_construct(&semStruct, 1, &semParams);
    /* Obtain instance handle */
    saSem = Semaphore_handle(&semStruct);

    BIOS_start();

    return (0);
}
