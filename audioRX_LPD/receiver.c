/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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

/***** Includes *****/
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"

#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"
#include "G722/G722_64_Decoder.h"

#include <stdlib.h>

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};


/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define ADPCM_FRAME_SIZE       60//384 /* 20 Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     8  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */


/***** Defines *****/
#define RADIO_RX_TASK_STACK_SIZE 2048
#define RADIO_RX_TASK_PRIORITY   1

#define UART_RX_TASK_STACK_SIZE 2048
#define UART_RX_TASK_PRIORITY   2


/***** Prototypes *****/
static void radioRxTaskFunction(UArg arg0, UArg arg1);
static void uartRxTaskFunction(UArg arg0, UArg arg1);

static void radioRxCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

static void uartConfigure();
void uart_writePayLoad(uint8_t *packet, uint16_t length);

/***** Variable declarations *****/
static Task_Params radioRxTaskParams;
Task_Struct radioRxTask;    /* not static so you can see in ROV */
static uint8_t radioRxTaskStack[RADIO_RX_TASK_STACK_SIZE];

static Task_Params uartRxTaskParams;
Task_Struct uartRxTask;    /* not static so you can see in ROV */
static uint8_t uartRxTaskStack[UART_RX_TASK_STACK_SIZE];

Semaphore_Struct semRadioRxStruct;
Semaphore_Handle semRadioRxHandle;

Semaphore_Struct semUartRxStruct;
Semaphore_Handle semUartRxHandle;

static RF_Object rfObject;
static RF_Handle rfHandle;
static RF_CmdHandle rfRxCmd;

UART_Handle uart = NULL;
UART_Params uartParams;


/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
#pragma DATA_ALIGN (rxDataEntryBuffer, 4);
static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES, ADPCM_FRAME_SIZE, NUM_APPENDED_BYTES)];

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
uint8_t packetReady = 0;
static uint16_t packetLength;
static uint8_t* packetDataPointer;
uint32_t curtime;


static PIN_Handle pinHandle;

/* Audio stuff */
static uint8_t* encodedData;
static uint16_t* unpackedData;
static int16_t* pcmData;

static bool CreateAudioBuffers();
static void DestroyAudioBuffers();

static G722DECODER audio_decoder;

/***** Function definitions *****/
void UartRxTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&uartRxTaskParams);
    uartRxTaskParams.stackSize = UART_RX_TASK_STACK_SIZE;
    uartRxTaskParams.priority = UART_RX_TASK_PRIORITY;
    uartRxTaskParams.stack = &uartRxTaskStack;
    uartRxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&uartRxTask, uartRxTaskFunction, &uartRxTaskParams, NULL);
}


void RadioRxTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&radioRxTaskParams);
    radioRxTaskParams.stackSize = RADIO_RX_TASK_STACK_SIZE;
    radioRxTaskParams.priority = RADIO_RX_TASK_PRIORITY;
    radioRxTaskParams.stack = &radioRxTaskStack;
    radioRxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&radioRxTask, radioRxTaskFunction, &radioRxTaskParams, NULL);
}

static void radioRxTaskFunction(UArg arg0, UArg arg1)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    if( RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            ADPCM_FRAME_SIZE + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(1);
    }

    /* Modify CMD_PROP_RX command for application needs */
    RF_cmdPropRxAdv.pQueue = &dataQueue;           /* Set the Data Entity queue for received data */
    RF_cmdPropRxAdv.rxConf.bAutoFlushIgnored = 1;  /* Discard ignored packets from Rx queue */
    RF_cmdPropRxAdv.rxConf.bAutoFlushCrcErr = 1;   /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRxAdv.maxPktLen = ADPCM_FRAME_SIZE;  /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRxAdv.pktConf.bRepeatOk = 1;
    RF_cmdPropRxAdv.pktConf.bRepeatNok = 1;

    if (!rfHandle) {
        /* Request access to the radio */
        rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

        /* Set the frequency */
        RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

        adpcm64_decode_init(&audio_decoder);
    }

    while (1) {
    /* Enter RX mode and stay forever in RX */
        rfRxCmd = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRxAdv, RF_PriorityNormal, &radioRxCallback, IRQ_RX_ENTRY_DONE);

        Semaphore_pend(semRadioRxHandle, BIOS_WAIT_FOREVER);
    }

}

static void radioRxCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    int readData0,readData1;
    int i;

    if (e & RF_EventRxEntryDone)
    {
        /* Toggle pin to indicate RX */
        PIN_setOutputValue(pinHandle, Board_LED2,!PIN_getOutputValue(Board_LED2));

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        packetLength      = ADPCM_FRAME_SIZE;
        packetDataPointer = (uint8_t*)(&currentDataEntry->data);

        //memcpy( encodedData, packetDataPointer, packetLength);

        readData0 = adpcm64_unpack_vadim((uint16_t*)packetDataPointer, unpackedData, packetLength/2);
        readData1 = adpcm64_decode_run(&audio_decoder, unpackedData, pcmData, readData0);

        packetReady = 1;

        RFQueue_nextEntry();

        Semaphore_post(semUartRxHandle);
    }
}

static void uartRxTaskFunction(UArg arg0, UArg arg1)
{
    /*
    int readData0 = 0;
    int readData1 = 0;
    int i = 0;
    */

    int encodedSize = ADPCM_FRAME_SIZE;

    if (uart == NULL) {
        /* Create a UART with data processing off. */
        uartConfigure();
        adpcm64_decode_init(&audio_decoder);

        if (uart == NULL) {
            System_abort("Error opening the UART");
        }
    }

    while (1) {

        /*
        for( i = 0; i < encodedSize; i++ )
        {
            encodedData[i] = (uint8_t)rand();
        }

        readData0 = adpcm64_unpack_vadim((uint16_t*)encodedData, unpackedData, encodedSize/2);
        readData1 = adpcm64_decode_run(&audio_decoder, unpackedData, pcmData, readData0);
        UART_write(uart, pcmData, sizeof(int16_t)*readData1);
*/

        Semaphore_pend(semUartRxHandle, BIOS_WAIT_FOREVER);
        if (packetReady) {
            //uart_writePayLoad((uint8_t*)unpackedData, 96);
            uart_writePayLoad((uint8_t*)pcmData, 192);
            packetReady = 0;
        }

    }
}


/*
 *  ======== main ========
 */
int main(void)
{
    Semaphore_Params semParams;

    /* Call board init functions. */
    Board_initGeneral();
    Board_initUART();

    CreateAudioBuffers();

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if(!ledPinHandle)
    {
        System_abort("Error initializing board LED pins\n");
    }

    /* Construct a Semaphore object to be used as a resource lock, inital count 0 */
    Semaphore_Params_init(&semParams);
    Semaphore_construct(&semRadioRxStruct, 0, &semParams);
    Semaphore_construct(&semUartRxStruct, 0, &semParams);


    // Initialize uart task
    semUartRxHandle = Semaphore_handle(&semUartRxStruct); //instance handle
    UartRxTask_init(ledPinHandle);

    // Initialize radio rx task
    semRadioRxHandle = Semaphore_handle(&semRadioRxStruct); //instance handle
    RadioRxTask_init(ledPinHandle);

    /* Start BIOS */
    BIOS_start();

    return (0);
}


//*****************************************************************************

static void uartConfigure()
{
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 921600;//460800;
    uart = UART_open(Board_UART0, &uartParams);
}

void uart_writePayLoad(uint8_t *packet, uint16_t length)
{
    UART_write(uart, packet, length);
}


static bool CreateAudioBuffers()
{
    int encodedSize = 2*ADPCM_FRAME_SIZE;
    int unpackedSize = (encodedSize * 16)/5;
    int decodedSize = unpackedSize * 2;

    encodedData = Memory_alloc(NULL, encodedSize, 0, NULL);
    unpackedData = Memory_alloc(NULL, unpackedSize, 0, NULL);
    pcmData =      Memory_alloc(NULL, decodedSize, 0, NULL);

    if ( (encodedData == NULL ) || (unpackedData == NULL) || (pcmData == NULL) )
    {
        return false;
    }

    return true;
}

static void DestroyAudioBuffers()
{
    int encodedSize = ADPCM_FRAME_SIZE;
    int unpackedSize = (encodedSize * 8)/5;
    int decodedSize = unpackedSize * 2;

    Memory_free(NULL, encodedData, encodedSize);
    Memory_free(NULL, unpackedData, unpackedSize);
    Memory_free(NULL, pcmData, decodedSize);
}
