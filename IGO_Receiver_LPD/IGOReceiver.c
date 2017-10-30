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
#include <ti/sysbios/family/arm/m3/Hwi.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include <ti/drivers/PIN.h>
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/PWM.h>

/* Board Header files */
#include "Board.h"

#include "RFQueue.h"
#include "G722/G722_64_Decoder.h"
#include "IGOCommon/IGOCommon.h"
#include "IGOCommon/rfsettings/smartrf_settings.h"
#include "RXHelperFunctions.h"

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
    Board_DIO15 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};


/* Packet RX Configuration */
#define NUM_DATA_ENTRIES       1  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */

/***** Defines *****/
#define RADIO_RX_TASK_STACK_SIZE 2048
#define RADIO_RX_TASK_PRIORITY   1

#define UART_RX_TASK_STACK_SIZE 1024
#define UART_RX_TASK_PRIORITY   2

#define UART_CMD_TASK_STACK_SIZE 512
#define UART_CMD_TASK_PRIORITY   2


/***** Prototypes *****/
static void radioRxTaskFunction(UArg arg0, UArg arg1);
static void uartRxTaskFunction(UArg arg0, UArg arg1);
static void uartCmdTaskFunction(UArg arg0, UArg arg1);


static void uartConfigure();
void uart_writePayLoad(uint8_t *packet, uint16_t length);

/***** Variable declarations *****/
static Task_Params radioRxTaskParams;
Task_Struct radioRxTask;    /* not static so you can see in ROV */
static uint8_t radioRxTaskStack[RADIO_RX_TASK_STACK_SIZE];

static Task_Params uartRxTaskParams;
Task_Struct uartRxTask;    /* not static so you can see in ROV */
static uint8_t uartRxTaskStack[UART_RX_TASK_STACK_SIZE];

static Task_Params uartCmdTaskParams;
Task_Struct uartCmdTask;    /* not static so you can see in ROV */
static uint8_t uartCmdTaskStack[UART_CMD_TASK_STACK_SIZE];

Semaphore_Struct semRadioRxStruct;
Semaphore_Handle semRadioRxHandle;

Semaphore_Struct semUartRxStruct;
Semaphore_Handle semUartRxHandle;

Semaphore_Struct semUartCmdStruct;
Semaphore_Handle semUartCmdHandle;

static RF_Object rfObject;
RF_Handle rfHandle;
RF_CmdHandle rfRxCmd;

static sysCommand sysCmd;

static BeaconPacket beaconMessage;

static uint16_t rxFrequency;

UART_Handle uart = NULL;
UART_Params uartParams;


/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
#pragma DATA_ALIGN (rxDataEntryBuffer, 4);
static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES, SENSOR_PACKET_LENGTH, NUM_APPENDED_BYTES)];

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
uint8_t packetReady = 0;
uint8_t cmdReceived = 0;
uint8_t sendCmdToTx = 0;
volatile uint8_t switchRF = 0;
static uint16_t packetLength;
static uint8_t* packetDataPointer;


static PIN_Handle pinHandle;

/* Audio stuff */
static uint8_t* encodedData;
static uint16_t* unpackedData;
static uint8_t* uartData;

//uart frame header
static char uartFrameHead[] = "RXFM";
static int frameHeadSize;

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

    frameHeadSize = sizeof(uartFrameHead);
    Task_construct(&uartRxTask, uartRxTaskFunction, &uartRxTaskParams, NULL);
}

void UartCmdTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&uartCmdTaskParams);
    uartCmdTaskParams.stackSize = UART_CMD_TASK_STACK_SIZE;
    uartCmdTaskParams.priority = UART_CMD_TASK_PRIORITY;
    uartCmdTaskParams.stack = &uartCmdTaskStack;
    uartCmdTaskParams.arg0 = (UInt)1000000;

    Task_construct(&uartCmdTask, uartCmdTaskFunction, &uartCmdTaskParams, NULL);
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

static void initRadio( uint16_t freq, uint16_t fract)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    //define data queue
    memset( rxDataEntryBuffer, 0, sizeof( rxDataEntryBuffer));

    if( RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            SENSOR_PACKET_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(1);
    }


    //Configure tex Beacon packet
    RF_cmdPropTxBeacon.pktLen = sizeof(beaconMessage);
    RF_cmdPropTxBeacon.pPkt = (uint8_t*)&beaconMessage;
    RF_cmdPropTxBeacon.startTrigger.triggerType = TRIG_ABSTIME;
    RF_cmdPropTxBeacon.startTime = 0;
    // Use the current time as an anchor point for future time stamps.
    // The Nth transmission in the future will be exactly N * RADIO_FRAME_PERIOD_MS after this time stamp.
    RF_cmdPropTxBeacon.startTime = RF_getCurrentTime();

    // A trigger in the past is triggered as soon as possible.
    // No error is given. This avoids assertion when button debouncing causes delay in TX trigger.
    RF_cmdPropTxBeacon.startTrigger.pastTrig = 1;

    beaconMessage.beaconInterval = RF_convertMsToRatTicks(RADIO_FRAME_PERIOD_MS);

    //Configure RX command
    RF_cmdPropRxAdv.pQueue = &dataQueue;           /* Set the Data Entity queue for received data */
    RF_cmdPropRxAdv.rxConf.bAutoFlushIgnored = 1;  /* Discard ignored packets from Rx queue */
    RF_cmdPropRxAdv.rxConf.bAutoFlushCrcErr = 1;   /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRxAdv.maxPktLen = SENSOR_PACKET_LENGTH;  /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRxAdv.pktConf.bRepeatOk = 0;//1;
    RF_cmdPropRxAdv.pktConf.bRepeatNok = 0;//1;

    RF_cmdPropRxAdv.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropRxAdv.startTime = 0;
    RF_cmdPropRxAdv.endTrigger.triggerType = TRIG_REL_START;
    RF_cmdPropRxAdv.endTime = RF_convertUsToRatTicks(5000);

    packetLength = SENSOR_PACKET_LENGTH;

    //open radio and set frequency
    RF_cmdFs.frequency = freq;
    RF_cmdFs.fractFreq = fract;
    RF_cmdPropRadioDivSetup.centerFreq = freq;

    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
    RF_runCmd( rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
}

static void radioRxTaskFunction(UArg arg0, UArg arg1)
{
    CreateAudioBuffers();

    // initialise SB-ADPC Decoder
    adpcm64_decode_init(&audio_decoder);

    initRadio(0x0365, 0x0000);

    while (1)
    {
        RF_cmdPropTxBeacon.startTime += RF_convertMsToRatTicks(RADIO_FRAME_PERIOD_MS);
        beaconMessage.txTime = RF_cmdPropTxBeacon.startTime;

        if( sendCmdToTx)
        {
            beaconMessage.Address0 = sysCmd.devAddr0;
            beaconMessage.Address1 = sysCmd.devAddr1;
            beaconMessage.cmdNumber = sysCmd.cmdNumber;
            beaconMessage.cmdValue = sysCmd.cmdValue;
            sendCmdToTx = 0;
        }
        else
        {
            beaconMessage.Address0 = 0;
            beaconMessage.Address1 = 0;
            beaconMessage.cmdNumber = 0;
            beaconMessage.cmdValue = 0;
        }

        //RF_cmdPropTxBeacon.pNextOp = (RF_Op*)&RF_cmdPropRxAdv;
        RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTxBeacon, RF_PriorityNormal, NULL, 0);

        //Enter RX mode and stay there for at least 2mS in RX (callback original implementation)
        //RF_EventMask
        result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRxAdv, RF_PriorityNormal, NULL, 0);//RF_EventRxEntryDone);

        if(switchRF == 1 )
        {
            packetReady = 0;
            switchRF = 0;

            RF_yield( rfHandle);
            RF_close(rfHandle);
            rfHandle = NULL;
            initRadio(rxFrequency, 0x0000);

        }

        if (((volatile RF_Op*)&RF_cmdPropRxAdv)->status == PROP_DONE_RXTIMEOUT)
        {
            //no signal received, zero data to uart
            PIN_setOutputValue(pinHandle, Board_LED2,0);
            memset(uartData+frameHeadSize, 0, packetLength);
        }
        else
        {
            //Toggle pin to indicate RX
            PIN_setOutputValue(pinHandle, Board_LED2,1);

             //Get current unhandled data entry
            currentDataEntry = RFQueue_getDataEntry();

            packetDataPointer = (uint8_t*)(&currentDataEntry->data)+0;

            memcpy(uartData+frameHeadSize, packetDataPointer, packetLength);
        }

        packetReady = 1;
        RFQueue_nextEntry();
        Semaphore_post(semUartRxHandle);
    }
}

static void uartRxTaskFunction(UArg arg0, UArg arg1)
{
    int bytesToUart = 0;

    if (uart == NULL) {
        /* Create a UART with data processing off. */
        uartConfigure();
        bytesToUart = SENSOR_PACKET_LENGTH + frameHeadSize;

        if (uart == NULL) {
            System_abort("Error opening the UART");
        }
    }

    while (1) {
        Semaphore_pend(semUartRxHandle, BIOS_WAIT_FOREVER);
        if (packetReady) {
            uart_writePayLoad( uartData, bytesToUart);
            packetReady = 0;
        }
    }
}

static void uartCmdTaskFunction(UArg arg0, UArg arg1)
{
    uint16_t newFreq;

    if (uart == NULL) {
        /* Create a UART with data processing off. */
        uartConfigure();

        if (uart == NULL) {
            System_abort("Error opening the UART");
        }
    }


    while (1) {
        Semaphore_pend(semUartCmdHandle, BIOS_WAIT_FOREVER);

        if( cmdReceived )
        {
            switch( sysCmd.deviceType)
            {
                case 0:{
                    //unassigned
                }
                break;

                case 1:{
                    //transmit packet to sensors
                    sendCmdToTx = 1;
                }
                break;

                case 2:{
                    //receiver command
                    switch( sysCmd.cmdNumber)
                    {
                        case 0:{
                        }
                        break;

                        case 1:{
                            //change operational frequency of the system
                            newFreq = rxChannelToFreq( sysCmd.cmdValue);
                            rxFrequency = newFreq;
                            switchRF = 1;
                        }
                        break;
                    }
                }
                break;

                default:{
                }
                break;
            }
        }
    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    Semaphore_Params semParamsA;
    Semaphore_Params semParamsB;
    Semaphore_Params semParamsC;

    /* Call board init functions. */
    Board_initGeneral();
    Board_initUART();

    frameHeadSize = sizeof(uartFrameHead);


    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if(!ledPinHandle)
    {
        System_abort("Error initializing board LED pins\n");
    }


    /* Construct a Semaphore object to be used as a resource lock, inital count 0 */
    Semaphore_Params_init(&semParamsA);
    Semaphore_construct(&semRadioRxStruct, 0, &semParamsA);

    Semaphore_Params_init(&semParamsB);
    Semaphore_construct(&semUartRxStruct, 0, &semParamsB);

    Semaphore_Params_init(&semParamsC);
    Semaphore_construct(&semUartCmdStruct, 0, &semParamsC);



    // Initialize radio rx task
    semRadioRxHandle = Semaphore_handle(&semRadioRxStruct); //instance handle
    RadioRxTask_init(ledPinHandle);

    // Initialize uart task
    semUartRxHandle = Semaphore_handle(&semUartRxStruct); //instance handle
    UartRxTask_init(ledPinHandle);

    // Initialize cmd task
    semUartCmdHandle = Semaphore_handle(&semUartCmdStruct); //instance handle
    UartCmdTask_init(ledPinHandle);


    /* Start BIOS */
    BIOS_start();

    return (0);
}


//*****************************************************************************
static void UART00_IRQHandler(UART_Handle handle, void *buffer, size_t num)
{
    memcpy( &sysCmd, buffer, num);
    cmdReceived = 1;
    Semaphore_post(semUartCmdHandle);
}

static void uartConfigure()
{
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 921600;
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = &UART00_IRQHandler;
    uart = UART_open(Board_UART0, &uartParams);
}

void uart_writePayLoad(uint8_t *packet, uint16_t length)
{
    char input[2];

    UART_write(uart, packet, length);

    UART_read(uart, input, sizeof(sysCommand));
}


static bool CreateAudioBuffers()
{
    int encodedSize = G722_P1_PAYLOAD_LENGTH;
    int unpackedSize = (encodedSize * 8)/5;

    encodedData  = Memory_alloc(NULL, encodedSize, 0, NULL);
    unpackedData = Memory_alloc(NULL, unpackedSize, 0, NULL);
    uartData     = Memory_alloc(NULL, SENSOR_PACKET_LENGTH + frameHeadSize, 0, NULL);
    memcpy( uartData, uartFrameHead, frameHeadSize);

    if ( (uartData == NULL ) || (encodedData == NULL ) || (unpackedData == NULL)  )
    {
        return false;
    }

    return true;
}

static void DestroyAudioBuffers()
{
    int encodedSize = G722_P1_PAYLOAD_LENGTH;
    int unpackedSize = (encodedSize * 8)/5;

    Memory_free(NULL, encodedData, encodedSize);
    Memory_free(NULL, unpackedData, unpackedSize);
    Memory_free(NULL, uartData, SENSOR_PACKET_LENGTH + frameHeadSize);
}



//callback version:
/*
rfc_CMD_TRIGGER_t triggerCmd;
configure external rx trigger
triggerCmd.commandNo = CMD_TRIGGER;
triggerCmd.triggerNo = 2;


//in radioConfig:
 RF_cmdPropRxAdv.endTrigger.triggerType = TRIG_NEVER;         // trigger.
 RF_cmdPropRxAdv.endTrigger.bEnaCmd = 1;       // Enable CMD_TRIGGER as an end trigger.
 RF_cmdPropRxAdv.endTrigger.triggerNo = 2;

//in radioRxTaskFunction

    while (1)
    {
        //Enter RX mode and stay forever in RX (callback original implementation)
        rfRxCmd = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRxAdv, RF_PriorityNormal, &radioRxCallback, IRQ_RX_ENTRY_DONE);
        Semaphore_pend(semRadioRxHandle, BIOS_WAIT_FOREVER);
        packetReady = 0;
        RF_yield( rfHandle);
        RF_close(rfHandle);
        rfHandle = NULL;
        initRadio(rxFrequency, 0x0000);
    }

static void radioRxCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxEntryDone)
    {
        //Toggle pin to indicate RX
        PIN_setOutputValue(pinHandle, Board_LED2,!PIN_getOutputValue(Board_LED2));

        //Get current unhandled data entry
        currentDataEntry = RFQueue_getDataEntry();

        //Handle the packet data, located at &currentDataEntry->data:
        packetLength      = SENSOR_PACKET_LENGTH;
        packetDataPointer = (uint8_t*)(&currentDataEntry->data)+0;

        memcpy(uartData+frameHeadSize, packetDataPointer, packetLength);

        packetReady = 1;

        RFQueue_nextEntry();

        Semaphore_post(semUartRxHandle);

    }
}
*/
