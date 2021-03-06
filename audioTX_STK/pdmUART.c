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
 *  ======== pdmstream.c ========
 */

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

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/display/Display.h>
#include <ti/drivers/UART.h>

/* PDM Driver */
#include <ti/drivers/pdm/PDMCC26XX.h>

/* Example/Board Header files */
#include "Board.h"

#include <stdint.h>

/* RF Settings */
#include "rfsettings/smartrf_settings.h"

#define TASKSTACKSIZE      1024

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];
Semaphore_Struct semStruct;
Semaphore_Handle saSem;

static RF_Object rfObject;
static RF_Handle rfHandle;

/* store the events for this application */
static uint16_t events = 0x0000;
#define SA_DEBOUNCE_COUNT_IN_MS  25

#define SA_PCM_START             0x0001
#define SA_PCM_BLOCK_READY       0x0002
#define SA_PCM_ERROR             0x0004
#define SA_PCM_STOP              0x0008
#define SA_BUTTON_EVENT          0x0010

/* Global memory storage for a PIN_Config table */
static PIN_State ledPinState;
static PIN_State buttonPinState;
/* Pin driver handles */
static PIN_Handle ledPinHandle;
static PIN_Handle buttonPinHandle;
static PIN_Id pendingPinId;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config ledPinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 *   - If the UP, DOWN, SELECT inputs are defined add them to the table.
 */
PIN_Config buttonPinTable[] = {
    Board_PIN_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_PIN_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

static bool processingButtonPress = false;
static int sampleIndex = 0;

/* Audio Protocol Define */
#define SA_PCM_BLOCK_SIZE_IN_SAMPLES       32
#define SA_PCM_BUFFER_NODE_SIZE_IN_BLOCKS   6

#define AUDIO_BUF_SAMPLES_U_SIZE    (SA_PCM_BLOCK_SIZE_IN_SAMPLES*\
                                      SA_PCM_BUFFER_NODE_SIZE_IN_BLOCKS)

#define AUDIO_BUF_COMPRESSED_SIZE   ((SA_PCM_BLOCK_SIZE_IN_SAMPLES*\
                                      SA_PCM_BUFFER_NODE_SIZE_IN_BLOCKS*\
                                     sizeof(uint16_t))/ PCM_COMPRESSION_RATE)
#define AUDIO_BUF_UNCOMPRESSED_SIZE (SA_PCM_BLOCK_SIZE_IN_SAMPLES*\
                                     SA_PCM_BUFFER_NODE_SIZE_IN_BLOCKS*\
                                     sizeof(uint16_t))

/* SA PDM drivers related */
static void SA_PDMCC26XX_callbackFxn(PDMCC26XX_Handle handle,PDMCC26XX_StreamNotification *streamNotification);
static PDMCC26XX_Handle pdmHandle = NULL;
static void RF_processPDMData(RF_Handle rf, UART_Handle urt);
static void *SA_audioMalloc(uint_least16_t size);
static void SA_audioFree(void *msg, size_t size);

static uint8_t dpacket[AUDIO_BUF_UNCOMPRESSED_SIZE];

PDMCC26XX_Params pdmParams;

/* SA audio streaming States */
typedef enum
{
  SA_AUDIO_IDLE,
  SA_AUDIO_STARTING,
  SA_AUDIO_STREAMING,
  SA_AUDIO_STOPPING,
  SA_AUDIO_ERROR
} saAudioState_t;
static saAudioState_t saAudioState = SA_AUDIO_IDLE;


void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId);

/*
 *  ======== taskFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void taskFxn(UArg arg0, UArg arg1)
{
    static int debounceCount = 0;

    UART_Handle uart;
    UART_Params uartParams;

    RF_Params rfParams;
    RF_Params_init(&rfParams);

    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 460800;

    uart = UART_open(Board_UART0, &uartParams);

    /* Loop forever */
    while (1) {

        /*
         * Do not pend on semaphore until all events have been processed
         */
        if (events == 0)
        {
            if (Semaphore_getCount(saSem) == 0) {
                System_printf("Sem blocked in task1\n");
            }

            /* Get access to resource */
            Semaphore_pend(saSem, BIOS_WAIT_FOREVER);
        }

        /*
         * Process button event, set in the pin interrupt routine
         */
        if (events & SA_BUTTON_EVENT)
        {
            /*
             * Perform debouncing without use of additional resources.
             * One could use a clock object. Instead this shows a way
             * to do it with the Task_sleep() which is a blocking call
             * which still allows the system to sleep. Only sleeping
             * 1ms at a time prevents blocking other events in this task
             * for too long. Since we're allowing other events in the
             * task to be processed this timeout is not accurate.
             * It will be greater than SA_DEBOUNCE_COUNT_IN_MS.
             */
            if (debounceCount >= SA_DEBOUNCE_COUNT_IN_MS)
            {
                if (!PIN_getInputValue(pendingPinId)) {
                    /* Toggle LED based on the button pressed */
                    switch (pendingPinId) {
                    case Board_PIN_BUTTON0:
                        events |= SA_PCM_START;
                        break;

                    case Board_PIN_BUTTON1:
                        events |= SA_PCM_STOP;
                        break;

                    default:
                        /* Do nothing */
                        break;
                    }
                }
                uint32_t tmpPinId = pendingPinId;
                pendingPinId = PIN_UNASSIGNED;
                debounceCount = 0;
                /* Mark event as processed */
                events &= ~SA_BUTTON_EVENT;

                /* Re-enable interrupts */
                processingButtonPress = false;
                PIN_setInterrupt(buttonPinHandle, tmpPinId | PIN_IRQ_NEGEDGE);
            }
            else
            {
                /*
                 * Wait 1ms before moving on.
                 */
                Task_sleep(1000 / Clock_tickPeriod);
                debounceCount++;
            }
        }

        /*
         * Process the PDM stream start event
         */
        if (events & SA_PCM_START) {
            if (pdmHandle == NULL)
            {
                /* Open PDM driver */
                pdmHandle = PDMCC26XX_open(&pdmParams);
            }
            if (pdmHandle && (saAudioState == SA_AUDIO_IDLE)) {
                saAudioState = SA_AUDIO_STARTING;

                /* Stream immediately if we simply dump over UART. */
                PDMCC26XX_startStream(pdmHandle);
                PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 0);
                saAudioState = SA_AUDIO_STREAMING;
            }
            /* Mark event as processed */
            events &= ~SA_PCM_START;
        }

        /*
         * Process event to stop PDM stream
         */
        if (events & SA_PCM_STOP)
        {
            if ((saAudioState != SA_AUDIO_IDLE) && (pdmHandle))
            {
                saAudioState = SA_AUDIO_STOPPING;
                /* Stop PDM stream */
                PDMCC26XX_stopStream(pdmHandle);
                /* In case no more callbacks will be made, attempt to flush remaining data now. */
                events |= SA_PCM_BLOCK_READY;
            }
            events &= ~SA_PCM_STOP;
        }

        /*
         * Process PDM block ready event. This event is set in the
         * PDM driver callback function.
         */
        if (events & SA_PCM_BLOCK_READY)
        {
            RF_processPDMData(rfHandle, uart);
            /* Mark event as processed */
            events &= ~SA_PCM_BLOCK_READY;
        }

        /*
         * Process the PCM error event.
         */
        if (events & SA_PCM_ERROR) {
            /* Stop stream if not already stopped */
            if ((saAudioState == SA_AUDIO_STREAMING) ||
                (saAudioState == SA_AUDIO_STARTING)) {

                events |= SA_PCM_STOP;
            }
            events &= ~SA_PCM_ERROR;
        }
    }
}

/*
 *  ======== main ========
 */
int main(void) {
    Task_Params taskParams;
    Semaphore_Params semParams;

    /* Call driver init functions */
    Board_initGeneral();
    UART_init();
    /*Display_init();*/

    /* Construct BIOS objects */
    Task_Params_init(&taskParams);
    taskParams.priority = 1;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)taskFxn, &taskParams, NULL);

    /* Initialize PDM driver (invokes I2S) */
    PDMCC26XX_init((PDMCC26XX_Handle) &(PDMCC26XX_config));

    PDMCC26XX_Params_init(&pdmParams);
    pdmParams.callbackFxn = SA_PDMCC26XX_callbackFxn;
    pdmParams.micGain = PDMCC26XX_GAIN_24;
    pdmParams.applyCompression = false;
    pdmParams.startupDelayWithClockInSamples = 512;
    pdmParams.retBufSizeInBytes = AUDIO_BUF_UNCOMPRESSED_SIZE + PCM_METADATA_SIZE;
    pdmParams.mallocFxn = (PDMCC26XX_MallocFxn) SA_audioMalloc;
    pdmParams.freeFxn = (PDMCC26XX_FreeFxn) SA_audioFree;
    //pdmParams.pdmBufferQueueDepth = 8;


    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if (!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);

    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if(!buttonPinHandle) {
        System_abort("Error initializing button pins\n");
    }

    /* Setup callback for button pins */
    if (PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0) {
        System_abort("Error registering button callback function");
    }

    /* Construct a Semaphore object to be use as a resource lock, inital count 1 */
    Semaphore_Params_init(&semParams);
    Semaphore_construct(&semStruct, 1, &semParams);

    /* Obtain instance handle */
    saSem = Semaphore_handle(&semStruct);

    /* This example has logging and many other debug capabilities enabled */
    System_printf("This example does not attempt to minimize code or data "
                  "footprint\n");
    System_flush();

    System_printf("Starting the PDM Stream example\nSystem provider is set to "
                  "SysMin. Halt the target to view any SysMin contents in "
                  "ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

/**
 *  @fn          SA_PDMCC26XX_callbackFxn
 *
 *  @brief       Application callback function to handle notifications from PDM
 *               driver.
 *
 *  @param[in]   handle - PDM driver handle
 *               pStreamNotification - voice data stream
 *  @param[out]  None
 *
 *  @return  None.
 */
static int callbackCount = 0;
static void SA_PDMCC26XX_callbackFxn(PDMCC26XX_Handle handle, PDMCC26XX_StreamNotification *pStreamNotification) {
    if (pStreamNotification->status == PDMCC26XX_STREAM_BLOCK_READY) {
        events |= SA_PCM_BLOCK_READY;
        callbackCount++;
    } else if (pStreamNotification->status == PDMCC26XX_STREAM_BLOCK_READY_BUT_PDM_OVERFLOW) {
        events |= SA_PCM_BLOCK_READY;
        events |= SA_PCM_ERROR;
    } else if (pStreamNotification->status == PDMCC26XX_STREAM_STOPPING) {
        events |= SA_PCM_BLOCK_READY;
        events |= SA_PCM_ERROR;
    } else {
        events |= SA_PCM_ERROR;
    }

    Semaphore_post(saSem);
}

static int mallocCount = 0;
static void *SA_audioMalloc(uint_least16_t size)
{
    Error_Block eb;
    Error_init(&eb);
    if (size > 64)
    {
        mallocCount++;
    }
    return Memory_alloc(NULL, size, 0, &eb);
}

static int freeCount = 0;
static void SA_audioFree(void *msg, size_t size)
{
    if (size > 64)
    {
        freeCount++;
    }
    Memory_free(NULL, msg, size);
}



/*
 *  ======== buttonCallbackFxn ========
 *  Pin interrupt Callback function board buttons configured in the pinTable.
 *  If Board_PIN_LED3 and Board_PIN_LED4 are defined, then we'll add them to the PIN
 *  callback function.
 */
void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId) {
    if (!processingButtonPress)
    {
        /*
         * Debounce logic is implemented in the task.
         * Disable interrupts while debouncing
         */
        PIN_setInterrupt(handle, pinId | PIN_IRQ_DIS);
        pendingPinId = pinId;
        events |= SA_BUTTON_EVENT;
        if (Semaphore_getCount(saSem) == 0) {
            processingButtonPress = true;
            Semaphore_post(saSem);
        }
    }
}


/**
 *  @fn          RF_processPDMData
 *
 * @brief        Processed the received audio packetd from PDM driver and packed
 *               to send over RF link.
 *
 * @param[in]    None.
 *
 * @param[out]   None.
 *
 * @return       None.
 *
 */

static void RF_processPDMData(RF_Handle rf, UART_Handle urt)
{
    PDMCC26XX_BufferRequest bufferRequest;

    RF_cmdPropTxAdv.pktLen = AUDIO_BUF_UNCOMPRESSED_SIZE;
    RF_cmdPropTxAdv.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropTxAdv.startTrigger.pastTrig = 1;
    RF_cmdPropTxAdv.startTime = 0;
    /*RF_cmdPropTxAdv.pPkt = dpacket;*/

    if ( (saAudioState == SA_AUDIO_STREAMING) ||
         (saAudioState == SA_AUDIO_STARTING) ||
         (saAudioState == SA_AUDIO_STOPPING) ) {
        // Block ready, read it out and send it,
        // if we're not already sending
        while (PDMCC26XX_requestBuffer(pdmHandle, &bufferRequest))
        {
            if (bufferRequest.status == PDMCC26XX_STREAM_BLOCK_READY)
            {
                RF_cmdPropTxAdv.pPkt = bufferRequest.buffer->pBuffer;
                RF_EventMask result = RF_runCmd(rf, (RF_Op*)&RF_cmdPropTxAdv, RF_PriorityHigh, NULL, 0);

                //UART_write(urt, dpacket, AUDIO_BUF_UNCOMPRESSED_SIZE);

                if (pdmParams.applyCompression)
                {
                    SA_audioFree(bufferRequest.buffer, PCM_METADATA_SIZE + AUDIO_BUF_COMPRESSED_SIZE);
                }
                else
                {
                    SA_audioFree(bufferRequest.buffer, PCM_METADATA_SIZE + AUDIO_BUF_UNCOMPRESSED_SIZE);
                }
            }
        }
        /*
         * We close the driver after flushing samples, after stopping the stream.
         */
        if (saAudioState == SA_AUDIO_STOPPING)
        {
            saAudioState = SA_AUDIO_IDLE;

            /* Close PDM driver */
            PDMCC26XX_close(pdmHandle);
            pdmHandle = NULL;
            PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);
            PIN_setOutputValue(ledPinHandle, Board_PIN_LED2, 0);
        }
    }
    else if (saAudioState == SA_AUDIO_IDLE)
    {
        // We may have received a callback for data after stopping the stream. Simply flush it.
        while (PDMCC26XX_requestBuffer(pdmHandle, &bufferRequest))
        {
            if (pdmParams.applyCompression)
            {
                SA_audioFree(bufferRequest.buffer, PCM_METADATA_SIZE + AUDIO_BUF_COMPRESSED_SIZE);
            }
            else
            {
                SA_audioFree(bufferRequest.buffer, PCM_METADATA_SIZE + AUDIO_BUF_UNCOMPRESSED_SIZE);
            }
        }
    }
}

