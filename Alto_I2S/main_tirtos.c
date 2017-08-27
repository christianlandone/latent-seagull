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
#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>

/* PDM Driver */
#include <ti/drivers/pdm/PDMCC26XX.h>

/* Example/Board Header files */
#include "Board.h"
#include "I2C/SensorI2C.h"
#include "I2C/CS42L55.h"

#include <stdint.h>
#include <unistd.h>

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
 * LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config ledPinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_DIO15 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
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
static int totalFrameCount = 0;
static int currentFrameCount = 0;
static int sessionCount = 0;

/* Audio Protocol Define */
#define SA_PCM_BLOCK_SIZE_IN_SAMPLES       32
#define SA_PCM_BUFFER_NODE_SIZE_IN_BLOCKS   6

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


void initCodec()
{

    uint8_t devAddress = 0x95;
    uint8_t regAddress = 0x01;
    uint8_t val = 0x16;
    int i;
    uint8_t buffer[16];

    bool status;
    unsigned char readBuffer[3];
    unsigned char writeBuffer[3];

    I2C_Params i2cParams;
    I2C_Handle i2cHandle;
    I2C_Transaction i2cTransaction;

    PIN_setOutputValue(ledPinHandle, Board_DIO15, 1);

    I2C_init();
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;
    i2cHandle = I2C_open(Board_I2C0, &i2cParams);

    writeBuffer[0] = 0x01;
    //writeBuffer[1] = 0x99;
    i2cTransaction.slaveAddress = 0x95;
    i2cTransaction.writeBuf = writeBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = readBuffer;
    i2cTransaction.readCount = 1;


    for (i = 0; i < 200; i++)
    {
        //SensorI2C_readReg(regAddress, &val, 1);
        bool status = I2C_transfer(i2cHandle, &i2cTransaction);
        if (!status) {
           // Unsuccessful I2C transfer
       }

        usleep(250000);
    }

    i = 0;

    /*
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 0);
    PIN_setOutputValue(ledPinHandle, Board_DIO15, 0);

    usleep(10000);

    PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 1);
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);
    PIN_setOutputValue(ledPinHandle, Board_DIO15, 0);

    usleep(10000);

    SensorI2C_open();

    SensorI2C_select(SENSOR_I2C_0,devAddress);



    for (i = 0; i < 20; i++)
    {
        SensorI2C_readReg(regAddress, &val, 1);
        usleep(250000);
    }
    ///////////////////////////////////////////////////////////////

    regAddress = 0x00;
    val = 0x99;
    SensorI2C_writeReg(regAddress, &val, 1);

    //2. Digital Regulator.
    regAddress = 0x2E;
    val = 0x30;
    SensorI2C_writeReg(regAddress, &val, 1);

    //3. ADC.
    regAddress = 0x32;
    val = 0x07;
    SensorI2C_writeReg(regAddress, &val, 1);

    //4. ADC.
    val = 0xFF;
    SensorI2C_writeReg(0x33, &val, 1);

    //5. ADC.
    val = 0xF8;
    SensorI2C_writeReg(0x32, &val, 1);

    //6. Zero Cross Detector.
    val = 0xDC;
    SensorI2C_writeReg(0x35, &val, 1);

    //7. PGA.
    val = 0xFC;
    SensorI2C_writeReg(0x36, &val, 1);

    //8. PGA.
    val = 0xAC;
    SensorI2C_writeReg(0x37, &val, 1);

    //9. DAC.
    val = 0xF8;
    SensorI2C_writeReg(0x3A, &val, 1);

    //10. Headphone Amplifier.
    val = 0xD3;
    SensorI2C_writeReg(0x3C, &val, 1);

    //11. Headphone & Line Amplifier.
    val = 0x23;
    SensorI2C_writeReg(0x3D, &val, 1);

    //12. Line Amplifier.
    val = 0x81;
    SensorI2C_writeReg(0x3E, &val, 1);

    //13. PGA & ADC.
    val = 0x46;
    SensorI2C_writeReg(0x3F, &val, 1);

    //14. [Disable test register access.].
    val = 0x00;
    SensorI2C_writeReg(0x00, &val, 1);
    */

    /* CS42L55 INITIALISATION */

    //M_WRI_I2C DEV_CS42L55, 0x09, 0x22 //������
    val = nHPFB | nHPFRZB | HPFA | nHPFRZA | HPFB_CF00 | HPFA_CF10;
    SensorI2C_writeReg(HPF_Control, &val, 1);

    // PGAA <- AIN1A � �������� �������� 9 db
    //M_WRI_I2C DEV_CS42L55, PGAA, nBOOSTx | PGAxMUX | PGAVOLx00db
    val = 0x0;//nBOOSTx | PGAxMUX | ( (MODE_CONTROL&RECORD_CONFIG_GAIN_MASK) >> (RECORD_CONFIG_GAIN_BIT-1) );
    SensorI2C_writeReg(PGAA, &val, 1);

    // PGAB - Use
    // AIN1A; PGA is bypassed
    // Selected Input to Line Amplifier Ch.B <- DACB
    // Selected Input to Line Amplifier Ch.A <- DACA
    // Selected Input to HP Amplifier Ch.B <- DACB
    // Selected Input to HP Amplifier Ch.A <- DACA
    //M_WRI_I2C DEV_CS42L55, ADC_LINE_HP_MUX, ADCAMUX_01 | ADCBMUX_00 | LINEBMUX | LINEAMUX | HPBMUX | HPAMUX
    //if ( ( MODE_CONTROL & RECORD_CONFIG_GAIN_MASK ) == 0 )
      val = ADCAMUX_01 | ADCBMUX_00 | LINEBMUX | LINEAMUX | HPBMUX | HPAMUX;
    //else
      //val = ADCAMUX_00 | ADCBMUX_00 | LINEBMUX | LINEAMUX | HPBMUX | HPAMUX;
    SensorI2C_writeReg(ADC_LINE_HP_MUX, &val, 1);

    //LRCK <- 12MHz
    //M_WRI_I2C DEV_CS42L55, CLK_CTL2, RATIO_01 | nGROUP32k | SPEED_11
    val = RATIO_01 | nGROUP32k | SPEED_11 ;
    SensorI2C_writeReg(CLK_CTL2, &val, 1);

    // Master (Output ONLY)
    // SCLK Inverted
    // Output SCLK Re-timed, bursted signal with minimal speed needed to clock the required data samples
    // MCLK signal into CODEC No divide
    // MCLK signal into CODEC ON
    // M_WRI_I2C DEV_CS42L55, CLK_CTL1, M_S | nINV_SCLK | SCK_MCK_00 | MCLKDIV2 | nMCLKDIS
    val = M_S | nINV_SCLK | SCK_MCK_00 | nMCLKDIV2 | nMCLKDIS ;
    SensorI2C_writeReg(CLK_CTL1, &val, 1);

    // Left Channel: ADCB       Right Channel: ADCA
    // ADCB=A Disabled
    // PGAB=A Disabled
    // ADC Signal Polarity Ch.B: Not Inverted
    // ADC Signal Polarity Ch.A: Not Inverted
    // Mute on ADC channel B: Not muted.
    // Mute on ADC channel A: Not muted.
    //M_WRI_I2C DEV_CS42L55, ADC_CTL, DIGSUM_00 | nADCB_A | nPGAB_A | nINV_ADCB | nINV_ADCA | nADCBMUTE | nADCAMUTE
    val = DIGSUM_00 | nADCB_A | nPGAB_A | nINV_ADCB | nINV_ADCA | nADCBMUTE | nADCAMUTE ;
    SensorI2C_writeReg(ADC_CTL, &val, 1);

    // DSP Status: Powered Down
    // HP/Line De-Emphasis: Diabled
    // Playback Channels B=A: Disabled; Independent channel control.
    // PCM Signal Polarity A: Not Inverted
    // PCM Signal Polarity B: Not Inverted
    // Master Playback Mute Ch.B: Not muted.
    // Master Playback Mute Ch.B: Not muted.
    //  M_WRI_I2C DEV_CS42L55, Playback_CTL1, PDNDSP | nDEEMPH | nPLYBCKB_A | nINV_PCMA | nINV_PCMB | nMSTBMUTE | nMSTAMUTE
    val = PDNDSP | nDEEMPH | nPLYBCKB_A | nINV_PCMA | nINV_PCMB | nMSTBMUTE | nMSTAMUTE;
    SensorI2C_writeReg(Playback_CTL1, &val, 1);

    // ADC Charge Pump OFF
    // ADC channel B OFF
    // ADC channel A ON
    // Entire CODEC power UP
    //M_WRI_I2C DEV_CS42L55, PowerCTL1, nPDN_CHRG | nPDN_ADCB | PDN_ADCA | PDN
    val = nPDN_CHRG | nPDN_ADCB | PDN_ADCA | PDN;
    SensorI2C_writeReg(PowerCTL1, &val, 1);


    SensorI2C_close();
}

/*
 *  ======== taskFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void taskFxn(UArg arg0, UArg arg1) {
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

    initCodec();

    /*
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 0);
    PIN_setOutputValue(ledPinHandle, Board_DIO15, 0);

    usleep(100000);

    PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 1);
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);
    PIN_setOutputValue(ledPinHandle, Board_DIO15, 1);

    uint8_t rdval;
    SensorI2C_open();
    SensorI2C_select(SENSOR_I2C_0,0x95);
    SensorI2C_readReg(0x01, &rdval, 1);
    SensorI2C_close();
    */

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
                currentFrameCount = 0;
                sessionCount++;
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
        if (events & SA_PCM_BLOCK_READY) {
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
    pdmParams.micGain = PDMCC26XX_GAIN_18;
    pdmParams.applyCompression = false;
    pdmParams.startupDelayWithClockInSamples = 512;
    pdmParams.retBufSizeInBytes = AUDIO_BUF_UNCOMPRESSED_SIZE + PCM_METADATA_SIZE;
    pdmParams.mallocFxn = (PDMCC26XX_MallocFxn) SA_audioMalloc;
    pdmParams.freeFxn = (PDMCC26XX_FreeFxn) SA_audioFree;


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
 *  @fn          SA_processPDMData
 *
 * @brief        Processed the received audio packetd from PDM driver and packed
 *               to send over RF4CE link.
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

    if ( (saAudioState == SA_AUDIO_STREAMING) ||
         (saAudioState == SA_AUDIO_STARTING) ||
         (saAudioState == SA_AUDIO_STOPPING) ) {
        // Block ready, read it out and send it,
        // if we're not already sending
        while (PDMCC26XX_requestBuffer(pdmHandle, &bufferRequest))
        {
            if (bufferRequest.status == PDMCC26XX_STREAM_BLOCK_READY)
            {
                UART_write(urt, bufferRequest.buffer->pBuffer, AUDIO_BUF_UNCOMPRESSED_SIZE);

                if (!pdmParams.applyCompression)
                {

                }
                totalFrameCount++;
                currentFrameCount++;
                if ((bufferRequest.buffer->metaData.seqNum & 0x0000000F) == 0x00)
                {
                }
                else
                {
                    asm(" NOP");
                }
                if (pdmParams.applyCompression) {
                    SA_audioFree(bufferRequest.buffer, PCM_METADATA_SIZE + AUDIO_BUF_COMPRESSED_SIZE);
                } else {
                    SA_audioFree(bufferRequest.buffer, PCM_METADATA_SIZE + AUDIO_BUF_UNCOMPRESSED_SIZE);
                }
            }
        }
        /*
         * We close the driver after flushing samples, after stopping the stream.
         */
        if (saAudioState == SA_AUDIO_STOPPING) {
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
        while (PDMCC26XX_requestBuffer(pdmHandle, &bufferRequest)) {
            if (pdmParams.applyCompression) {
                SA_audioFree(bufferRequest.buffer, PCM_METADATA_SIZE + AUDIO_BUF_COMPRESSED_SIZE);
            } else {
                SA_audioFree(bufferRequest.buffer, PCM_METADATA_SIZE + AUDIO_BUF_UNCOMPRESSED_SIZE);
            }
        }
    }
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
