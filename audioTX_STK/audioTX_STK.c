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
#include <ti/drivers/PWM.h>
#include <ti/drivers/I2S.h>

/* Example/Board Header files */
#include "Board.h"

/* PDM Driver */
#include <ti/drivers/pdm/PDMCC26XX.h>

/* Encoder */
#include "G722/G722_64_Encoder.h"

/* RF Settings */
#include "IGOCommon/rfsettings/smartrf_settings.h"

/*general settings*/
#include "IGOCommon/IGOCommon.h"

#include <stdint.h>
#include <unistd.h>


#define AUDIO_ENCODE_TASK_STACKSIZE      4096
#define AUDIO_ENCODE_TASK_PRIO           1

#define RADIO_TX_TASK_STACK_SIZE    1024
#define RADIO_TX_TASK_PRIO          2

Task_Struct AEncodeTaskStruct;
Task_Struct RadioTxTaskStruct;

Char AEncodeTaskStack[AUDIO_ENCODE_TASK_STACKSIZE];
Char RadioTxTaskStack[RADIO_TX_TASK_STACK_SIZE];

Semaphore_Struct AEncodeSemStruct;
Semaphore_Struct RadioTxSemStruct;

Semaphore_Handle AEncodeSem;
Semaphore_Handle RadioTxSem;



// Internal Events for RTOS application
#define IGOTX_ZERO_EVENT                         0x0001
#define IGOTX_AUDIO_FRAME_EVENT                  0x0002
#define IGOTX_AUDIO_ERROR_EVENT                  0x0004
#define IGOTX_AUDIO_START_EVENT                  0x0008
#define IGOTX_AUDIO_STOP_EVENT                   0x0010
#define IGOTX_BUTTON_EVENT                       0x0020
//define IGOTX_XXX_XXX_EVENT                     0x0020 0x0040 0x0080 0x0100 0x0200 0x0400

// events flag for internal application events.
static uint16_t events = IGOTX_ZERO_EVENT;
uint8_t packetReady = 0;

// Global memory storage for a PIN_Config table
static PIN_State ledPinState;

// Pin driver handles
static PIN_Handle ledPinHandle;

// LED pin configuration table: - All LEDs board LEDs are off.
PIN_Config ledPinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  Board_DIO15 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  Board_DIO26 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  Board_DIO27 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX
    PIN_TERMINATE
};


typedef enum {
  STREAM_STATE_IDLE,
  STREAM_STATE_STARTING,
  STREAM_STATE_ACTIVE,
  STREAM_STATE_STOPPING,
  STREAM_STATE_ERROR,
} STREAM_STATE_E;

static G722ENCODER encoder;

static RF_Object rfObject;
static RF_Handle rfHandle;

static int16_t  *pcmSamples;
static int16_t  *ChannelSamples;
static uint16_t *encodedSamples;
static uint16_t *packedSamples;
static uint8_t  *payloadBuffer;

struct {
  STREAM_STATE_E streamState;
  uint8_t streamType;
  uint8_t samplesPerFrame;
} streamVariables = {STREAM_STATE_IDLE, 0, 0};


static void processAudioFrame();
static bool startAudioStream();
static void stopAudioStream();

static bool audioStreamInProgress = false;

//static void initPWM_MClk();
static bool initAudioBus();
static bool createAudioBuffers();
static void destroyAudioBuffers();

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//PDM drivers specific
static PDMCC26XX_Handle pdmHandle = NULL;
static void pdmCallbackFxn(PDMCC26XX_Handle handle,PDMCC26XX_StreamNotification *streamNotification);
static void *SA_audioMalloc(uint_least16_t size);
static void SA_audioFree(void *msg, size_t size);
PDMCC26XX_Params pdmParams;
static int16_t dGain;
//PDM drivers specific
//////////////////////////////////////////////////////////////////////////////////////////////////////////

static void taskFxnAudioEncode(UArg arg0, UArg arg1)
{
    adpcm64_encode_init(&encoder);

    if( !initAudioBus())
    {
        while(1);
    }

    events |= IGOTX_AUDIO_START_EVENT;

    while (1)
    {
        /////////////////////////////////////////////////////////
        if (events == IGOTX_ZERO_EVENT)
        {
            if (Semaphore_getCount(AEncodeSem) == 0) {
                System_printf("Sem blocked in task1\n");
            }
            Semaphore_pend(AEncodeSem, BIOS_WAIT_FOREVER);
        }

        /////////////////////////////////////////////////////////
        if (events & IGOTX_AUDIO_START_EVENT)
        {
            audioStreamInProgress = startAudioStream();
            events &= ~IGOTX_AUDIO_START_EVENT;
        }

        /////////////////////////////////////////////////////////
        if (events & IGOTX_AUDIO_FRAME_EVENT)
        {
            if (audioStreamInProgress)
            {
                processAudioFrame();
            }
            events &= ~IGOTX_AUDIO_FRAME_EVENT;
        }
        /////////////////////////////////////////////////////////
        if (events & IGOTX_AUDIO_STOP_EVENT)
        {
            stopAudioStream();

            events &= ~IGOTX_AUDIO_STOP_EVENT;
        }

        /////////////////////////////////////////////////////////
        if (events & IGOTX_AUDIO_ERROR_EVENT)
        {
            events &= ~IGOTX_AUDIO_ERROR_EVENT;
        }

        /////////////////////////////////////////////////////////
    }
}

static void taskFxnRadioTx(UArg arg0, UArg arg1)
{
    /////////////////////////////////////////////////////
    //Radio stuff
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    //set frequency
    RF_cmdPropRadioDivSetup.centerFreq = 0x0361;
    RF_cmdFs.frequency = 0x0361;
    RF_cmdFs.fractFreq = 0x0000;

    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);


    RF_cmdPropTxAdv.pktLen = SENSOR_PACKET_LENGTH;
    RF_cmdPropTxAdv.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropTxAdv.startTrigger.pastTrig = 1;
    RF_cmdPropTxAdv.startTime = 0;

    while (1) {
        Semaphore_pend(RadioTxSem, BIOS_WAIT_FOREVER);
        if (packetReady) {

            RF_cmdPropTxAdv.pPkt = (uint8_t*)payloadBuffer;
            RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTxAdv, RF_PriorityHigh, NULL, 0);
            packetReady = 0;

        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////
//  ======== main ========
////////////////////////////////////////////////////////////////////////////////////////
main(void) {
    Task_Params AEncodeTaskParams;
    Semaphore_Params AEncodeSemParams;

    Task_Params RadioTxTaskParams;
    Semaphore_Params RadioTxSemParams;

    // Call driver init functions
    Board_initGeneral();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // Construct BIOS objects for audio encoding task
    Task_Params_init(&AEncodeTaskParams);
    AEncodeTaskParams.priority = AUDIO_ENCODE_TASK_PRIO;
    AEncodeTaskParams.stackSize = AUDIO_ENCODE_TASK_STACKSIZE;
    AEncodeTaskParams.stack = &AEncodeTaskStack;
    Task_construct(&AEncodeTaskStruct, (Task_FuncPtr)taskFxnAudioEncode, &AEncodeTaskParams, NULL);

    // Construct a Semaphore object to be use as a resource lock, inital count 1
    Semaphore_Params_init(&AEncodeSemParams);
    Semaphore_construct(&AEncodeSemStruct, 1, &AEncodeSemParams);

    // Obtain instance handle
    AEncodeSem = Semaphore_handle(&AEncodeSemStruct);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // Construct BIOS objects for radio transmission
    Task_Params_init(&RadioTxTaskParams);
    RadioTxTaskParams.priority = RADIO_TX_TASK_PRIO;
    RadioTxTaskParams.stackSize = RADIO_TX_TASK_STACK_SIZE;
    RadioTxTaskParams.stack = &RadioTxTaskStack;
    Task_construct(&RadioTxTaskStruct, (Task_FuncPtr)taskFxnRadioTx, &RadioTxTaskParams, NULL);

    // Construct a Semaphore object to be use as a resource lock, inital count 1
    Semaphore_Params_init(&RadioTxSemParams);
    Semaphore_construct(&RadioTxSemStruct, 1, &RadioTxSemParams);

    // Obtain instance handle
    RadioTxSem = Semaphore_handle(&RadioTxSemStruct);


    // Open LED pins
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if (!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);

    // Start BIOS
    BIOS_start();

    return (0);
}


static void pdmCallbackFxn(PDMCC26XX_Handle handle, PDMCC26XX_StreamNotification *pStreamNotification)
{
    if (pStreamNotification->status == PDMCC26XX_STREAM_BLOCK_READY)
    {
        events |= IGOTX_AUDIO_FRAME_EVENT;
    }
    else if (pStreamNotification->status == PDMCC26XX_STREAM_BLOCK_READY_BUT_PDM_OVERFLOW)
    {
        events |= IGOTX_AUDIO_FRAME_EVENT;
        events |= IGOTX_AUDIO_ERROR_EVENT;
    }
    else if (pStreamNotification->status == PDMCC26XX_STREAM_STOPPING)
    {
        events |= IGOTX_AUDIO_FRAME_EVENT;
        events |= IGOTX_AUDIO_ERROR_EVENT;
    }
    else
    {
        events |= IGOTX_AUDIO_ERROR_EVENT;
    }

    Semaphore_post(AEncodeSem);
}


////////////////////////////////////////////////////////////////////////////////////////
//  Stand alonee MCLK generation
////////////////////////////////////////////////////////////////////////////////////////
/*
static void initPWM_MClk()
{
    ///////////////////////////////////////////////////////////////
    //start pwm on pin 12
    PWM_Handle pwm1 = NULL;
    PWM_Params params;

    PWM_init();

    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_FRACTION;
    params.dutyValue = 0;
    params.periodUnits = PWM_PERIOD_HZ;
    params.periodValue = 6e6;
    pwm1 = PWM_open(Board_PWM2, &params);
    if (pwm1 == NULL)
    {
        while (1);
    }

    PWM_start(pwm1);
    PWM_setDuty( pwm1, PWM_DUTY_FRACTION_MAX/2);
}
*/

////////////////////////////////////////////////////////////////////////////////////////
//  Audio Bus Initialisation
////////////////////////////////////////////////////////////////////////////////////////
static bool initAudioBus()
{
    /* Initialize PDM driver (invokes I2S) */
    PDMCC26XX_init((PDMCC26XX_Handle) &(PDMCC26XX_config));

    PDMCC26XX_Params_init(&pdmParams);
    pdmParams.callbackFxn = pdmCallbackFxn;
    pdmParams.micGain = PDMCC26XX_GAIN_24;
    pdmParams.applyCompression = false;
    pdmParams.startupDelayWithClockInSamples = 512;
    pdmParams.retBufSizeInBytes = 2* PCM_SAMPLES_PER_FRAME + PCM_METADATA_SIZE;
    pdmParams.mallocFxn = (PDMCC26XX_MallocFxn) SA_audioMalloc;
    pdmParams.freeFxn = (PDMCC26XX_FreeFxn) SA_audioFree;
    //pdmParams.pdmBufferQueueDepth = 8;

    pdmHandle = PDMCC26XX_open(&pdmParams);

    streamVariables.streamType = AUDIO_G722_P1;
    streamVariables.samplesPerFrame = PCM_SAMPLES_PER_FRAME;

    //create audio buffers
    if( !createAudioBuffers()){
        return false;
    }


    return true;
}

//////////////////////////////////////////////////////////

static bool startAudioStream()
{
    if (streamVariables.streamState != STREAM_STATE_IDLE){
        return false;
    }

    streamVariables.streamState = STREAM_STATE_STARTING;

    // Try to start I2S stream
    if (!PDMCC26XX_startStream(pdmHandle)){
        return false;
    }

    // Move to ACTIVE as we have completed start sequence
    streamVariables.streamState = STREAM_STATE_ACTIVE;
    return true;
}

static void stopAudioStream()
{
    if ((streamVariables.streamState != STREAM_STATE_IDLE) && (pdmHandle))
    {
        streamVariables.streamState = STREAM_STATE_STOPPING;
        /* Stop PDM stream */
        PDMCC26XX_stopStream(pdmHandle);;

        /* In case no more callbacks will be made, attempt to flush remaining data now. */
        events |= IGOTX_AUDIO_FRAME_EVENT;;
    }
}

static void processAudioFrame()
{
    int16_t idx;
    int readData;
    PDMCC26XX_BufferRequest bufferRequest;

    dGain = 6;

    while (PDMCC26XX_requestBuffer(pdmHandle, &bufferRequest))
    {
        if (bufferRequest.status == PDMCC26XX_STREAM_BLOCK_READY)
        {
            int16_t* pb = (int16_t*)bufferRequest.buffer->pBuffer;

            for( idx = 0; idx < PCM_SAMPLES_PER_FRAME; idx++)
            {
                ChannelSamples[idx] = dGain * pb[idx];
            }

            readData = adpcm64_encode_run(&encoder, ChannelSamples, encodedSamples, PCM_SAMPLES_PER_FRAME);
            readData = adpcm64_pack_vadim(encodedSamples, packedSamples, readData);
            memcpy( payloadBuffer+4, packedSamples, G722_P1_PAYLOAD_LENGTH );

            //direct TX
            //RF_cmdPropTxAdv.pPkt = (uint8_t*)payloadBuffer;
            //RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTxAdv, RF_PriorityHigh, NULL, 0);

            //.. or delegate to RFTX task
            packetReady = 1;
            Semaphore_post(RadioTxSem);

            SA_audioFree(bufferRequest.buffer, PCM_METADATA_SIZE + 2* PCM_SAMPLES_PER_FRAME);
        }
    }
}


static bool createAudioBuffers()
{
    // Allocate memory for decoded PCM data
    //pcmSamples = Memory_alloc(NULL, sizeof(int16_t) * (2*PCM_SAMPLES_PER_FRAME * I2SCC26XX_QUEUE_SIZE), 0, NULL);
    ChannelSamples = Memory_alloc(NULL, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME, 0, NULL);
    encodedSamples = Memory_alloc(NULL, sizeof(uint16_t) * PCM_SAMPLES_PER_FRAME/2, 0, NULL);
    packedSamples = Memory_alloc(NULL, sizeof(uint8_t) * G722_P1_PAYLOAD_LENGTH, 0, NULL);
    payloadBuffer = Memory_alloc(NULL, sizeof(uint8_t) * SENSOR_PACKET_LENGTH, 0, NULL);
    payloadBuffer[0] = 0xA7;
    payloadBuffer[1] = 0x02;
    payloadBuffer[2] = 0;
    payloadBuffer[3] = 0;

    if ( (payloadBuffer == NULL) || (ChannelSamples == NULL) || (encodedSamples == NULL) || (packedSamples == NULL))
    {
        return false;
    }

    return true;
}


static void destroyAudioBuffers()
{
    //Memory_free(NULL, pcmSamples, sizeof(int16_t) * (2*PCM_SAMPLES_PER_FRAME * I2SCC26XX_QUEUE_SIZE));
    Memory_free(NULL, ChannelSamples, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME);
    Memory_free(NULL, encodedSamples, sizeof(uint16_t) * PCM_SAMPLES_PER_FRAME/2);
    Memory_free(NULL, packedSamples, sizeof(uint8_t) * G722_P1_PAYLOAD_LENGTH);
    Memory_free(NULL, payloadBuffer, sizeof(uint8_t) * SENSOR_PACKET_LENGTH );
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
