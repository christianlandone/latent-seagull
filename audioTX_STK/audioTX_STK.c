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
#include "rfsettings/smartrf_settings.h"


#include <stdint.h>
#include <unistd.h>


#define AUDIOSTREAM_STACKSIZE      2048

Task_Struct AStreamTaskStruct;
Char AStreamTaskStack[AUDIOSTREAM_STACKSIZE];
Semaphore_Struct AS_SemStruct;
Semaphore_Handle AS_Sem;

#define AUDIO_G722_P1              0x05


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

// Global memory storage for a PIN_Config table
static PIN_State ledPinState;

// Pin driver handles
static PIN_Handle ledPinHandle;

// LED pin configuration table: - All LEDs board LEDs are off.
PIN_Config ledPinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    //Board_DIO15 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};


#define PCM_SAMPLES_PER_FRAME    96

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


static int16_t *pcmSamples;
static int16_t *ChannelSamples;
static uint16_t *encodedSamples;
static uint16_t *packedSamples;

/* SA PDM drivers related */
static PDMCC26XX_Handle pdmHandle = NULL;
static void pdmCallbackFxn(PDMCC26XX_Handle handle,PDMCC26XX_StreamNotification *streamNotification);
static void *SA_audioMalloc(uint_least16_t size);
static void SA_audioFree(void *msg, size_t size);
PDMCC26XX_Params pdmParams;
static int16_t dGain;


//ssize_t written = 0;
struct {
  STREAM_STATE_E streamState;
  uint8_t streamType;
  uint8_t samplesPerFrame;
} streamVariables = {STREAM_STATE_IDLE, 0, 0};


static void processAudioFrame();
static bool startAudioStream();
static void stopAudioStream();

static bool audioStreamInProgress = false;

static bool initAudioBus();
static bool createAudioBuffers();
static void destroyAudioBuffers();


static void taskFxnAStream(UArg arg0, UArg arg1)
{
    adpcm64_encode_init(&encoder);

    /////////////////////////////////////////////////////
    //Radio stuff
    RF_Params rfParams;
    RF_Params_init(&rfParams);
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);


    RF_cmdPropTxAdv.pktLen = (PCM_SAMPLES_PER_FRAME * 5)/8 ;
    RF_cmdPropTxAdv.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropTxAdv.startTrigger.pastTrig = 1;
    RF_cmdPropTxAdv.startTime = 0;


    initAudioBus();

    events |= IGOTX_AUDIO_START_EVENT;

    while (1)
    {
        /////////////////////////////////////////////////////////
        if (events == IGOTX_ZERO_EVENT)
        {
            if (Semaphore_getCount(AS_Sem) == 0) {
                System_printf("Sem blocked in task1\n");
            }
            Semaphore_pend(AS_Sem, BIOS_WAIT_FOREVER);
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

////////////////////////////////////////////////////////////////////////////////////////
//  ======== main ========
////////////////////////////////////////////////////////////////////////////////////////

main(void) {
    Task_Params AStreamTaskParams;
    Semaphore_Params AS_SemParams;

    /* Call driver init functions */
    Board_initGeneral();

    // Construct BIOS objects for I2S Stream task
    Task_Params_init(&AStreamTaskParams);
    AStreamTaskParams.priority = 1;
    AStreamTaskParams.stackSize = AUDIOSTREAM_STACKSIZE;
    AStreamTaskParams.stack = &AStreamTaskStack;
    Task_construct(&AStreamTaskStruct, (Task_FuncPtr)taskFxnAStream, &AStreamTaskParams, NULL);


    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if (!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);


    /* Construct a Semaphore object to be use as a resource lock, inital count 1 */
    Semaphore_Params_init(&AS_SemParams);
    Semaphore_construct(&AS_SemStruct, 1, &AS_SemParams);

    /* Obtain instance handle */
    AS_Sem = Semaphore_handle(&AS_SemStruct);

    /* Start BIOS */
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
    } else
    {
        events |= IGOTX_AUDIO_ERROR_EVENT;
    }

    Semaphore_post(AS_Sem);
}


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

            RF_cmdPropTxAdv.pPkt = (uint8_t*)packedSamples;
            RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTxAdv, RF_PriorityHigh, NULL, 0);

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
    packedSamples = Memory_alloc(NULL, sizeof(uint16_t) * 5*(PCM_SAMPLES_PER_FRAME/8), 0, NULL);

    //if ( (pcmSamples == NULL ) || (ChannelSamples == NULL) || (encodedSamples == NULL) || (packedSamples == NULL))
    if ((ChannelSamples == NULL) || (encodedSamples == NULL) || (packedSamples == NULL))
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
    Memory_free(NULL, packedSamples, sizeof(uint16_t) * 5*(PCM_SAMPLES_PER_FRAME/8));
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
