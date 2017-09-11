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
//#include "I2C/SensorI2C.h"
#include "I2C/CS42L55.h"
#include "I2SDrv/I2SCC26XX.h"

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
    Board_DIO15 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
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

uint8_t i2sContMgtBuffer[I2S_BLOCK_OVERHEAD_IN_BYTES * I2SCC26XX_QUEUE_SIZE] = {0};

//ssize_t written = 0;
struct {
  STREAM_STATE_E streamState;
  uint8_t streamType;
  uint8_t samplesPerFrame;
} streamVariables = {STREAM_STATE_IDLE, 0, 0};


static I2SCC26XX_Handle i2sHandle = NULL;
static I2SCC26XX_StreamNotification i2sStream;


static void processAudioFrame();
static bool startAudioStream();
static void stopAudioStream();

static void i2sCallbackFxn(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *notification);


static I2SCC26XX_Params i2sParams = {
  .requestMode            = I2SCC26XX_CALLBACK_MODE,
  .ui32requestTimeout     = BIOS_WAIT_FOREVER,
  .callbackFxn            = i2sCallbackFxn,
  .blockSize              = PCM_SAMPLES_PER_FRAME,
  .pvContBuffer           = NULL,
  .ui32conBufTotalSize    = 0,
  .pvContMgtBuffer        = (void *) i2sContMgtBuffer,
  .ui32conMgtBufTotalSize = sizeof(i2sContMgtBuffer),
  .currentStream          = &i2sStream
};

static bool audioStreamInProgress = false;

static void initPWM_MClk();
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

static void i2sCallbackFxn(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *notification)
{
    if (notification->status == I2SCC26XX_STREAM_ERROR)
    {
        events |= IGOTX_AUDIO_ERROR_EVENT;
        Semaphore_post(AS_Sem);
    }
    else if (notification->status == I2SCC26XX_STREAM_BUFFER_READY)
    {
        // Provide buffer
        events |= IGOTX_AUDIO_FRAME_EVENT;
        Semaphore_post(AS_Sem);
    }
}


////////////////////////////////////////////////////////////////////////////////////////
//  Stand alonee MCLK generation
////////////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////////////
//  Audio Bus Initialisation
////////////////////////////////////////////////////////////////////////////////////////
static bool initAudioBus()
{
    i2sHandle = (I2SCC26XX_Handle)&(I2SCC26XX_config);
    I2SCC26XX_init(i2sHandle);

    //configure device
    initADCDevice(ledPinHandle);

    // start external MCLK
    initPWM_MClk();

    streamVariables.streamType = AUDIO_G722_P1;
    streamVariables.samplesPerFrame = PCM_SAMPLES_PER_FRAME;

    //create audio buffers
    if( !createAudioBuffers()){
        return false;
    }

    //initialise i2s parameters
    I2SCC26XX_Params_init( &i2sParams, I2SCC26XX_I2S, NULL );

    i2sParams.blockSize              = streamVariables.samplesPerFrame;
    i2sParams.pvContBuffer           = (void *) pcmSamples;
    i2sParams.ui32conBufTotalSize    = sizeof(int16_t) * (2*streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE);
    i2sParams.i32SampleRate = -1;
    I2SCC26XX_Handle i2sHandleTmp = I2SCC26XX_open(i2sHandle, &i2sParams);

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
    if (!I2SCC26XX_startStream(i2sHandle)){
        return false;
    }

    // Move to ACTIVE as we have completed start sequence
    streamVariables.streamState = STREAM_STATE_ACTIVE;
    return true;
}

static void stopAudioStream()
{
    if ((streamVariables.streamState != STREAM_STATE_IDLE) && (i2sHandle))
    {
        streamVariables.streamState = STREAM_STATE_STOPPING;
        /* Stop PDM stream */
        I2SCC26XX_stopStream(i2sHandle);

        /* In case no more callbacks will be made, attempt to flush remaining data now. */
        events |= IGOTX_AUDIO_FRAME_EVENT;;
    }
}

static void processAudioFrame()
{
    int16_t idx;
    int readData;
    I2SCC26XX_BufferRequest bufferRequest;
    I2SCC26XX_BufferRelease bufferRelease;
    bool gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);

    while (gotBuffer)
    {
        int16_t* pb = (int16_t*)bufferRequest.bufferIn;

        for(idx = 0; idx < streamVariables.samplesPerFrame; idx++ )
        {
            ChannelSamples[idx] = pb[2*idx+1];
        }

        readData = adpcm64_encode_run(&encoder, ChannelSamples, encodedSamples, PCM_SAMPLES_PER_FRAME);
        readData = adpcm64_pack_vadim(encodedSamples, packedSamples, readData);

        RF_cmdPropTxAdv.pPkt = (uint8_t*)packedSamples;
        RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTxAdv, RF_PriorityHigh, NULL, 0);

        bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
        bufferRelease.bufferHandleOut = NULL;
        I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);

        gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
    }
}


static bool createAudioBuffers()
{
    // Allocate memory for decoded PCM data
    pcmSamples = Memory_alloc(NULL, sizeof(int16_t) * (2*PCM_SAMPLES_PER_FRAME * I2SCC26XX_QUEUE_SIZE), 0, NULL);
    ChannelSamples = Memory_alloc(NULL, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME, 0, NULL);
    encodedSamples = Memory_alloc(NULL, sizeof(uint16_t) * PCM_SAMPLES_PER_FRAME/2, 0, NULL);
    packedSamples = Memory_alloc(NULL, sizeof(uint16_t) * 5*(PCM_SAMPLES_PER_FRAME/8), 0, NULL);

    if ( (pcmSamples == NULL ) || (ChannelSamples == NULL) || (encodedSamples == NULL) || (packedSamples == NULL))
    {
        return false;
    }
    return true;
}


static void destroyAudioBuffers()
{
    Memory_free(NULL, pcmSamples, sizeof(int16_t) * (2*PCM_SAMPLES_PER_FRAME * I2SCC26XX_QUEUE_SIZE));
    Memory_free(NULL, ChannelSamples, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME);
    Memory_free(NULL, encodedSamples, sizeof(uint16_t) * PCM_SAMPLES_PER_FRAME/2);
    Memory_free(NULL, packedSamples, sizeof(uint16_t) * 5*(PCM_SAMPLES_PER_FRAME/8));
}