#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
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

#include <ti/drivers/rf/RF.h>
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_data_entry.h)
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Example/Board Header files */
#include "Board.h"
#include "I2C/CS42L55.h"
#include "I2SDrv/I2SCC26XX.h"

/* Encoder */
#include "G722/G722_64_Encoder.h"

/* RF Settings */
#include "IGOCommon/rfsettings/smartrf_settings.h"

/*general settings*/
#include "IGOCommon/IGOCommon.h"

//device address in network
#define DEV_ADDR0 123
#define DEV_ADDR1 001

#define RX_BEACON_END_TIME  (RX_START_TO_SETTLE_TICKS + RX_TIMEOUT_TICKS + RX_TIMEOUT_MARGIN + RX_START_MARGIN);
#define TX_SLOT_US 500;

//beacon stuff
#define NUM_APPENDED_BYTES     2

#define SYMBOL_RATE            500000                    /* 500 kbits per second */
#define US_PER_SYMBOL          (1000000 / SYMBOL_RATE)
#define PREAMBLE_BITS          32
#define SYNCWORD_BITS          32

//An arbitrarily chosen value to compensate for the potential drift of the RAT and the RTC.
#define RX_START_MARGIN        RF_convertUsToRatTicks(100)
//Tight, but ideal duration for receiving all bits of the preamble and the sync word.
#define RX_TIMEOUT_TICKS       RF_convertUsToRatTicks((PREAMBLE_BITS + SYNCWORD_BITS) * US_PER_SYMBOL)
//Arbitrarily chosen margin added to the RX timeout to compensate calculation errors.
#define RX_TIMEOUT_MARGIN      RF_convertUsToRatTicks(100)
//Time between RX start trigger and the radio being ready to receive the first preamble bit. This is a fixed value for CMD_PROP_RX
#define RX_START_TO_SETTLE_TICKS   256
//Time between TX start trigger and first bit on air. This is a fixed value for CMD_PROP_TX.
#define TX_START_TO_PREAMBLE_TICKS 384

#define AUDIO_ENCODE_TASK_STACKSIZE      4096
#define AUDIO_ENCODE_TASK_PRIO           1

#define RADIO_TX_TASK_STACK_SIZE    2048
#define RADIO_TX_TASK_PRIO          2


Task_Struct AEncodeTaskStruct;
Task_Struct RadioTxTaskStruct;

Char AEncodeTaskStack[AUDIO_ENCODE_TASK_STACKSIZE];
Char RadioTxTaskStack[RADIO_TX_TASK_STACK_SIZE];

Semaphore_Struct AEncodeSemStruct;
Semaphore_Struct RadioTxSemStruct;

Semaphore_Handle AEncodeSem;
Semaphore_Handle RadioTxSem;


// Radio thread states:
#define IGO_RF_STATE_IDLE                   0x0001
#define IGO_RF_STATE_SEARCHING              0x0002
#define IGO_RF_STATE_SYNCED                 0x0004
#define IGO_RF_STATE_TEST                   0x0008
static uint16_t rfState = IGO_RF_STATE_IDLE;

// received beacon packets outcome
#define IGO_BEACON_OK       0x0001
#define IGO_BEACON_KO       0x0002
#define IGO_BEACON_MISSED   0x0004
static uint16_t beaconRxOutcome = IGO_BEACON_MISSED;

// Internal Events for audio acquisition and encoding thread
#define IGOTX_ZERO_EVENT                         0x0001
#define IGOTX_AUDIO_FRAME_EVENT                  0x0002
#define IGOTX_AUDIO_ERROR_EVENT                  0x0004
#define IGOTX_AUDIO_START_EVENT                  0x0008
#define IGOTX_AUDIO_STOP_EVENT                   0x0010
//define IGOTX_XXX_XXX_EVENT                     0x0020 0x0040 0x0080 0x0100 0x0200 0x0400
// events flag for internal application events.
static uint16_t events = IGOTX_ZERO_EVENT;

//flag signals that a data packet is ready to be transmitted
uint8_t packetReady = 0;

// Global memory storage for a PIN_Config table
static PIN_State ledPinState;

// Pin driver handles
static PIN_Handle ledPinHandle;

// LED pin configuration table: - All LEDs board LEDs are off.
PIN_Config ledPinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_DIO15 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_DIO26 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_DIO27 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
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

//beacon stuff
static int noPosBeacons = 0;
static int noLosBeacons = 0;

static BeaconPacket beaconMessage;
// Queue object that the RF Core will fill with data
static dataQueue_t rxQueue;
// A single queue item that points to a data buffer
static rfc_dataEntryPointer_t rxItem;
// Word-aligned buffer for the packet payload + meta data. */
static uint8_t rxBuffer[((sizeof(BeaconPacket) + NUM_APPENDED_BYTES) + 8)];

static int16_t  *pcmSamples;
static int16_t  *ChannelSamples;
static uint16_t *encodedSamples;
static uint16_t *packedSamples;
static uint8_t  *payloadBuffer;
static uint8_t  *dummyBuffer;

//commands related
static uint8_t txEnabled = 1;
static uint8_t audioEnabled = 0;
static uint16_t txSlotStart = TX_SLOT_US;

struct {
  STREAM_STATE_E streamState;
  uint8_t streamType;
  uint8_t samplesPerFrame;
} streamVariables = {STREAM_STATE_IDLE, 0, 0};



static void processAudioFrame();
static bool startAudioStream();
static void stopAudioStream();

static bool audioStreamInProgress = false;

static void taskFxnAudioEncode(UArg arg0, UArg arg1);
static void taskFxnRadioTx(UArg arg0, UArg arg1);

void WaitForSyncedRxState_rxCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
void SyncedRxState_rxCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

static bool initRadioSystem();
static bool initAudioBus();

static void initPWM_MClk();
static bool createAudioBuffers();
static void destroyAudioBuffers();

///////////////////////////////////////////////////////////////////////////////////////////////////
//I2S Specific
static void i2sCallbackFxn(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *notification);
uint8_t i2sContMgtBuffer[I2S_BLOCK_OVERHEAD_IN_BYTES * I2SCC26XX_QUEUE_SIZE] = {0};
static I2SCC26XX_Handle i2sHandle = NULL;
static I2SCC26XX_StreamNotification i2sStream;
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
//I2S Specific
///////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////
//  ======== main ========
////////////////////////////////////////////////////////////////////////////////////////
main(void)
{
    Task_Params AEncodeTaskParams;
    Semaphore_Params AEncodeSemParams;

    Task_Params RadioTxTaskParams;
    Semaphore_Params RadioTxSemParams;

    // Call driver init functions
    Board_initGeneral();

    // Open LED pins
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if (!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    //initRadioSystem();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // Construct BIOS objects for audio encoding task
    Task_Params_init(&AEncodeTaskParams);
    AEncodeTaskParams.priority = AUDIO_ENCODE_TASK_PRIO;
    AEncodeTaskParams.stackSize = AUDIO_ENCODE_TASK_STACKSIZE;
    AEncodeTaskParams.stack = &AEncodeTaskStack;
    Task_construct(&AEncodeTaskStruct, (Task_FuncPtr)taskFxnAudioEncode, &AEncodeTaskParams, NULL);

    // Construct a Semaphore object to be use as a resource lock, inital count 1
    Semaphore_Params_init(&AEncodeSemParams);
    Semaphore_construct(&AEncodeSemStruct, 0, &AEncodeSemParams);

    // Obtain instance handle
    AEncodeSem = Semaphore_handle(&AEncodeSemStruct);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // Construct BIOS objects for radio transmission
    Task_Params_init(&RadioTxTaskParams);
    RadioTxTaskParams.priority = RADIO_TX_TASK_PRIO;
    RadioTxTaskParams.stackSize = RADIO_TX_TASK_STACK_SIZE;
    RadioTxTaskParams.stack = &RadioTxTaskStack;
    Task_construct(&RadioTxTaskStruct, (Task_FuncPtr)taskFxnRadioTx, &RadioTxTaskParams, NULL);

    // Construct a Semaphore object to be use as a resource lock
    Semaphore_Params_init(&RadioTxSemParams);
    Semaphore_construct(&RadioTxSemStruct, 0, &RadioTxSemParams);

    // Obtain instance handle
    RadioTxSem = Semaphore_handle(&RadioTxSemStruct);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////

    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);

    // Start BIOS
    BIOS_start();

    return (0);
}

static void taskFxnAudioEncode(UArg arg0, UArg arg1)
{
    noPosBeacons = 0;

    adpcm64_encode_init(&encoder);

    if( !initAudioBus())
    {
        while(1);
    }

    events = IGOTX_AUDIO_START_EVENT;//IGOTX_ZERO_EVENT;


    while (1)
    {
        /////////////////////////////////////////////////////////
        if (events & IGOTX_ZERO_EVENT)
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
            events = IGOTX_ZERO_EVENT;
            stopAudioStream();

            //events &= ~IGOTX_AUDIO_STOP_EVENT;

        }

        /////////////////////////////////////////////////////////
        if (events & IGOTX_AUDIO_ERROR_EVENT)
        {
            events &= ~IGOTX_AUDIO_ERROR_EVENT;
        }
        /////////////////////////////////////////////////////////
    }
}
void WaitForSyncedRxState_rxCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventLastCmdDone)
    {
        if ((RF_cmdPropRxBeacon.status == PROP_DONE_OK) || (RF_cmdPropRxBeacon.status == PROP_DONE_ENDED))
        {
            beaconRxOutcome = IGO_BEACON_OK;
            Semaphore_post(RadioTxSem);
        }
        else
        {
            beaconRxOutcome = IGO_BEACON_MISSED;
            Semaphore_post(RadioTxSem);
        }
    }
}

void SyncedRxState_rxCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventLastCmdDone)
    {
        if ((RF_cmdPropRxBeacon.status == PROP_DONE_OK) || (RF_cmdPropRxBeacon.status == PROP_DONE_ENDED))
        {
            beaconRxOutcome = IGO_BEACON_OK;
            Semaphore_post(RadioTxSem);
        }
        else if (RF_cmdPropRxBeacon.status == PROP_DONE_RXTIMEOUT)
        {
            beaconRxOutcome = IGO_BEACON_MISSED;
            Semaphore_post(RadioTxSem);
        }
    }
}


static bool initRadioSystem()
{
    bool retVal = false;

    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_cmdPropRadioDivSetup.centerFreq = 0x365;
    RF_cmdFs.frequency = 0x0365;
    RF_cmdFs.fractFreq = 0x0000;

    RF_cmdPropTxAdv.pktLen = SENSOR_PACKET_LENGTH;
    RF_cmdPropTxAdv.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropTxAdv.startTrigger.pastTrig = 1;
    RF_cmdPropTxAdv.startTime = 0;

    rxItem.config.type = DATA_ENTRY_TYPE_PTR;
    rxItem.config.lenSz = 0;
    rxItem.length = sizeof(rxBuffer);
    rxItem.pNextEntry = (uint8_t*)&rxItem;
    rxItem.pData = (uint8_t*)&rxBuffer[0];
    rxItem.status = DATA_ENTRY_PENDING;
    rxQueue.pCurrEntry = (uint8_t*)&rxItem;
    rxQueue.pLastEntry = NULL;

    /* Modify CMD_PROP_RX command for application needs */
    RF_cmdPropRxBeacon.pQueue = &rxQueue;                /* Set the Data Entity queue for received data */
    RF_cmdPropRxBeacon.rxConf.bAutoFlushIgnored = true;  /* Discard ignored packets from Rx queue */
    RF_cmdPropRxBeacon.rxConf.bAutoFlushCrcErr = true;   /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRxBeacon.rxConf.bIncludeHdr = true;        /* Put length field in front of queue entries. */
    RF_cmdPropRxBeacon.rxConf.bAppendTimestamp = true;   /* Append RX time stamp to the packet payload */
    RF_cmdPropRxBeacon.maxPktLen = sizeof(BeaconPacket); /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRxBeacon.pktConf.bRepeatOk = false;        /* Stop after receiving a single valid packet */
    RF_cmdPropRxBeacon.pktConf.bRepeatNok = true;

    beaconMessage.beaconInterval = RF_convertMsToRatTicks(RADIO_FRAME_PERIOD_MS);

    if( rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams) )
    {
        RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
        retVal = true;
    }

    return retVal;
}

static void taskFxnRadioTx(UArg arg0, UArg arg1)
{
    /////////////////////////////////////////////////////
    //Radio stuff
    initRadioSystem();

    dummyBuffer = Memory_alloc(NULL, sizeof(uint8_t) * SENSOR_PACKET_LENGTH, 0, NULL);

    rfState = IGO_RF_STATE_SEARCHING;

    while (1)
    {
        /////////////////////////////////////////////////////////
        //the transmitter has been activated, We are waiting for the first beacon
        if (rfState == IGO_RF_STATE_SEARCHING)
        {
            PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 0);
            rxItem.status = DATA_ENTRY_PENDING;
            // Start RX command to receive a single packet. The receiver is switched on immediately and will not time out until a valid
            // packet is received.
            RF_cmdPropRxBeacon.startTrigger.triggerType = TRIG_NOW;
            RF_cmdPropRxBeacon.endTrigger.triggerType = TRIG_NEVER;
            RF_CmdHandle cmd = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRxBeacon, RF_PriorityNormal, &WaitForSyncedRxState_rxCallback, RF_EventRxEntryDone);

            Semaphore_pend(RadioTxSem, BIOS_WAIT_FOREVER);

            uint8_t length;
            uint32_t rxTime;

            memcpy(&length, rxBuffer, 1);
            if (length == sizeof(BeaconPacket))
            {
                PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 0);

                noPosBeacons++;

                if( noPosBeacons > NUMBER_FOUND_BEACONS_SYNC )
                {
                    memcpy(&beaconMessage, (uint8_t*)rxBuffer + 1, sizeof(BeaconPacket));
                    memcpy(&rxTime, (uint8_t*)rxBuffer + 1 + sizeof(BeaconPacket), 4);
                    RF_cmdPropRxBeacon.startTime = rxTime - RX_START_TO_SETTLE_TICKS - RX_START_MARGIN;

                    RF_cmdPropTxAdv.startTrigger.triggerType = TRIG_ABSTIME;
                    RF_cmdPropTxAdv.startTrigger.pastTrig = 1;
                    RF_cmdPropTxAdv.startTime = RF_cmdPropRxBeacon.startTime + RF_convertUsToRatTicks(txSlotStart);

                    noLosBeacons = 0;
                    rfState = IGO_RF_STATE_TEST;
                    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);
                }
            }
        }

        /////////////////////////////////////////////////////////
        if (rfState == IGO_RF_STATE_SYNCED)
        {
            rxItem.status = DATA_ENTRY_PENDING;
            //so we found a signal, and we know it's a valid beacon from the base station, we schedule rx at the frame interval
            RF_cmdPropRxBeacon.startTrigger.triggerType = TRIG_ABSTIME;
            RF_cmdPropRxBeacon.startTime += RF_convertMsToRatTicks(RADIO_FRAME_PERIOD_MS);//beaconMessage.beaconInterval;
            RF_cmdPropRxBeacon.endTrigger.triggerType = TRIG_REL_START;
            RF_cmdPropRxBeacon.endTime = RX_BEACON_END_TIME;

            RF_CmdHandle cmd = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRxBeacon, RF_PriorityNormal, &SyncedRxState_rxCallback, RF_EventRxEntryDone);

            RF_cmdPropTxAdv.startTrigger.triggerType = TRIG_ABSTIME;
            RF_cmdPropTxAdv.startTime += RF_convertMsToRatTicks(RADIO_FRAME_PERIOD_MS);//beaconMessage.beaconInterval;

            if(txEnabled){
                if(audioEnabled){
                    RF_cmdPropTxAdv.pPkt = (uint8_t*)payloadBuffer;
                }
                else{
                    RF_cmdPropTxAdv.pPkt = (uint8_t*)dummyBuffer;
                }
                RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropTxAdv, RF_PriorityNormal, NULL, 0);
            }

            Semaphore_pend(RadioTxSem, BIOS_WAIT_FOREVER);

            if( beaconRxOutcome == IGO_BEACON_MISSED)
            {
                noLosBeacons++;
                //too many beacons missed go back to synchronisation stage
                if( noLosBeacons > NUMBER_LOST_BEACONS_UNSYNC)
                {
                    noPosBeacons = 0;
                    rfState = IGO_RF_STATE_SEARCHING;
                }
            }
            else if( beaconRxOutcome == IGO_BEACON_OK)
            {
                uint8_t length;
                uint32_t rxTime;

                memcpy(&length, rxBuffer, 1);
                memcpy(&beaconMessage, (uint8_t*)rxBuffer + 1, sizeof(BeaconPacket));
                memcpy(&rxTime, (uint8_t*)rxBuffer + 1 + sizeof(BeaconPacket), 4);
                PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);
                noLosBeacons = 0;

                if( (beaconMessage.Address0 == DEV_ADDR0) && (beaconMessage.Address1 == DEV_ADDR1))
                {
                    switch( beaconMessage.cmdNumber)
                    {
                        case 0:{
                        }
                        break;

                        case 1:{
                            txEnabled = beaconMessage.cmdValue;
                        }
                        break;

                        case 2:{
                            audioEnabled = beaconMessage.cmdValue;
                        }
                        break;

                        case 3:{
                            txSlotStart = 100 * beaconMessage.cmdValue;
                            rfState = IGO_RF_STATE_SEARCHING;
                        }
                        break;
                    }

                }
            }
        }
        /////////////////////////////////////////////////////////
        if (rfState == IGO_RF_STATE_TEST)
        {
            //so we found a signal, and we know it's a valid beacon from the base station, we schedule rx at the frame interval
            rxItem.status = DATA_ENTRY_PENDING;

            ///////////////////////////////////////////////////////////////////////
            //receive bEacon
            RF_cmdPropRxBeacon.startTrigger.triggerType = TRIG_ABSTIME;
            RF_cmdPropRxBeacon.startTime += RF_convertMsToRatTicks(RADIO_FRAME_PERIOD_MS);//beaconMessage.beaconInterval;
            RF_cmdPropRxBeacon.endTrigger.triggerType = TRIG_REL_START;
            RF_cmdPropRxBeacon.endTime = RX_BEACON_END_TIME;
            RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRxBeacon, RF_PriorityNormal, NULL, 0);
            if (RF_cmdPropRxBeacon.status == PROP_DONE_RXTIMEOUT)
            {
                //too many beacons missed go back to synchronisation stage
                if( noLosBeacons++> NUMBER_LOST_BEACONS_UNSYNC){
                    noPosBeacons = 0;
                    rfState = IGO_RF_STATE_SEARCHING;
                }
            }
            else if ((RF_cmdPropRxBeacon.status == PROP_DONE_OK) || (RF_cmdPropRxBeacon.status == PROP_DONE_ENDED))
            {
                uint8_t length;
                uint32_t rxTime;

                memcpy(&length, rxBuffer, 1);
                memcpy(&beaconMessage, (uint8_t*)rxBuffer + 1, sizeof(BeaconPacket));
                memcpy(&rxTime, (uint8_t*)rxBuffer + 1 + sizeof(BeaconPacket), 4);
                PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);
                noLosBeacons = 0;

                if( (beaconMessage.Address0 == DEV_ADDR0) && (beaconMessage.Address1 == DEV_ADDR1))
                {
                    switch( beaconMessage.cmdNumber)
                    {
                        case 0:{
                        }
                        break;

                        case 1:{
                            txEnabled = beaconMessage.cmdValue;
                        }
                        break;

                        case 2:{
                            audioEnabled = beaconMessage.cmdValue;
                        }
                        break;

                        case 3:{
                            txSlotStart = 100 * beaconMessage.cmdValue;
                            rfState = IGO_RF_STATE_SEARCHING;
                        }
                        break;
                    }

                }
                noLosBeacons = 0;
            }

            //transmit data
            RF_cmdPropTxAdv.startTrigger.triggerType = TRIG_ABSTIME;
            RF_cmdPropTxAdv.startTime += (RF_convertMsToRatTicks(RADIO_FRAME_PERIOD_MS));
            RF_cmdPropTxAdv.pPkt = (uint8_t*)payloadBuffer;//dummyBuffer;
            RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTxAdv, RF_PriorityNormal, NULL, 0);

            //PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);
        }
    }

    Memory_free(NULL, dummyBuffer, sizeof(uint8_t) * SENSOR_PACKET_LENGTH );
}


static void i2sCallbackFxn(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *notification)
{
    if (notification->status == I2SCC26XX_STREAM_ERROR)
    {
        events |= IGOTX_AUDIO_ERROR_EVENT;
        Semaphore_post(AEncodeSem);
    }
    else if (notification->status == I2SCC26XX_STREAM_BUFFER_READY)
    {
        // Provide buffer
        events |= IGOTX_AUDIO_FRAME_EVENT;
        Semaphore_post(AEncodeSem);
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
        events |= IGOTX_AUDIO_FRAME_EVENT;
    }
}

static void processAudioFrame()
{
    int16_t idx;
    int readDataEnc;
    int readDataPack;

    I2SCC26XX_BufferRequest bufferRequest;
    I2SCC26XX_BufferRelease bufferRelease;

    bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN;
    bool gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);

    while (gotBuffer)
    {
        int16_t* pb = (int16_t*)bufferRequest.bufferIn;

        for(idx = 0; idx < streamVariables.samplesPerFrame; idx++ )
        {
            ChannelSamples[idx] = pb[2*idx+1];
        }


        readDataEnc = adpcm64_encode_run(&encoder, ChannelSamples, encodedSamples, PCM_SAMPLES_PER_FRAME);
        readDataPack = adpcm64_pack_vadim(encodedSamples, packedSamples, readDataEnc);
        memcpy( payloadBuffer+4, packedSamples, G722_P1_PAYLOAD_LENGTH );

        bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
        bufferRelease.bufferHandleOut = NULL;
        I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);

        bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN;
        gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
    }
}


static bool createAudioBuffers()
{
    // Allocate memory for decoded PCM data
    pcmSamples = Memory_alloc(NULL, sizeof(int16_t) * (2*PCM_SAMPLES_PER_FRAME * I2SCC26XX_QUEUE_SIZE), 0, NULL);
    ChannelSamples = Memory_alloc(NULL, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME, 0, NULL);
    encodedSamples = Memory_alloc(NULL, sizeof(uint16_t) * PCM_SAMPLES_PER_FRAME/2, 0, NULL);
    packedSamples = Memory_alloc(NULL, sizeof(uint8_t) * G722_P1_PAYLOAD_LENGTH, 0, NULL);
    payloadBuffer = Memory_alloc(NULL, sizeof(uint8_t) * SENSOR_PACKET_LENGTH, 0, NULL);

    payloadBuffer[0] = 0xA7;
    payloadBuffer[1] = 0x01;
    payloadBuffer[2] = 0x0;
    payloadBuffer[3] = 0x0;

    if ( (payloadBuffer == NULL) || (pcmSamples == NULL ) || (ChannelSamples == NULL) || (encodedSamples == NULL) || (packedSamples == NULL))
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
    Memory_free(NULL, packedSamples, sizeof(uint8_t) * G722_P1_PAYLOAD_LENGTH);
    Memory_free(NULL, payloadBuffer, sizeof(uint8_t) * SENSOR_PACKET_LENGTH );
}


