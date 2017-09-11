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
#include "I2C/SensorI2C.h"
#include "I2C/CS42L55.h"
#include "I2SDrv/I2SCC26XX.h"

/* Encoder */
#include "G722/G722_64_Encoder.h"


#include <stdint.h>
#include <unistd.h>


#define TASKSTACKSIZE      1024

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];
Semaphore_Struct semStruct;
Semaphore_Handle saSem;

#define BLE_AUDIO_CMD_STOP                    0x00
#define BLE_AUDIO_CMD_START                   0x04
#define BLE_AUDIO_CMD_START_MSBC              0x05
#define BLE_AUDIO_CMD_NONE                    0xFF

#define RAS_DATA_TIC1_CMD                     0x01

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_PERIODIC_EVT                      0x0004
#define SBP_CONN_EVT_END_EVT                  0x0008
#define SBP_KEY_CHANGE_EVT                    0x0010
#define SBP_I2S_FRAME_EVENT                   0x0020
#define SBP_I2S_ERROR_EVENT                   0x0040
#define SBP_SEND_STOP_CMD_EVENT               0x0080
#define SBP_SEND_START_CMD_EVENT              0x0100
#define SBP_STOP_I2S_EVENT                    0x0200
#define SBP_START_I2S_EVENT                   0x0400
#define SBP_I2S_DMA_EVENT                     0x0620

// events flag for internal application events.
static uint16_t events = 0x0001;

/* Global memory storage for a PIN_Config table */
static PIN_State ledPinState;
/* Pin driver handles */
static PIN_Handle ledPinHandle;


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

#define BLEAUDIO_NUM_NOT_PER_FRAME_MSBC   3

#define MSBC_SAMPLES_PER_FRAME    80

typedef enum {
  STREAM_STATE_IDLE,
  STREAM_STATE_SEND_START_CMD,
  STREAM_STATE_START_I2S,
  STREAM_STATE_ACTIVE,
  STREAM_STATE_SEND_STOP_CMD,
  STREAM_STATE_STOP_I2S,
} STREAM_STATE_E;

G722ENCODER encoder;

int16_t *pcmSamples;
int16_t *uartSamples;
uint16_t *encodedSamples;
uint16_t *packedSamples;

uint8_t i2sContMgtBuffer[I2S_BLOCK_OVERHEAD_IN_BYTES * I2SCC26XX_QUEUE_SIZE] = {0};

ssize_t written = 0;
struct {
  STREAM_STATE_E streamState;
  STREAM_STATE_E requestedStreamState;
  uint8_t streamType;
  uint8_t requestedStreamType;
  uint8_t samplesPerFrame;
  uint8_t notificationsPerFrame;
  int8_t si;
  int16_t pv;
  uint8_t activeLED;
} streamVariables = {STREAM_STATE_IDLE, STREAM_STATE_IDLE, 0, 0, 0, 0, 0, 0, 0};



static I2SCC26XX_Handle i2sHandle = NULL;
static I2SCC26XX_StreamNotification i2sStream;

static void i2sCallbackFxn(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *notification);


static I2SCC26XX_Params i2sParams = {
  .requestMode            = I2SCC26XX_CALLBACK_MODE,
  .ui32requestTimeout     = BIOS_WAIT_FOREVER,
  .callbackFxn            = i2sCallbackFxn,
  .blockSize              = MSBC_SAMPLES_PER_FRAME,
  .pvContBuffer           = NULL,
  .ui32conBufTotalSize    = 0,
  .pvContMgtBuffer        = (void *) i2sContMgtBuffer,
  .ui32conMgtBufTotalSize = sizeof(i2sContMgtBuffer),
  .currentStream          = &i2sStream
};

static bool i2sStreamInProgress = false;

static void initCodecADC();
static void initPWM_MClk();

static void initI2SBus_In();
static void startI2SStream_In();



static void taskFxnI2S_In(UArg arg0, UArg arg1)
{
    int16_t idx;
    int readData;

    adpcm64_encode_init(&encoder);

    //UART stream
    UART_Handle uartHandle;
    UART_Params uartParams;

    initCodecADC();
    initPWM_MClk();

    initI2SBus_In();
    streamVariables.streamState = STREAM_STATE_START_I2S;
    streamVariables.requestedStreamState = STREAM_STATE_ACTIVE;
    startI2SStream_In();



    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 460800;

    uartHandle = UART_open(Board_UART0, &uartParams);

    while (1)
    {
        if (events == 0)
        {
            if (Semaphore_getCount(saSem) == 0) {
                System_printf("Sem blocked in task1\n");
            }
            Semaphore_pend(saSem, BIOS_WAIT_FOREVER);
        }

        /////////////////////////////////////////////////////////

        if (events & SBP_I2S_FRAME_EVENT)
        {
          events &= ~SBP_I2S_FRAME_EVENT;
          if (i2sStreamInProgress) {
            I2SCC26XX_BufferRequest bufferRequest;
            I2SCC26XX_BufferRelease bufferRelease;
            bool gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);

            while (gotBuffer)
            {
                // Flush on UART
                int16_t* pb = (int16_t*)bufferRequest.bufferIn;

                for(idx = 0; idx < streamVariables.samplesPerFrame; idx++ )
                {
                    uartSamples[idx] = pb[2*idx+1];
                }

                readData = adpcm64_encode_run(&encoder, uartSamples, encodedSamples, MSBC_SAMPLES_PER_FRAME);
                readData = adpcm64_pack_vadim(encodedSamples, packedSamples, readData);
                UART_write(uartHandle, (uint8_t*)packedSamples , readData* sizeof(int16_t));//streamVariables.samplesPerFrame * sizeof(int16_t));

4rl;weeeeezx=-g5//                UART_write(uartHandle, (uint8_t*)uartSamples , MSBC_SAMPLES_PER_FRAME* sizeof(int16_t));

                bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
                bufferRelease.bufferHandleOut = NULL;
                I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);

                gotBuffer = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
            }
          }
        }

        if (events & SBP_I2S_ERROR_EVENT)
        {
          events &= ~SBP_I2S_ERROR_EVENT;
          //PIN_setOutputValue(ledPinHandle, Board_PIN_LED0,!PIN_getOutputValue(Board_PIN_LED0));

          // Move to stop state
          /*
          hwiKey = Hwi_disable();
          streamVariables.streamState = STREAM_STATE_SEND_STOP_CMD;
          events |= SBP_SEND_STOP_CMD_EVENT;
          Hwi_restore(hwiKey);
          PIN_setOutputValue( hSbpPins, Board_DIO27_ANALOG, 1);
          */
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////
//  ======== main ========
////////////////////////////////////////////////////////////////////////////////////////

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
    Task_construct(&task0Struct, (Task_FuncPtr)taskFxnI2S_In, &taskParams, NULL);


    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if (!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);


    /* Construct a Semaphore object to be use as a resource lock, inital count 1 */
    Semaphore_Params_init(&semParams);
    Semaphore_construct(&semStruct, 1, &semParams);

    /* Obtain instance handle */
    saSem = Semaphore_handle(&semStruct);

    /* Start BIOS */
    BIOS_start();

    return (0);
}

static void i2sCallbackFxn(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *notification)
{
    if (notification->status == I2SCC26XX_STREAM_ERROR)
    {
        events |= SBP_I2S_ERROR_EVENT;
        Semaphore_post(saSem);
    }
    else if (notification->status == I2SCC26XX_STREAM_BUFFER_READY)
    {
        // Provide buffer
        events |= SBP_I2S_FRAME_EVENT;
        Semaphore_post(saSem);
        //PIN_setOutputValue( hSbpPins, Board_DIO25_ANALOG, !(PIN_getOutputValue(Board_DIO25_ANALOG)));
        //PIN_setOutputValue( hSbpPins, (streamVariables.activeLED == Board_LED0) ? Board_LED1 : Board_LED0, 0);
    }
}

////////////////////////////////////////////////////////////////////////////////////////
//  Audio codec init
////////////////////////////////////////////////////////////////////////////////////////
static void initCodecADC()
{
    uint8_t devAddress = 0x4A;
    uint8_t regAddress = 0x00;
    uint8_t val = 0x16;

    PIN_setOutputValue(ledPinHandle, Board_DIO15, 1);

    usleep(10000);

    SensorI2C_open();
    SensorI2C_select(SENSOR_I2C_0,devAddress);


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
    SensorI2C_writeReg(0x34, &val, 1);

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

    ////////////////////////////////////////////////////////////////////////
    // CS42L55 INITIALISATION

    //M_WRI_I2C DEV_CS42L55, 0x09, 0x22
    val = nHPFB | nHPFRZB | HPFA | nHPFRZA | HPFB_CF00 | HPFA_CF10;
    SensorI2C_writeReg(HPF_Control, &val, 1);

    // PGAA <- AIN1A
    //M_WRI_I2C DEV_CS42L55, PGAA, nBOOSTx | PGAxMUX | PGAVOLx00db
    val = BOOSTx | PGAxMUX | PGAVOLx00db;
    SensorI2C_writeReg(PGAA, &val, 1);

    // PGAB - Use
    // AIN1A; PGA is bypassed
    // Selected Input to Line Amplifier Ch.B <- DACB
    // Selected Input to Line Amplifier Ch.A <- DACA
    // Selected Input to HP Amplifier Ch.B <- DACB
    // Selected Input to HP Amplifier Ch.A <- DACA
    //M_WRI_I2C DEV_CS42L55, ADC_LINE_HP_MUX, ADCAMUX_01 | ADCBMUX_00 | LINEBMUX | LINEAMUX | HPBMUX | HPAMUX
    //if ( ( MODE_CONTROL & RECORD_CONFIG_GAIN_MASK ) == 0 )
        //val = ADCAMUX_01 | ADCBMUX_00 | LINEBMUX | LINEAMUX | HPBMUX | HPAMUX;
    //else
    val = ADCAMUX_01 | ADCBMUX_00 | LINEBMUX | LINEAMUX | HPBMUX | HPAMUX;
    SensorI2C_writeReg(ADC_LINE_HP_MUX, &val, 1);

    //LRCK <- 12MHz
    //M_WRI_I2C DEV_CS42L55, CLK_CTL2, RATIO_01 | nGROUP32k | SPEED_11
    val = RATIO_01 | GROUP32k | SPEED_10 ;
    SensorI2C_writeReg(CLK_CTL2, &val, 1);

    // Master (Output ONLY)
    // SCLK Inverted
    // Output SCLK Re-timed, bursted signal with minimal speed needed to clock the required data samples
    // MCLK signal into CODEC No divide
    // MCLK signal into CODEC ON
    // M_WRI_I2C DEV_CS42L55, CLK_CTL1, M_S | nINV_SCLK | SCK_MCK_00 | MCLKDIV2 | nMCLKDIS
    val =  M_S | nINV_SCLK | SCK_MCK_00 | nMCLKDIV2 | nMCLKDIS ;
    SensorI2C_writeReg(CLK_CTL1, &val, 1);

    // Left Channel: ADCB       Right Channel: ADCA
    // ADCB=A Disabled
    // PGAB=A Disabled
    // ADC Signal Polarity Ch.B: Not Inverted
    // ADC Signal Polarity Ch.A: Not Inverted
    // Mute on ADC channel B: Not muted.
    // Mute on ADC channel A: Not muted.
    //M_WRI_I2C DEV_CS42L55, ADC_CTL, DIGSUM_00 | nADCB_A | nPGAB_A | nINV_ADCB | nINV_ADCA | nADCBMUTE | nADCAMUTE
    val =  DIGSUM_00 | nADCB_A | nPGAB_A | nINV_ADCB | nINV_ADCA | nADCBMUTE | nADCAMUTE ;
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
    val = nPDN_CHRG | PDN_ADCB | PDN_ADCA | PDN;
    SensorI2C_writeReg(PowerCTL1, &val, 1);

    SensorI2C_close();
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
//  I2S Bus Initialisation
////////////////////////////////////////////////////////////////////////////////////////
static void initI2SBus_In()
{
    i2sHandle = (I2SCC26XX_Handle)&(I2SCC26XX_config);
    I2SCC26XX_init(i2sHandle);


    streamVariables.streamType = BLE_AUDIO_CMD_START_MSBC;
    streamVariables.samplesPerFrame = MSBC_SAMPLES_PER_FRAME;
    streamVariables.notificationsPerFrame = BLEAUDIO_NUM_NOT_PER_FRAME_MSBC;
    streamVariables.activeLED = Board_LED2;
    streamVariables.requestedStreamType = BLE_AUDIO_CMD_NONE;
    streamVariables.requestedStreamState = STREAM_STATE_ACTIVE;

    //initialise i2s parameters
    I2SCC26XX_Params_init( &i2sParams, I2SCC26XX_I2S, NULL );

    // Allocate memory for decoded PCM data
    pcmSamples = Memory_alloc(NULL, sizeof(int16_t) * (2*streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE), 0, NULL);
    uartSamples = Memory_alloc(NULL, sizeof(int16_t) * MSBC_SAMPLES_PER_FRAME, 0, NULL);
    encodedSamples = Memory_alloc(NULL, sizeof(uint16_t) * MSBC_SAMPLES_PER_FRAME/2, 0, NULL);
    packedSamples = Memory_alloc(NULL, sizeof(uint16_t) * 5*(MSBC_SAMPLES_PER_FRAME/8), 0, NULL);

    i2sParams.blockSize              = streamVariables.samplesPerFrame;
    i2sParams.pvContBuffer           = (void *) pcmSamples;
    i2sParams.ui32conBufTotalSize    = sizeof(int16_t) * (2*streamVariables.samplesPerFrame * I2SCC26XX_QUEUE_SIZE);
    i2sParams.i32SampleRate = -1;
    I2SCC26XX_Handle i2sHandleTmp = I2SCC26XX_open(i2sHandle, &i2sParams);
}

static void startI2SStream_In(void)
{
    if (streamVariables.streamState == STREAM_STATE_START_I2S) {
        if (streamVariables.requestedStreamState == STREAM_STATE_ACTIVE) {
            // Try to start I2S stream
            i2sStreamInProgress = I2SCC26XX_startStream(i2sHandle);

            if (i2sStreamInProgress) {
                // Move to ACTIVE as we have completed start sequence
                streamVariables.streamState = STREAM_STATE_ACTIVE;
            }
            else {
                //Display_print0(dispHandle, 5, 0, "Failed to start I2S stream");
            }
        }
        else {
            //Display_print0(dispHandle, 5, 0, "Started stream when Active was not requested");
        }
    }
}
