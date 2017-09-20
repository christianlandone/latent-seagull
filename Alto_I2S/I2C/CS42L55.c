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


/* Example/Board Header files */
#include "Board.h"

#include "CS42L55.h"
#include "SensorI2C.h"

////////////////////////////////////////////////////////////////////////////////////////
//  Audio codec init
////////////////////////////////////////////////////////////////////////////////////////
void initADCDevice(PIN_Handle enablePin)
{
    uint8_t devAddress = 0x4A;
    uint8_t regAddress = 0x00;
    uint8_t val = 0x16;

    PIN_setOutputValue(enablePin, Board_DIO15, 1);

    //usleep(100000);

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


