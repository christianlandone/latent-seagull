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
 *    ======== i2ctmp007.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <ti/drivers/PIN.h>

/* Example/Board Header files */
#include "Board.h"
#include "I2C/SensorI2C.h"
#include "I2C/CS42L55.h"

#define TASKSTACKSIZE       640

#define TMP007_DIE_TEMP     0x0001  /* Die Temp Result Register */
#define TMP007_OBJ_TEMP     0x0003  /* Object Temp Result Register */

/* Global memory storage for a PIN_Config table */
static PIN_State ledPinState;
/* Pin driver handles */
static PIN_Handle ledPinHandle;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config ledPinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_DIO15 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);
    PIN_setOutputValue(ledPinHandle, Board_DIO15, 0);

    usleep(1000);

    PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 1);
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 0);
    PIN_setOutputValue(ledPinHandle, Board_DIO15, 1);

    SensorI2C_open();

    SensorI2C_select(SENSOR_I2C_0,0x094);


    ///////////////////////////////////////////////////////////////
    uint8_t val;

    val = 0x99;
    SensorI2C_writeReg(0x00, &val, 1);

    //2. Digital Regulator.
    val = 0x30;
    SensorI2C_writeReg(0x2E, &val, 1);

    //3. ADC.
    val = 0x07;
    SensorI2C_writeReg(0x32, &val, 1);

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

    /* CS42L55 INITIALISATION */

    //M_WRI_I2C DEV_CS42L55, 0x09, 0x22 //Фильтр
    val = nHPFB | nHPFRZB | HPFA | nHPFRZA | HPFB_CF00 | HPFA_CF10;
    SensorI2C_writeReg(HPF_Control, &val, 1);

    // PGAA <- AIN1A и включаем усиление 9 db
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

    /*
    val = 0xFF;
    for (i = 0; i < 20000; i++) {
        SensorI2C_writeReg(0xA0, &val, 1);
        usleep(1000);
    }*/

    SensorI2C_close();


    return (NULL);
}
