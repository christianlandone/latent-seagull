/*****************************************************************************
 *   pmu.h:  Header file for NXP LPC134x Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2008.09.01  ver 1.00    Preliminary version, first Release
 *
******************************************************************************/
#ifndef __CS42L55_H 
#define __CS42L55_H


#define DEV_CS42L55	0x94

///////////////////////////////////////////////////////////////////////
//----------- CS42L55 REG -----------------------
//===============
//PGA Vol MUX	
//===============
#define	PGAA					0x0B // REG Shannel A
#define	PGAB					0x0C // REG Shannel B
//bits	
#define	BOOSTx					0x80 // +20 db BOOST ON
#define	nBOOSTx					0x00 // +20 db BOOST OFF
#define	PGAxMUX					0x00 // Selected Input to PGAx <- AIN1x
#define	nPGAxMUX				0x40 // Selected Input to PGAx <- AIN2x
#define	PGAVOLx12db				0x18
#define	PGAVOLx09db				0x12
#define	PGAVOLx01db				0x02
#define	PGAVOLx00db				0x00

//============
//Power control 1	
//===============
#define	PowerCTL1				0x02
//bits
#define	PDN_CHRG				0x00 // ADC Charge Pump ON
#define	nPDN_CHRG				0x08 //	ADC Charge Pump OFF
#define	PDN_ADCB				0x00 //	ADC channel B ON
#define	nPDN_ADCB				0x04 //	ADC channel B OFF
#define	PDN_ADCA				0x00 //	ADC channel A ON
#define	nPDN_ADCA				0x02 //	ADC channel A OFF
#define	PDN						0x00 // Entire CODEC power UP
#define	nPDN					0x01 // Entire CODEC power DOWN

//===============
//Power control 2	
//===============
#define	PowerCTL2				0x03
//bits
#define	PDN_HPB_00				0x00 // Headphone channel B is ON when the HPDETECT pin, 29, is LO.
									 // Headphone channel B is OFF when the HPDETECT pin, 29, is HI.
#define	PDN_HPB_01				0x40 //	Headphone channel B is ON when the HPDETECT pin, 29, is HI.
									 // Headphone channel B is OFF when the HPDETECT pin, 29, is LO.
#define	PDN_HPB_10				0x80 //	Headphone channel B is always ON.
#define	PDN_HPB_11				0xC0 //	Headphone channel B is always OFF.
////
#define	PDN_HPA_00				0x00 // Headphone channel A is ON when the HPDETECT pin, 29, is LO.
									 // Headphone channel A is OFF when the HPDETECT pin, 29, is HI.
#define	PDN_HPA_01				0x10 //	Headphone channel A is ON when the HPDETECT pin, 29, is HI.
									 // Headphone channel A is OFF when the HPDETECT pin, 29, is LO.
#define	PDN_HPA_10				0x20 //	Headphone channel A is always ON.
#define	PDN_HPA_11				0x30 //	Headphone channel A is always OFF.
////
#define	PDN_LINB_00				0x00 // Line channel B is ON when the HPDETECT pin, 29, is LO.
									 // Line channel B is OFF when the HPDETECT pin, 29, is HI.
#define	PDN_LINB_01				0x04 //	Line channel B is ON when the HPDETECT pin, 29, is HI.
									 // Line channel B is OFF when the HPDETECT pin, 29, is LO.
#define	PDN_LINB_10				0x08 //	Line channel B is always ON.
#define	PDN_LINB_11				0x0C //	Line channel B is always OFF.
////
#define	PDN_LINA_00				0x00 // Line channel A is ON when the HPDETECT pin, 29, is LO.
									 // Line channel A is OFF when the HPDETECT pin, 29, is HI.
#define	PDN_LINA_01				0x01 //	Line channel A is ON when the HPDETECT pin, 29, is HI.
									 // Line channel A is OFF when the HPDETECT pin, 29, is LO.
#define	PDN_LINA_10				0x02 //	Line channel A is always ON.
#define	PDN_LINA_11				0x03 //	Line channel A is always OFF.

//===============
//Clocking control 1	
//===============
#define	CLK_CTL1				0x04
//bits
#define	M_S						0x20 // Master (Output ONLY)
#define	nM_S					0x00 //	Slave (Input ONLY)
#define	INV_SCLK				0x10 //	SCLK Inverted
#define	nINV_SCLK				0x00 //	SCLK Not Inverted
#define	SCK_MCK_00				0x00 //	Output SCLK Re-timed, bursted signal with minimal speed needed to clock the required data samples
#define	SCK_MCK_10				0x08 //	Output SCLK MCLK signal after the MCLK divide (MCLKDIV2) circuit
#define	SCK_MCK_11				0x0C // Output SCLK MCLK signal before the MCLK divide (MCLKDIV2) circuit
#define	nMCLKDIV2				0x00 // MCLK signal into CODEC No divide
#define	MCLKDIV2				0x02 // MCLK signal into CODEC Divided by 2
#define	nMCLKDIS				0x00 // MCLK signal into CODEC ON
#define	MCLKDIS					0x01 // Off; Disables the clock tree to save power when the CODEC is powered down.

//==============
//Clocking control 2	
//===============
#define	CLK_CTL2				0x05
//bits
#define	SPEED_01				0x40 // Single-Speed Mode (SSM)
#define	SPEED_10				0x10 // Half-Speed Mode (HSM)
#define	SPEED_11				0x18 // Quarter-Speed Mode (QSM)
#define	GROUP32k				0x04 // YES
#define	nGROUP32k				0x00 // NO
#define	RATIO_01				0x01 // 125
#define	RATIO_11				0x03 // 136

//===============
//ADC, Line, HP MUX
//===============
#define	ADC_LINE_HP_MUX			0x08
//bits
#define	ADCBMUX_00				0x00 // PGAB - Use
#define	ADCBMUX_01				0x40 // AIN1B; PGA is bypassed
#define	ADCBMUX_10				0x80 // AIN2B; PGA is bypassed
#define	ADCAMUX_00				0x00 // PGAA - Use
#define	ADCAMUX_01				0x10 // AIN1A; PGA is bypassed
#define	ADCAMUX_10				0x20 // AIN2A; PGA is bypassed
#define	LINEBMUX				0x00 // Selected Input to Line Amplifier Ch.B <- DACB
#define	nLINEBMUX				0x08 // Selected Input to Line Amplifier Ch.B <- PGAB
#define	LINEAMUX				0x00 // Selected Input to Line Amplifier Ch.A <- DACA
#define	nLINEAMUX				0x04 // Selected Input to Line Amplifier Ch.A <- PGAA
#define	HPBMUX					0x00 // Selected Input to HP Amplifier Ch.B <- DACB
#define	nHPBMUX					0x02 // Selected Input to HP Amplifier Ch.B <- PGAB
#define	HPAMUX					0x00 // Selected Input to HP Amplifier Ch.A <- DACA
#define	nHPAMUX					0x01 // Selected Input to HP Amplifier Ch.A <- PGAA

//===============
//Misc. ADC Control
//===============
#define	ADC_CTL					0x0A
//bits
#define	ADCB_A					0x80 // ADCBA Enabled
#define	nADCB_A					0x00 // ADCBA Disabled
#define	PGAB_A					0x40 // PGABA Enabled
#define	nPGAB_A					0x00 // PGABA Disabled
#define	DIGSUM_00				0x00 // Left Channel: ADCA		Right Channel: ADCB
#define	DIGSUM_01				0x10 // Left Channel: (ADCA + ADCB)/2		Right Channel: (ADCA + ADCB)/2
#define	DIGSUM_10				0x20 // Left Channel: (ADCA - ADCB)/2		Right Channel: (ADCA - ADCB)/2
#define	DIGSUM_11				0x30 // Left Channel: ADCB		Right Channel: ADCA
#define	nINV_ADCB				0x00 // ADC Signal Polarity Ch.B: Not Inverted 
#define	INV_ADCB				0x08 // ADC Signal Polarity Ch.B: Inverted 
#define	nINV_ADCA				0x00 // ADC Signal Polarity Ch.A: Not Inverted 
#define	INV_ADCA				0x04 // ADC Signal Polarity Ch.A: Inverted 
#define	nADCBMUTE				0x00 // Mute on ADC channel B: Not muted. 
#define	ADCBMUTE				0x02 // Mute on ADC channel B: muted. 
#define	nADCAMUTE				0x00 // Mute on ADC channel A: Not muted. 
#define	ADCAMUTE				0x01 // Mute on ADC channel A: muted. 

//===============
//Playback Control 1
//===============
#define	Playback_CTL1			0x0F
//bits
#define	PDNDSP					0x80 // DSP Status: Powered Down
#define	nPDNDSP					0x00 // DSP Status: Powered Up
#define	DEEMPH					0x40 // Enabled
#define	nDEEMPH					0x00 // Diabled
#define	PLYBCKB_A				0x10 // Enabled; Ganged channel control. Channel A volume control controls channel B volume.
#define	nPLYBCKB_A				0x00 // Disabled; Independent channel control.
#define	INV_PCMA				0x04 // PCM Signal Polarity A: Inverted
#define	nINV_PCMA				0x00 // PCM Signal Polarity A: Not Inverted
#define	INV_PCMB				0x08 // PCM Signal Polarity B: Inverted
#define	nINV_PCMB				0x00 // PCM Signal Polarity B: Not Inverted
#define	MSTBMUTE				0x02 // Master Playback Mute Ch.B: Muted
#define	nMSTBMUTE				0x00 // Master Playback Mute Ch.B: Not muted.
#define	MSTAMUTE				0x01 // Master Playback Mute Ch.B: Muted
#define	nMSTAMUTE				0x00 // Master Playback Mute Ch.B: Not muted.

//==============
//HPF Control
//==============
#define HPF_Control                             0x09
//bits
#define HPFB                                    0x80 // ADCB High-Pass Filter enabled
#define nHPFB                                   0x00 // ADCB High-Pass Filter disabled
#define HPFRZB                                  0x40 // ADCB High-Pass Filter Freeze; Frozen DC Subtraction
#define nHPFRZB                                 0x00 // ADCB High-Pass Filter Freeze; Continuous DC Subtraction
#define HPFA                                    0x20 // ADCA High-Pass Filter enabled
#define nHPFA                                   0x00 // ADCA High-Pass Filter disabled
#define HPFRZA                                  0x10 // ADCB High-Pass Filter Freeze; Frozen DC Subtraction
#define nHPFRZA                                 0x00 // ADCB High-Pass Filter Freeze; Continuous DC Subtraction

#define HPFB_CF00                               0x00 // HPFB Corner Frequency
#define HPFB_CF01                               0x04
#define HPFB_CF10                               0x08
#define HPFB_CF11                               0x0C

#define HPFA_CF00                               0x00 // HPFA Corner Frequency
#define HPFA_CF01                               0x01
#define HPFA_CF10                               0x02
#define HPFA_CF11                               0x03

extern void init_i2c_cs42l55();
extern void init_spi_cs42l55();
#endif /* end CS42L55 */
/*****************************************************************************
**                            End Of File
******************************************************************************/
