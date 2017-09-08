/** ============================================================================
 *  @file       G722_64_Decoder.h
 *
 *  @brief      Custom implementation of SB-ADPCM
 *
 *  This module provides functions for decoding SB-ADPCM data to 16 khz 16 bit LPCM
 *
 *  ============================================================================
 */

#ifndef g722_64_decoder__include
#define g722_64_decoder__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    int sl;
    int spl;
    int szl ;
    int rlt  [3];
    int al   [3];
    int plt  [3];
    int dlt  [7];
    int bl   [7];
}BLOCK4DEC;

typedef struct
{
    BLOCK4DEC block4_low;
    BLOCK4DEC block4_high;

    int x[24];
    int nbl;
    int nbh;
    int slow;
    int detlow;
    int shigh;
    int dethigh;
}G722DECODER;


extern void adpcm64_decode_init(G722DECODER* dec);
extern int adpcm64_decode_run (G722DECODER* dec, unsigned short* datain,short* dataout,int words);//not packed 10bit words

extern int adpcm64_unpack_alex (unsigned short* datain,unsigned short* dataout,int words);
extern int adpcm64_unpack_vadim (unsigned short* datain,unsigned short* dataout,int words);


#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_pdm_Codec1__include */
