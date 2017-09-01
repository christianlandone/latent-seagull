/** ============================================================================
 *  @file       Codec1.h
 *
 *  @brief      Implementation of TI Codec Type 1 (IMA ADPCM) encoding and decoding.
 *
 *  This module provides functions for encoding and decoding data using ADPCM encoding.
 *
 *  ============================================================================
 */

#ifndef g722_64_encoder__include
#define g722_64_encoder__include

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
}BLOCK4;

typedef struct
{
    BLOCK4 block4_low;
    BLOCK4 block4_high;

    int x[24];
    int slow;
    int detlow;
    int shigh;
    int dethigh;
    int nbl;
    int nbh;

}G722ENCODER;


extern void adpcm64_encode_init(G722ENCODER* enc);
extern int adpcm64_encode_run (G722ENCODER* enc, short* datain,unsigned short* dataout,int samples);//samples - even

extern int adpcm64_pack_alex (unsigned short* datain,unsigned short* dataout,int words);
extern int adpcm64_pack_vadim (unsigned short* datain,unsigned short* dataout,int words);


#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_pdm_Codec1__include */
