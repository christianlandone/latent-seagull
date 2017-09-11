#include <stdio.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>

#include "G722_64_Decoder.h"


#define x dec->x
#define nbl dec->nbl
#define nbh dec->nbh
#define slow dec->slow
#define detlow dec->detlow
#define shigh dec->shigh
#define dethigh dec->dethigh

//#define DEBUG_DECODER
#define PREDICTOR_LIMIT
//#define DECODER_LIMIT

__inline void block4_init(BLOCK4DEC* data);
__inline int block4_run (BLOCK4DEC* data, int dl);

static int *x_array;

void adpcm64_decode_init(G722DECODER* dec)
{
    int i;
    block4_init(&(dec->block4_low));
    block4_init(&(dec->block4_high));

    for(i = 0; i < 24; i++)
        x[i] = 0;

    nbl = 0;
    nbh = 0;
    slow = 0;
    detlow  = 32;
    shigh = 0;
    dethigh = 8;
}

int adpcm64_decode_run (G722DECODER* dec, unsigned short* datain,short* dataout,int words)
{
    int ilowr, dlowt, rlow, i, j;
    int ihigh, dhigh, rhigh ;
    int xout1,xout2;
    /****************************************************************/
    /********************************  block3l *********************/
    int wd1, wd2, wd3;
    int wl[8] = {-60, -30, 58, 172, 334, 538, 1198, 3042 }  ;
    int wl_l[8] = {-60-100, -30-100, 58-100, 172-100, 334-100, 538-100, 1198-100, 3042-100 } ;
    int rl42[16] = {0, 7, 6, 5, 4, 3, 2, 1, 7, 6, 5, 4, 3,  2, 1, 0 } ;

    int ilb[32] = {2048, 2093, 2139, 2186, 2233, 2282, 2332,
                              2383, 2435, 2489, 2543, 2599, 2656, 2714,
                              2774, 2834, 2896, 2960, 3025, 3091, 3158,
                              3228, 3298, 3371, 3444, 3520, 3597, 3676,
                              3756, 3838, 3922, 4008 } ;
    /************************************** block3h  ***************/
    //int wh[3] = {0, -214, 798} ;
    //int rh2[4] = {2, 1, 2, 1} ;
    /******************* block5l ***********************************/
    int qm6[64] =
         {0,      -136,   -136,   -136,
         -24808,  -21904, -19008, -16704,
         -14984,  -13512, -12280, -11192,
         -10232,  -9360,  -8576,  -7856,
         -7192, -6576,  -6000,  -5456,
         -4944, -4464,  -4008,  -3576,
         -3168, -2776,  -2400,  -2032,
         -1688, -1360,  -1040,  -728,
         24808, 21904,  19008,  16704,
         14984, 13512,  12280,  11192,
         10232, 9360,   8576,   7856,
         7192,  6576,   6000,   5456,
         4944,  4464,   4008,   3576,
         3168,  2776,   2400,   2032,
         1688,  1360,   1040,   728,
         432,   136,    -432,   -136 } ;

    int risil6[64] = {0, -1, -1, -1, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19,
        -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7,-6, -5, -4, -3 ,
         30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14,
                            13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3 ,2, 1, -2, -1} ;

    int risih4[16] = { 0,-7,-6,-5,-4,-3,-2,-1,7,6,5,4,3,2,1,0} ;
    /************************** BLOCK 2H *********************************/
    int qm2[16] =
        {72*8, -3101*8, -1765*8, -1219*8,   -858*8,  -587*8, -370*8, -190*8,
         3101*8,    1765*8,      1219*8,    858*8,    587*8,  370*8,  190*8, - 72*8     } ;
    /********************** block 2l *******************************/
    //int qm4[16] = {0, -20456, -12896, -8968, -6288, -4240,  -2584,  -1200, 20456,12896, 8968, 6288, 4240, 2584, 1200, 0};
/*********************************************************************/
    int QMF_coeff[24] = {-11,3,53,-11,-156,12,362, 32,-805,-210,3876,951,951,3876,-210,-805,32,362,12,-156,-11,53,3,-11};
/*********************************************************************/
    /*
    int *x_array = (int*)malloc((sizeof(int)*(2*words+22)));
    if (x_array == NULL)
    {
        return 0;
    }*/
    x_array = Memory_alloc(NULL, (sizeof(int)*(2*words+22)), 0, NULL);

    /***************************************************************/
    for(i = 0; i < 22; i++)
        x_array[i] = x[i+1];

    /* input low & high band adpcm */

    for(j=0; j < words; j++)
    {
        ilowr = (datain[j] >> 4) & 63;
        ihigh = datain[j] & 0xf;
        /*********************** BLOCK 5L, LOW BAND INVQBL ****************/

        if(detlow<=40)
            wd2=risil6[ilowr];
        else
        {
            wd2 = qm6[ilowr] ;
            wd2 = (detlow * wd2 ) >> 15 ;
        }

        dlowt=wd2;

        /******************************** BLOCK 5L, RECONS ****************/
        rlow = slow + wd2 ;

        /******************************** BLOCK 6L, LIMIT ******************/
        if (rlow > 16383 ) rlow = 16383;
        else if (rlow < -16384 ) rlow = -16384;

        /************************************** BLOCK 2L, INVQAL ************/
        wd1 = ilowr >> 2 ;
        /************************************** BLOCK 3L, LOGSCL *************/
        wd2 = rl42[wd1];
        wd1 = (nbl * 127) >> 7;
        nbl = wd1 + wl[wd2];

        if (nbl < 0)  nbl = 0;
        else if (nbl > 18432) nbl = 18432;
        /************************************** BLOCK 3L, SCALEL *************/
        wd1 =  (nbl >> 6) & 31 ;
        wd2 = nbl >> 11 ;
        wd3= ((8 - wd2) < 0)? ilb[wd1] << (wd2 - 8) : ilb[wd1] >> (8 - wd2) ;
        detlow = wd3 << 2 ;

        slow = block4_run( &(dec->block4_low) ,dlowt) ;
        /************************************** BLOCK 2H, INVQAH ************/
        if(dethigh<=12)
            dhigh=risih4[ihigh];
        else
        {
            wd2 = qm2[ihigh] ;
            dhigh = (dethigh * wd2) >> 15 ;
        }

        rhigh  = dhigh + shigh ;

        if (rhigh >  16383 )  rhigh =  16383 ;
        else if (rhigh < -16384 )  rhigh = -16384 ;

        /************************************** BLOCK 2H, LOGSCL ************/
        wd1=ihigh?ihigh:0xf;
        wd2 = rl42[wd1] ;
        wd1 = (nbh * 127) >> 7 ;

        if(dethigh<=12) nbh = wd1+ wl_l[wd2] ;
        else            nbh = wd1 + wl_l[wd2] ;

        if (nbh < 0)    nbh = 0;
        else if (nbh > 22528)   nbh = 22528;

    /************************************** BLOCK 3H, SCALEH *************/
        wd1 =  (nbh >> 6) & 31 ;
        wd2 = nbh >> 11 ;
        wd3= ((10 - wd2) < 0)? ilb[wd1] << (wd2 - 10) : ilb[wd1] >> (10 - wd2) ;
        dethigh = wd3<<2 ;

        shigh = block4_run(&(dec->block4_high) ,dhigh) ;

        wd1 = rlow - rhigh;

    #ifdef DECODER_LIMIT
        if (wd1 > 16383) wd1 = 16383;
        if (wd1 < -16384) wd1 = -16384;
    #endif

    #ifdef  DEBUG_DECODER
        if (wd1 >  16383 )  printf("wd1 =  16383 \n");
        if (wd1 < -16384 )  printf("wd1 = -16384 \n");
    #endif

        wd2 = rlow + rhigh ;

    #ifdef DECODER_LIMIT
        if (wd2 > 16383) wd2 = 16383;
        if (wd2 < -16384) wd2 = -16384;
    #endif

    #ifdef  DEBUG_DECODER
        if (wd2 >  16383 )  printf("wd2 =  16383 \n");
        if (wd2 < -16384 )  printf("wd2 = -16384 \n");
    #endif

        x_array[22+2*j] = wd1;
        x_array[22+2*j+1] = wd2;
    }

    for(j=0; j < words; j++)
    {
        /************************************* ACCUM C&D *************************/
            /* qmf tap coefficients          */
        xout1 = 0; xout2 = 0;
        for(i = 0; i < 24; i+=2)
        {
            xout1 += x_array[2*j+i] * QMF_coeff[i];
            xout2 += x_array[2*j+i+1] * QMF_coeff[i+1];
        }

        xout1>>=11;
        if (xout1 >  4*8191)  xout1 =  4*8191 ;
        else if (xout1 < -4*8192)  xout1 = -4*8192 ;

        xout2>>=11;
        if (xout2 >  4*8191)  xout2 =  4*8191 ;
        else if (xout2 < -4*8192)  xout2 = -4*8192 ;

        dataout[2*j] = (xout1);
        dataout[2*j+1] = (xout2);
    }

    for(i = 0; i < 22; i++)
        x[i+1] = x_array[2*words+i];

    Memory_free(NULL, x_array, (sizeof(int)*(2*words+22)));
    //free(x_array);

    return (words*2);
}

__inline void block4_init(BLOCK4DEC* data)
{
    int i;
    data->sl = 0 ;
    data->spl = 0 ;
    data->szl = 0 ;
    for(i = 0; i < 3; i++)
    {
        data->rlt[i] = 0;
        data->al[i] = 0;
        data->plt[i] = 0;
    }
    for(i = 0; i < 7; i++)
    {
        data->dlt[i] = 0;
        data->bl[i] = 0;
    }
}

/******************************Predictor***********************************/
__inline int block4_run (BLOCK4DEC* data, int dl)
{
    int i;
    int wd1, wd2, wd3, wd4, wd5;
// *************************************** BLOCK 4L, RECONS ***********
    data->dlt[0] = dl;
    data->rlt[0] = data->sl + dl ;
#ifdef PREDICTOR_
      if (data->rlt[0] > 32767) data->rlt[0] = 32767;
      else if (data->rlt[0] < -32768) data->rlt[0] = -32768;
#endif
// *************************************** BLOCK 4L, PARREC ***********
    data->plt[0] = dl + data->szl ;
#ifdef PREDICTOR_LIMIT
      if (data->plt[0] > 32767) data->plt[0] = 32767;
      else if (data->plt[0] < -32768) data->plt[0] = -32768;
#endif
// *****************************BLOCK 4L, UPPOL2*************************

    wd1 = data->al[1] << 2;

    if ( wd1 > 32767 ) wd1 = 32767;
    else if ( wd1 < -32768 ) wd1 = -32768;

    wd2= ( ((data->plt[0]^data->plt[1])) >= 0 )?  -wd1: wd1 ;

    if ( wd2 > 32767 ) wd2 = 32767;

    wd2 = wd2 >> 7 ;
    wd3= ( ((data->plt[0]^data->plt[2])) >= 0 )?  128: -128 ;
    wd4 = wd2 + wd3 ;


    wd5 = (data->al[2] * 32512) >> 15 ;

    wd4 = wd4 + wd5 ;

    if ( wd4  >  12288 )  wd4 =  12288 ;
    else if ( wd4  < -12288 )  wd4 = -12288 ;

    data->al[2] = wd4;
// ************************************* BLOCK 4L, UPPOL1 ***************

    wd1= ( ((data->plt[0]^data->plt[1])) >= 0 )?  192: -192 ;

    wd2 = (data->al[1] * 32640) >> 15 ;
    wd4 = wd1 + wd2 ;
    wd3 = (15360 - data->al[2]) ;

    if ( wd4 >  wd3)  wd4 =  wd3 ;
    else if ( wd4  < -wd3)  wd4 = -wd3 ;
    data->al[1] = wd4;

// *************************************** BLOCK 4L, UPZERO ************
    wd1 = ( dl == 0 ) ? 0 : 128;

    for(i = 1; i<7; i++)
    {
        wd2 = ( ((dl^data->dlt[i])) >= 0 ) ? wd1 : -wd1 ;
        wd3 = (data->bl[i] * 32640) >> 15 ;
        data->bl[i] = wd2 + wd3 ;
#ifdef PREDICTOR_LIMIT
      if (data->bl[i] > 32767) data->bl[i] = 32767;
      else if (data->bl[i] < -32768) data->bl[i] = -32768;
#endif
    }
// ********************************* BLOCK 4L, DELAYA ******************
    for(i = 5; i >= 0; i--)
        data->dlt[i+1] = data->dlt[i];

    data->rlt[2] = data->rlt[1];
    data->rlt[1] = data->rlt[0];
    data->plt[2] = data->plt[1];
    data->plt[1] = data->plt[0];

// ********************************* BLOCK 4L, FILTEP ******************

    wd1 = ( data->al[1] * data->rlt[1] ) >> 14 ;
    wd2 = ( data->al[2] * data->rlt[2] ) >> 14 ;
    data->spl = wd1 + wd2 ;
#ifdef PREDICTOR_LIMIT
      if (data->spl > 32767) data->spl = 32767;
      else if (data->spl < -32768) data->spl = -32768;
#endif
// *************************************** BLOCK 4L, FILTEZ ***********
    data->szl = 0;
    for(i = 1; i < 7; i++)
        data->szl += (data->bl[i] * data->dlt[i]) >> 14 ;
#ifdef PREDICTOR_LIMIT
      if (data->szl > 32767) data->szl = 32767;
      else if (data->szl < -32768) data->szl = -32768;
#endif
// *********************************BLOCK 4L, PREDIC *******************
    data->sl = data->spl + data->szl ;
#ifdef PREDICTOR_LIMIT
      if (data->sl > 32767) data->sl = 32767;
      else if (data->sl < -32768) data->sl = -32768;
#endif
    return (data->sl) ;
}

int adpcm64_unpack_alex (unsigned short* datain,unsigned short* dataout,int words)
{
    int i;
    if((words%5) != 0)
    {
        return 0;
    }

    for(i = 0; i < (words/5); i++)
    {
         dataout[8*i+0] = ((datain[5*i+0]>>6) & 0x03FF);
         dataout[8*i+1] = ((datain[5*i+0]<<4) & 0x03F0) | ((datain[5*i+4]>>12) & 0x000F);
         dataout[8*i+2] = ((datain[5*i+1]>>6) & 0x03FF);
         dataout[8*i+3] = ((datain[5*i+1]<<4) & 0x03F0) | ((datain[5*i+4]>>8 ) & 0x000F);
         dataout[8*i+4] = ((datain[5*i+2]>>6) & 0x03FF);
         dataout[8*i+5] = ((datain[5*i+2]<<4) & 0x03F0) | ((datain[5*i+4]>>4 ) & 0x000F);
         dataout[8*i+6] = ((datain[5*i+3]>>6) & 0x03FF);
         dataout[8*i+7] = ((datain[5*i+3]<<4) & 0x03F0) | ((datain[5*i+4]    ) & 0x000F);
    }
    return words/5*8;
}
int adpcm64_unpack_vadim (unsigned short* datain,unsigned short* dataout,int words)
{
    int i;
    if((words%5) != 0)
    {
        return 0;
    }

    for(i = 0; i < (words/5); i++)
    {
         dataout[8*i+0] = ( datain[5*i+0]      & 0x03FF)                                 ;
         dataout[8*i+1] = ((datain[5*i+0]>>10) & 0x003F) | ((datain[5*i+1]<<6) & 0x03C0);
         dataout[8*i+2] = ((datain[5*i+1]>>4 ) & 0x03FF)                                 ;
         dataout[8*i+3] = ((datain[5*i+1]>>14) & 0x0003) | ((datain[5*i+2]<<2) & 0x03FC);
         dataout[8*i+4] = ((datain[5*i+2]>>8 ) & 0x00FF) | ((datain[5*i+3]<<8) & 0x0300);
         dataout[8*i+5] = ((datain[5*i+3]>>2 ) & 0x03FF)                                 ;
         dataout[8*i+6] = ((datain[5*i+3]>>12) & 0x000F) | ((datain[5*i+4]<<4) & 0x03F0);
         dataout[8*i+7] = ((datain[5*i+4]>>6 ) & 0x03FF)                                 ;
    }
    return words/5*8;
}
