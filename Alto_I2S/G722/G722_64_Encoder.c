#include <stdio.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>

#include "G722_64_Encoder.h"

#define x enc->x
#define slow	enc->slow
#define detlow	enc->detlow
#define shigh	enc->shigh
#define dethigh	enc->dethigh
#define nbl	enc->nbl
#define nbh	enc->nbh

__inline void block4_init(BLOCK4* data);
__inline int block4_run (BLOCK4* data, int dl);


void adpcm64_encode_init(G722ENCODER* enc)
{
	int i;
	block4_init(&(enc->block4_low));
	block4_init(&(enc->block4_high));

	for(i = 0; i < 24; i++)
        x[i] = 0;

	slow = 0 ;
	detlow  = 32 ;
	shigh = 0 ;
	dethigh = 8 ;
	nbl = 0 ;
	nbh = 0 ;
}

int adpcm64_encode_run (G722ENCODER* enc, short* datain,unsigned short* dataout,int samples)
{
	int *datain_x = NULL;
    int j=0;                /* counter                        */
   	int ilow=0,ihigh=0;
	int xlow, xhigh;           /* low and high band pcm from qmf */
/*************************** trn_qmf *********************************/
	int sumeven, sumodd;       /* even and odd tap accumulators  */
	int *xlow_array = NULL; 
	int *xhigh_array = NULL; 
/*************************** encoder *********************************/
	int dlowt;
	int dhigh;
/*************************** block1l *********************************/
	int  el, mil, wd, wd1 ;
	int q6[32] = {0, 35, 72, 110, 150, 190, 233, 276, 323,
		370, 422, 473, 530, 587, 650, 714, 786,
		858, 940, 1023, 1121, 1219, 1339, 1458,
		1612, 1765, 1980, 2195, 2557, 2919, 0x7fff, 0x7fff} ;
	int iln[32] = {0, 63, 62, 31, 30, 29, 28, 27, 26, 25,
		24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14,
		13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 0 } ;
	int ilp[32] = {0, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52,
		51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41,
		40, 39, 38, 37, 36, 35, 34, 33, 32, 0 } ;
/*************************** BLOCK 2L ********************************/
	int ril, wd2 ;
    int qm6[64] =
		{-136,   -136,   -136,   -136,
         -24808, -21904, -19008, -16704,
         -14984,-13512, -12280, -11192,
         -10232,-9360,  -8576,  -7856,
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
/*************************** BLOCK 3L ********************************/
	int  il4, wd3;
	int wl[8] = {-60, -30, 58, 172, 334, 538, 1198, 3042 } ;
	int wl_l[8] = {-60-100, -30-100, 58-100, 172-100, 334-100, 538-100, 1198-100, 3042-100 } ;
	int rl42[16] = {0, 7, 6, 5, 4, 3, 2, 1, 7, 6, 5, 4, 3, 2, 1, 0 } ;

	int ilb[32] = {2048, 2093, 2139, 2186, 2233, 2282, 2332,
		2383, 2435, 2489, 2543, 2599, 2656, 2714,
		2774, 2834, 2896, 2960, 3025, 3091, 3158,
		3228, 3298, 3371, 3444, 3520, 3597, 3676,
		3756, 3838, 3922, 4008 } ;
/*************************** BLOCK 1H ********************************/
	int eh, mih;
	int ihn[8] = {15, 7, 6, 5, 4, 3, 2, 1 } ;
	int ihp[8] = { 0, 14,13,12,11,10, 9, 8 } ;
	int q4[8] = {0,144,280,479,723,1039,1492,2342};
/************************** BLOCK 2H *********************************/
	int qm2[16] =
		{72*8, -3101*8,	-1765*8, -1219*8,	-858*8,  -587*8, -370*8, -190*8,
		 3101*8,	1765*8,	     1219*8,	858*8,    587*8,  370*8,  190*8, - 72*8  } ;
/************************** BLOCK 3H *********************************/
//	int wh[3] = {0, -214, 798} ;
//	int rh2[4] = {2, 1, 2, 1} ;
/*********************************************************************/
	int QMF_coeff[24] = {3,-11,-11,53,12,-156, 32,362,-210,-805,951,3876,3876,951,-805,-210,362,32,-156,12,53,-11,-11,3};
/*********************************************************************/
/*    BEGINNING OF EXECUTION, READ IN A PCM SAMPLE FROM STDIN          */
	if( (samples%2) != 0 )
	{
		return 0;
	}

	//xlow_array = (int*) malloc(sizeof(int)*(samples/2));
	//xhigh_array = (int*) malloc(sizeof(int)*(samples/2));
	//datain_x = (int*) malloc(sizeof(int)*(samples+22));

	xlow_array = Memory_alloc(NULL, sizeof(int) * (samples/2), 0, NULL);
	xhigh_array = Memory_alloc(NULL, sizeof(int) * (samples/2), 0, NULL);
	datain_x = Memory_alloc(NULL, sizeof(int) * (samples+22), 0, NULL);

	if ( (xlow_array == NULL) || (xlow_array == NULL) || (xlow_array == NULL) )
	{
		return 0;
	}

	for( j = 0; j < 22; j++ )
		datain_x[j] = x[j];
	for( j = 0; j < (samples); j++ )
		datain_x[j+22] = (int)datain[j];
	for( j = 0; j < 22; j++ )
		x[j] = datain_x[samples+j];

	for( j = 0; j < (samples/2); j++ )
	{
		/* PROCESS PCM THROUGH THE QMF FILTER  */
		int i;

		/* DISCARD EVERY OTHER QMF OUTPUT */

		sumeven = 0;	
		sumodd = 0;	
	
		for(i = 0; i<24; i+=2)
		{
			sumodd  += (datain_x[2*j+i]) * QMF_coeff[i];
			sumeven += (datain_x[2*j+i+1]) * QMF_coeff[i+1];
		}

		xlow = (sumeven + sumodd) >> 14;
		xhigh = (sumeven - sumodd) >> 14;

		if(xlow>0x3fff) xlow=0x3fff;
		else if(xlow<-16384) xlow=-16384;

		if(xhigh>0x3fff) xhigh=0x3fff;
		else if(xhigh<-16384) xhigh=-16384;

		xlow_array[j] = xlow;
		xhigh_array[j] = xhigh;
	}

	for( j = 0; j < (samples/2); j++ )
	{
		int i;
		xlow = xlow_array[j];
		xhigh = xhigh_array[j];
	
		/*************************************** BLOCK 1L, SUBTRA ************/

		el = xlow - (slow) ;


		if ( el > 32767 ) el = 32767;
		else if ( el < -32768 ) el = -32768;

		/*************************************** BLOCK 1L, QUANTL ************/

		if (el>= 0)
		{
			wd = el;
			mil = el;
		}
		else
		{
			wd = -(el+1);
			mil = -el;
		}

		if(detlow > 40)
		{
			int mil_step = 8;
			mil = 15;
			for( i = 0; i<4; i++)
			{
				wd2 = q6[mil] * (detlow) ;
				wd1 = wd2 >> 12;
				if (wd >= wd1) 
					mil += mil_step;
				else
					mil -= mil_step;

				mil_step >>= 1;
			}

			wd2 = q6[mil] * (detlow) ;
			wd1 = wd2 >> 12;
			if (wd >= wd1) 
				mil++;
		}

		if (mil > 30)
			mil = 30;

		if (el < 0)
			ilow = iln[mil];
		else
			ilow = ilp[mil];


		/************************************** BLOCK 2L, INVQAL ************/
		ril = ilow >> 2 ;

		if(detlow <= 40)
			dlowt = risil6[ilow];
		else
			dlowt = ((detlow) * qm6[ilow] ) >> 15;

		/************************************** BLOCK 3L, LOGSCL *************/
		il4 = rl42[ril];
		wd = ((nbl) * 127) >> 7 ;
		nbl = wd + wl[il4] ;

		if (nbl < 0) nbl = 0 ;
		else if (nbl > 18432) nbl = 18432 ;
		/************************************** BLOCK 3L, SCALEL *************/
		wd1 = (nbl >> 6) & 31 ;
		wd2 = nbl >> 11 ;
		if ((8 - wd2) < 0)
			wd3 = ilb[wd1] << (wd2 - 8);
		else
			wd3 = ilb[wd1] >> (8 - wd2);

		detlow  = wd3 << 2 ;

		/************************************** BLOCK 3L, DELAYA *************/
		slow = block4_run (&(enc->block4_low), dlowt) ;
		/*************************************** BLOCK 1H, SUBTRA ************/

		eh = xhigh - (shigh) ;
		if ( eh > 32767 ) eh = 32767;
		else if ( eh < -32768 ) eh = -32768;

		if (eh >= 0)
		{
			wd = eh;
			mih = eh;
		}
		else
		{
			wd = -(eh+1);
			mih = -eh;
		}
		/*************************************** BLOCK 1H, QUANTH ************/

		if(dethigh > 12)
		{
			int mih_step = 2;
			mih = 4;
			for( i = 0; i<2; i++)
			{
				wd2 = q4[mih] * (dethigh) ;
				wd1 = wd2 >> 12;
				if (wd >= wd1) 
					mih += mih_step;
				else
					mih -= mih_step;

				mih_step >>= 1;
			}

			wd2 = q4[mih] * (dethigh) ;
			wd1 = wd2 >> 12;
			if (wd < wd1) 
				mih--;
		}

		if (mih > 7) 
			mih = 7;

		if( eh < 0 )
			ihigh = ihn[mih];
		else
			ihigh = ihp[mih];

		/************************************** BLOCK 2H, INVQAH ************/
		if(dethigh <= 12)
			dhigh=risih4[ihigh];
		else
			dhigh = ((dethigh) * qm2[ihigh]) >> 15 ;
		/************************************** BLOCK 3H, LOGSCL *************/
		ril=ihigh?ihigh:0xf;
		il4 = rl42[ril] ;
		wd = ((nbh) * 127) >> 7 ;

		nbh = wd + wl_l[il4] ;

		if (nbh < 0) nbh = 0 ;
		else if (nbh > 22528) nbh = 22528 ;

		/************************************** BLOCK 3H, SCALEH *************/
		wd1 =  (nbh >> 6) & 31 ;
		wd2 = nbh >> 11 ;

		if ((10 - wd2) < 0)
			wd3 = ilb[wd1] << (wd2 - 10);
		else
			wd3 = ilb[wd1] >> (10 - wd2);

		dethigh = wd3<<2;

		/************************************** BLOCK 3L, DELAYA *************/
		shigh = block4_run (&(enc->block4_high), dhigh) ;

		dataout[j] = (ilow<<4) + ihigh;
	}

	//free(xlow_array);
	//free(xhigh_array);
	//free(datain_x);
	Memory_free(NULL, xlow_array, sizeof(int) * (samples/2));
	Memory_free(NULL, xhigh_array, sizeof(int) * (samples/2));
	Memory_free(NULL, datain_x, sizeof(int) * (samples+22));
	return samples/2;
} 

__inline void block4_init(BLOCK4* data)
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
__inline int block4_run (BLOCK4* data, int dl)
{
	int i;
	int wd1, wd2, wd3, wd4, wd5;
// *************************************** BLOCK 4L, RECONS ***********
    if ( dl > 32767 ) dl = 32767;
    else if ( dl < -32768 ) dl = -32768;

	data->dlt[0] = dl;
	data->rlt[0] = data->sl + dl ;

    if ( data->rlt[0] > 32767 ) data->rlt[0] = 32767;
    else if ( data->rlt[0] < -32768 ) data->rlt[0] = -32768;
// *************************************** BLOCK 4L, PARREC ***********
	data->plt[0] = dl + data->szl ;

    if ( data->plt[0] > 32767 ) data->plt[0] = 32767;
    else if ( data->plt[0] < -32768 ) data->plt[0] = -32768;
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

//////////
    if ( data->spl > 32767 ) data->spl = 32767;
    else if ( data->spl < -32768 ) data->spl = -32768;
//////////

// *************************************** BLOCK 4L, FILTEZ ***********
	data->szl = 0;
	for(i = 1; i < 7; i++)
		data->szl += (data->bl[i] * data->dlt[i]) >> 14 ;

// *********************************BLOCK 4L, PREDIC *******************

//////////
    if ( data->szl > 32767 ) data->szl = 32767;
    else if ( data->szl < -32768 ) data->szl = -32768;
//////////

	data->sl = data->spl + data->szl ;

//////////
    if ( data->sl > 32767 ) data->sl = 32767;
    else if ( data->sl < -32768 ) data->sl = -32768;
//////////

	return (data->sl) ;
}

int adpcm64_pack_alex (unsigned short* datain,unsigned short* dataout,int words)
{
	int i;
	if((words%8) != 0)
	{
		return 0;
	}
	for(i = 0; i < (words/8); i++)
	{
		dataout[i*5+0] = ((datain[i*8+0]<<6) & 0xffc0) | ((datain[i*8+1]>>4) & 0x003F);
		dataout[i*5+1] = ((datain[i*8+2]<<6) & 0xffc0) | ((datain[i*8+3]>>4) & 0x003F);
		dataout[i*5+2] = ((datain[i*8+4]<<6) & 0xffc0) | ((datain[i*8+5]>>4) & 0x003F);
		dataout[i*5+3] = ((datain[i*8+6]<<6) & 0xffc0) | ((datain[i*8+7]>>4) & 0x003F);
		dataout[i*5+4] = ((datain[i*8+1]<<12) & 0xF000) | ((datain[i*8+3]<<8) & 0x0F00) | ((datain[i*8+5]<<4) & 0x00F0) | (datain[i*8+7] & 0x000F);
	}
	return words/8*5;
}
int adpcm64_pack_vadim (unsigned short* datain,unsigned short* dataout,int words)
{
	int i;
	if((words%8) != 0)
	{
		return 0;
	}
	for(i = 0; i < (words/8); i++)
	{
		dataout[i*5+0] = (datain[i*8] & 0x03ff) | ((datain[i*8+1]<<10) & 0xfc00);
		dataout[i*5+1] = ((datain[i*8+1]>>6) & 0x000f) | ((datain[i*8+2]<<4) & 0x3ff0) | ((datain[i*8+3]<<14) & 0xC000);
		dataout[i*5+2] = ((datain[i*8+3]>>2) & 0x00FF) | ((datain[i*8+4]<<8) & 0xFF00);
		dataout[i*5+3] = ((datain[i*8+4]>>8) & 0x0003) | ((datain[i*8+5]<<2) & 0x0ffc) | ((datain[i*8+6]<<12) & 0xF000);
		dataout[i*5+4] = ((datain[i*8+6]>>4) & 0x003f) | ((datain[i*8+7]<<6) & 0xffc0);
	}
	return words/8*5;
}
