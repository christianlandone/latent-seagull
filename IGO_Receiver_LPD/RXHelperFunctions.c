/*
 * RXHelperFunctions.c
 *
 *  Created on: 28 Oct 2017
 *      Author: chris
 */

#include "RXHelperFunctions.h""

uint16_t rxChannelToFreq( uint8_t nChannel)
{
    uint16_t newFreq;

    switch(nChannel)
    {
        case 0:
        {
            newFreq = 0x035C;
        }
        break;

        case 1:
        {
            newFreq = 0x035D;
        }
        break;

        case 2:
        {
            newFreq = 0x035E;
        }
        break;

        case 3:
        {
            newFreq = 0x035F;
        }
        break;

        case 4:
        {
            newFreq = 0x0360;
        }
        break;

        case 5:
        {
            newFreq = 0x0361;
        }
        break;

        case 6:
        {
            newFreq = 0x0362;
        }
        break;

        case 7:
        {
            newFreq = 0x0363;
        }
        break;

        case 8:
        {
            newFreq = 0x0364;
        }
        break;

        case 9:
        {
            newFreq = 0x0365;
        }
        break;


        default:
        {
            newFreq = 0x35C;
        }
        break;
    }
}


