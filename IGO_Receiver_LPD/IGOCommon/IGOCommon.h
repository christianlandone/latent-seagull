#ifndef __IGO_COMMON_H__
#define __IGO_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

/////////////////////////////////////////////////////////
//audio stream and encoding
/////////////////////////////////////////////////////////

#define AUDIO_SAMPLE_RATE    16000

// number of PCM samples acquired per transmission frame
#define PCM_SAMPLES_PER_FRAME    96

#define AUDIO_G722_P1            0x05

#define G722_P1_CRATIO_NUM       5
#define G722_P1_CRATIO_DEN       8

#define G722_P1_PAYLOAD_LENGTH  ((PCM_SAMPLES_PER_FRAME*G722_P1_CRATIO_NUM)/G722_P1_CRATIO_DEN)

#define SENSOR_OPS_DATA_LENGTH  4

#define SENSOR_PACKET_LENGTH (SENSOR_OPS_DATA_LENGTH + G722_P1_PAYLOAD_LENGTH)
#define SENSOR_MAX_PACKET_LENGTH 255

#define RX_COMMAND_SIZE 16

#define RADIO_FRAME_PERIOD_MS  ((PCM_SAMPLES_PER_FRAME*1000)/AUDIO_SAMPLE_RATE)

/* Convenience macro to convert milliseconds into RAT clock ticks */
#define RF_convertUsToRatTicks(microseconds) \
    (((uint32_t)(microseconds)) * 4)

/* Convenience macro to convert milliseconds into RAT clock ticks */
#define RF_convertMsToRatTicks(milliseconds) \
    (((uint32_t)(milliseconds)) * 1000 * 4)

/* Convenience macro to convert milliseconds into CPU delay cycles */
#define CPU_convertMsToDelayCycles(milliseconds) \
    (((uint32_t)(milliseconds)) * (48000 / 3))

/* Convenience macro to divide integers and always round the result up */
#define DIV_INT_ROUND_UP(divident, divisor) \
    (((divident) + ((divisor) - 1)) / (divisor))

/* An abstract data type for the message content. */
typedef struct {
    uint32_t txTime;
    uint32_t beaconInterval;
    uint8_t ledState;
} BeaconPacket;


#ifdef __cplusplus
}
#endif

#endif /* __IGO_COMMON_H__ */
