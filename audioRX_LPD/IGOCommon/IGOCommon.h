#ifndef __IGO_COMMON_H__
#define __IGO_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

/////////////////////////////////////////////////////////
//audio stream and encoding
/////////////////////////////////////////////////////////

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

#ifdef __cplusplus
}
#endif

#endif /* __IGO_COMMON_H__ */
