#ifndef _SMARTRF_SETTINGS_H_
#define _SMARTRF_SETTINGS_H_

//*********************************************************************************
// Generated by SmartRF Studio version 2.6.1 (build #20)
// Tested for SimpleLink SDK version: CC13x0 SDK 1.30.xx.xx
// Device: CC1350 Rev. 2.1
// 
//*********************************************************************************

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)
#include <ti/drivers/rf/RF.h>



// TI-RTOS RF Mode Object
extern RF_Mode RF_prop;


// RF Core API commands
extern rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup;
extern rfc_CMD_FS_t RF_cmdFs;
extern rfc_CMD_PROP_TX_t RF_cmdPropTx;
extern rfc_CMD_PROP_RX_t RF_cmdPropRx;
extern rfc_CMD_PROP_TX_ADV_t RF_cmdPropTxAdv;
extern rfc_CMD_PROP_RX_ADV_t RF_cmdPropRxAdv;

extern rfc_CMD_PROP_TX_t RF_cmdPropTxBeacon;
extern rfc_CMD_PROP_RX_t RF_cmdPropRxBeacon;

#endif // _SMARTRF_SETTINGS_H_

