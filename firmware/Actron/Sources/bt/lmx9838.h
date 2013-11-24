/*
 * Copyright (c) 2013, Maxim Osipov <maxim.osipov@gmail.com>
 */

#ifndef LMX9838_H
#define LMX9838_H

#include <stdio.h>
#include "derivative.h"


#define GAP_INQUIRY				      0x00
#define GAP_DEVICE_FOUND		      0x01
#define GAP_REMOTE_DEVICE_NAME	      0x02
#define GAP_READ_LOCAL_NAME		      0x03
#define GAP_WRITE_LOCAL_NAME	      0x04
#define GAP_READ_LOCAL_BDA		      0x05
#define GAP_SET_SCANMODE		      0x06
#define SPP_SET_PORT_CONFIG		      0x07
#define SPP_GET_PORT_CONFIG		      0x08
#define SPP_PORT_CONFIG_CHANGED	      0x09
#define SPP_ESTABLISH_LINK		      0x0A
#define SPP_LINK_ESTABLISHED	      0x0B
#define SPP_INCOMING_LINK_ESTABLISHED 0x0C
#define SPP_RELEASE_LINK		      0x0D
#define SPP_LINK_RELEASED		      0x0E
#define SPP_SEND_DATA			      0x0F
#define SPP_INCOMING_DATA		      0x10
#define SPP_TRANSPARENT_MODE	      0x11
#define SPP_CONNECT_DEFAULT_CON	      0x12
#define SPP_STORE_DEFAULT_CON	      0x13
#define SPP_GET_LIST_DEFAULT_CON	  0x14
#define SPP_DELETE_DEFAULT_CON	      0x15
#define GAP_GET_FIXED_PIN		      0x16
#define GAP_SET_FIXED_PIN		      0x17
#define GAP_GET_SECURITY_MODE	      0x18
#define GAP_SET_SECURITY_MODE	      0x19
#define RESTORE_FACTORY_SETTINGS	  0x1A
#define GAP_REMOVE_PAIRING		      0x1B
#define GAP_LIST_PAIRED_DEVICES	      0x1C
#define FORCE_MASTER_ROLE		      0x1D
#define SDAP_SERVICE_REQUEST	      0x1E
#define GET_PORTS_TO_OPEN		      0x1F
#define READ_RSSI				      0x20
#define GAP_ENTER_SNIFF_MODE	      0x21
#define SET_PORTS_TO_OPEN		      0x22
#define CHANGE_NVS_UART_SPEED	      0x23
#define SB_TEST_MODE			      0x24
#define MODULE_READY			      0x25    
#define RESET					      0x26
#define CHANGE_LOCAL_BDADDRESS	      0x27
#define STORE_CLASS_OF_DEVICE	      0x28
#define ENABLE_SDP_RECORD		      0x29
#define DELETE_SDP_RECORDS		      0x2A
#define STORE_SPP_RECORD		      0x2B    // Not available for LMX9830.. Better not to be used anyway
#define STORE_DUN_RECORD		      0x2C    // Not available for LMX9830.. Better not to be used anyway
#define STORE_FAX_RECORD		      0x2D    // Not available for LMX9830.. Better not to be used anyway
#define STORE_OPP_RECORD		      0x2E    // Not available for LMX9830.. Better not to be used anyway
#define STORE_FTP_RECORD		      0x2F    // Not available for LMX9830.. Better not to be used anyway
#define STORE_SYNC_RECORD		      0x30    // Not available for LMX9830.. Better not to be used anyway
#define STORE_SDP_RECORD		      0x31
#define SDAP_CONNECT			      0x32
#define SDAP_DISCONNECT			      0x33
#define SDAP_CONNECTION_LOST	      0x34
#define SDAP_SERVICE_BROWSE		      0x35
#define SDAP_SERVICE_SEARCH		      0x36
#define GAP_EXIT_SNIFF_MODE		      0x37
#define GAP_ENTER_PARK_MODE		      0x38
#define GAP_EXIT_PARK_MODE		      0x39
#define GAP_ENTER_HOLD_MODE		      0x3A
#define GAP_SET_LINK_POLICY		      0x3B
#define GAP_GET_LINK_POLICY		      0x3C
#define GAP_POWER_SAVE_MODE_CHANGED	  0x3D
#define SPP_PORT_STATUS_CHANGED	      0x3E
#define SDAP_ATTRIBUTE_REQUEST	      0x3F
#define SPP_GET_PORT_STATUS		      0x40
#define SPP_PORT_SET_DTR		      0x41
#define SPP_PORT_SET_RTS		      0x42
#define SPP_PORT_BREAK			      0x43
#define SPP_PORT_OVERRUN_ERROR	      0x44
#define SPP_PORT_PARITY_ERROR	      0x45
#define SPP_PORT_FRAMING_ERROR	      0x46
#define FIRMWARE_UPGRADE		      0x47    // LMX9820 specific
#define WRITE_ROM_PATCH 		      0x47    // LMX9830 specific
#define CHANGE_UART_SETTINGS	      0x48
#define READ_OPERATION_MODE		      0x49
#define WRITE_OPERATION_MODE	      0x4A
#define RF_TEST_MODE			      0x4B
#define SET_DEFAULT_LINK_POLICY	      0x4C
#define GET_DEFAULT_LINK_POLICY	      0x4D
#define SET_EVENT_FILTER		      0x4E
#define GET_EVENT_FILTER		      0x4F
#define GAP_ACL_ESTABLISHED		      0x50
#define GAP_ACL_TERMINATED		      0x51
#define DISABLE_TL				      0x52	
#define TL_ENABLED				      0x53
//unused entry 
#define SET_DEFAULT_LINK_TIMEOUT	  0x55
#define GET_DEFAULT_LINK_TIMEOUT	  0x56
#define SPP_SET_LINK_TIMEOUT	      0x57
#define SPP_GET_LINK_TIMEOUT	      0x58
#define GAP_SET_AUDIO_CONFIG	      0x59
#define GAP_GET_AUDIO_CONFIG	      0x5A
#define SET_DEFAULT_AUDIO_CONFIG	  0x5B
#define GET_DEFAULT_AUDIO_CONFIG	  0x5C
#define GAP_ESTABLISH_SCO_LINK	      0x5D
#define GAP_RELEASE_SCO_LINK	      0x5E
#define GAP_MUTE_MIC			      0x5F
#define GAP_SET_VOLUME			      0x60
#define GAP_GET_VOLUME			      0x61
#define GAP_CHANGE_SCO_PACKET	      0x62
#define SET_DEFAULT_LINK_LATENCY	  0x63
#define GET_DEFAULT_LINK_LATENCY	  0x64
#define HCI_COMMAND				      0x65    // LMX9830 specific
#define AWAIT_INITIALIZATION_EVENT    0x66    // LMX9830 specific
#define ENTER_BLUETOOTH_MODE          0x66    // LMX9830 specific
#define SET_CLOCK_FREQUENCY           0x67    // LMX9830 specific    
#define GET_CLOCK_FREQUENCY           0x68    // LMX9830 specific
#define SET_CLOCK_AND_BAUDRATE        0x69    // LMX9830 specific
//unused entry 
#define SET_GPIO_WPU                  0x6B    // LMX9830 specific
#define GET_GPIO_STATE                0x6C    // LMX9830 specific
#define SET_GPIO_DIRECTION            0x6D    // LMX9830 specific
#define SET_GPIO_OUTPUT_HIGH          0x6E    // LMX9830 specific
#define SET_GPIO_OUTPUT_LOW           0x6F    // LMX9830 specific
#define READ_MEM                      0x70    // LMX9830 specific
#define WRITE_MEM                     0x71    // LMX9830 specific
#define READ_NVS                      0x72    // LMX9830 specific
#define WRITE_NVS                     0x73    // LMX9830 specific
#define NVS_CHANGED_EVENT             0x73    // LMX9830 specific
#define SET_PCM_SLAVE_CONFIG          0x74    // LMX9830 specific
#define GAP_GET_PIN                   0x75    // LMX9830 specific


void lmx9838_init(void);
void lmx9838_task(void);
int lmx9838_connected(void);


#endif
