/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2009 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
**************************************************************************//*!
*
* @file usb_descriptor.h
*
* @author
*
* @version
*
* @date May-08-2009
*
* @brief The file contains USB stack class layer api header function.
*
*****************************************************************************/

#ifndef _USB_DESCRIPTOR_H
#define _USB_DESCRIPTOR_H

/******************************************************************************
* Includes
*****************************************************************************/
#include "types.h"
#include "usb_class.h"
#include "usb_framework.h"

/******************************************************************************
* Constants - None
*****************************************************************************/

/******************************************************************************
* Macro's
*****************************************************************************/
#define CDC_CC                           (0x02)
#define DATA_CLASS_SUPPORT               (TRUE)
#define CIC_NOTIF_ELEM_SUPPORT           (TRUE) /* Mandatory */
#define DIC_ISOCHRONOUS_SETTING          (FALSE)

#define IMPLEMENT_QUEUING                (TRUE)

/* Communication Class SubClass Codes */
#define DIRECT_LINE_CONTROL_MODEL           (0x01)
#define ABSTRACT_CONTROL_MODEL              (0x02)
#define TELEPHONE_CONTROL_MODEL             (0x03)
#define MULTI_CHANNEL_CONTROL_MODEL         (0x04)
#define CAPI_CONTROL_MOPDEL                 (0x05)
#define ETHERNET_NETWORKING_CONTROL_MODEL   (0x06)
#define ATM_NETWORKING_CONTROL_MODEL        (0x07)
#define WIRELESS_HANDSET_CONTROL_MODEL      (0x08)
#define DEVICE_MANAGEMENT                   (0x09)
#define MOBILE_DIRECT_LINE_MODEL            (0x0A)
#define OBEX                                (0x0B)
#define ETHERNET_EMULATION_MODEL            (0x0C)

/* Communication Class Protocol Codes */
#define NO_CLASS_SPECIFIC_PROTOCOL  (0x00)
#define AT_250_PROTOCOL             (0x01)
#define AT_PCCA_101_PROTOCOL        (0x02)
#define AT_PCCA_101_ANNEX_O         (0x03)
#define AT_GSM_7_07                 (0x04)
#define AT_3GPP_27_007              (0x05)
#define AT_TIA_CDMA                 (0x06)
#define ETHERNET_EMULATION_PROTOCOL (0x07)
#define EXTERNAL_PROTOCOL           (0xFE)
#define VENDOR_SPECIFIC             (0xFF)

/* Data Class Protocol Codes */
/* #define NO_CLASS_SPECIFIC_PROTOCOL  (0x00) */
#define PYHSICAL_INTERFACE_PROTOCOL    (0x30)
#define HDLC_PROTOCOL                  (0x31)
#define TRANSPARENT_PROTOCOL           (0x32)
#define MANAGEMENT_PROTOCOL            (0x50)
#define DATA_LINK_Q931_PROTOCOL        (0x51)
#define DATA_LINK_Q921_PROTOCOL        (0x52)
#define DATA_COMPRESSION_V42BIS        (0x90)
#define EURO_ISDN_PROTOCOL             (0x91)
#define RATE_ADAPTION_ISDN_V24         (0x92)
#define CAPI_COMMANDS                  (0x93)
#define HOST_BASED_DRIVER              (0xFD)
#define CDC_UNIT_FUNCTIONAL            (0xFE)
/* #define VENDOR_SPECIFIC             (0xFF) */

/* Descriptor SubType in Communications Class Functional Descriptors */
#define HEADER_FUNC_DESC              (0x00)
#define CALL_MANAGEMENT_FUNC_DESC     (0x01)
#define ABSTRACT_CONTROL_FUNC_DESC    (0x02)
#define DIRECT_LINE_FUNC_DESC         (0x03)
#define TELEPHONE_RINGER_FUNC_DESC    (0x04)
#define TELEPHONE_REPORT_FUNC_DESC    (0x05)
#define UNION_FUNC_DESC               (0x06)
#define COUNTRY_SELECT_FUNC_DESC      (0x07)
#define TELEPHONE_MODES_FUNC_DESC     (0x08)
#define USB_TERMINAL_FUNC_DESC        (0x09)
#define NETWORK_CHANNEL_FUNC_DESC     (0x0A)
#define PROTOCOL_UNIT_FUNC_DESC       (0x0B)
#define EXTENSION_UNIT_FUNC_DESC      (0x0C)
#define MULTI_CHANNEL_FUNC_DESC       (0x0D)
#define CAPI_CONTROL_FUNC_DESC        (0x0E)
#define ETHERNET_NETWORKING_FUNC_DESC (0x0F)
#define ATM_NETWORKING_FUNC_DESC      (0x10)
#define WIRELESS_CONTROL_FUNC_DESC    (0x11)
#define MOBILE_DIRECT_LINE_FUNC_DESC  (0x12)
#define MDLM_DETAIL_FUNC_DESC         (0x13)
#define DEVICE_MANAGEMENT_FUNC_DESC   (0x14)
#define OBEX_FUNC_DESC                (0x15)
#define COMMAND_SET_FUNC_DESC         (0x16)
#define COMMAND_SET_DETAIL_FUNC_DESC  (0x17)
#define TELEPHONE_CONTROL_FUNC_DESC   (0x18)
#define OBEX_SERVICE_ID_FUNC_DESC     (0x19)


#define CIC_SUBCLASS_CODE                ABSTRACT_CONTROL_MODEL
#define CIC_PROTOCOL_CODE                NO_CLASS_SPECIFIC_PROTOCOL
#define DIC_PROTOCOL_CODE                NO_CLASS_SPECIFIC_PROTOCOL
#define CIC_ENDP_COUNT                   (0+(CIC_NOTIF_ELEM_SUPPORT & 0x01) * 1)
#define DIC_ENDP_COUNT                   (2)

#define CIC_NOTIF_ENDPOINT               (5)
#define CIC_NOTIF_ENDP_PACKET_SIZE       (16)
#define DIC_BULK_IN_ENDPOINT             (3)
#define DIC_BULK_IN_ENDP_PACKET_SIZE     (16)/* max supported is 64 */
#define DIC_BULK_OUT_ENDPOINT            (4)
#define DIC_BULK_OUT_ENDP_PACKET_SIZE    (16)/* max supported is 64*/

#if DIC_ISOCHRONOUS_SETTING
#define DIC_ISO_IN_ENDPOINT              (3)
#define DIC_ISO_IN_ENDP_PACKET_SIZE      (64)
#define DIC_ISO_OUT_ENDPOINT             (4)
#define DIC_ISO_OUT_ENDP_PACKET_SIZE     (64)
#endif
#define REMOTE_WAKEUP_SHIFT              (5)

/* Various descriptor sizes */
#define DEVICE_DESCRIPTOR_SIZE            (18)
#define CONFIG_ONLY_DESC_SIZE             (9)
#define CDC_CONFIG_DESC_SIZE                  (CONFIG_ONLY_DESC_SIZE + 28 +   \
    (CIC_NOTIF_ELEM_SUPPORT & 0x01) \
    * 7 + DATA_CLASS_SUPPORT * 23)
#define IAD_CONFIG_DESC_SIZE              (8)

#define IFACE_ONLY_DESC_SIZE              (9)
#define ENDP_ONLY_DESC_SIZE               (7)


#if HIGH_SPEED_DEVICE
#define DEVICE_QUALIFIER_DESCRIPTOR_SIZE    (10)
#define OTHER_SPEED_CONFIG_DESCRIPTOR_SIZE  (CONFIG_DESC_SIZE)
#endif


/* Max descriptors provided by the Application */
#define USB_MAX_STD_DESCRIPTORS           (8)

/* Max configuration supported by the Application */
#define USB_MAX_CONFIG_SUPPORTED          (1)

/* Max string descriptors supported by the Application */
#define USB_MAX_STRING_DESCRIPTORS        (4)

/* Max language codes supported by the USB */
#define USB_MAX_LANGUAGES_SUPPORTED       (1)

/* string descriptors sizes */
#define USB_STR_DESC_SIZE (2)
#define USB_STR_0_SIZE    (2)
#define USB_STR_1_SIZE    (56)


#define USB_STR_2_SIZE    (30)
#define USB_STR_3_SIZE    (32)
#define USB_STR_n_SIZE    (32)

/* descriptors codes */
#define USB_DEVICE_DESCRIPTOR     (1)
#define USB_CONFIG_DESCRIPTOR     (2)
#define USB_STRING_DESCRIPTOR     (3)
#define USB_IFACE_DESCRIPTOR      (4)
#define USB_ENDPOINT_DESCRIPTOR   (5)
#define USB_DEVQUAL_DESCRIPTOR    (6)
#define USB_CS_INTERFACE          (0x24)
#define USB_CS_ENDPOINT           (0x25)

#define USB_MAX_SUPPORTED_INTERFACES     (1)
/* Implementation Specific Macros */
#define LINE_CODING_SIZE              (0x07)
#define COMM_FEATURE_DATA_SIZE        (0x02)

#define LINE_CODE_DTERATE_IFACE0      (115200) /*e.g 9600 is 0x00002580 */
#define LINE_CODE_CHARFORMAT_IFACE0   (0x00)   /* 1 stop bit */
#define LINE_CODE_PARITYTYPE_IFACE0   (0x00)   /* No Parity */
#define LINE_CODE_DATABITS_IFACE0     (0x08)   /* Data Bits Format */

#define LINE_CODE_DTERATE_IFACE1      (9600)   /*e.g. 115200 is 0x0001C200*/
#define LINE_CODE_CHARFORMAT_IFACE1   (0x00)   /* 1 stop bit */
#define LINE_CODE_PARITYTYPE_IFACE1   (0x00)   /* No Parity */
#define LINE_CODE_DATABITS_IFACE1     (0x08)   /* Data Bits Format */

#define STATUS_ABSTRACT_STATE_IFACE0  (0x0000) /* Disable Multiplexing
                                                ENDP in this interface will
                                                continue to accept/offer data*/
#define STATUS_ABSTRACT_STATE_IFACE1  (0x0000) /* Disable Multiplexing
                                                ENDP in this interface will
                                                continue to accept/offer data*/
#define COUNTRY_SETTING_IFACE0        (0x0000) /* Country Code in the format as
                                                defined in [ISO3166] */
#define COUNTRY_SETTING_IFACE1        (0x0000) /* Country Code in the format as
                                                defined in [ISO3166] */

/* Notifications Support */
#define PSTN_SUBCLASS_NOTIF_SUPPORT     (TRUE)
#define WMC_SUBCLASS_NOTIF_SUPPORT      (FALSE)
#define CDC_CLASS_NOTIF_SUPPORT         (FALSE)

#if HIGH_SPEED_DEVICE
#define USB_DEVQUAL_DESCRIPTOR      (6)
#define USB_OTHER_SPEED_DESCRIPTOR  (7)
#endif

#define DEVICE_DESC_DEVICE_CLASS             (0x00)
#define DEVICE_DESC_DEVICE_SUBCLASS          (0x00)
#define DEVICE_DESC_DEVICE_PROTOCOL          (0x00)
#define DEVICE_DESC_NUM_CONFIG_SUPPOTED      (0x01)
#define CONFIG_DESC_NUM_INTERFACES_SUPPOTED  (0x01)
#define CONFIG_DESC_CURRENT_DRAWN            (0x32)

/* mass storage specific macros */
#define MASS_STORAGE_CLASS           (0x08)
#define SCSI_TRANSPARENT_COMMAND_SET (0x06)
#define BULK_ONLY_PROTOCOL           (0x50)
#define PERIPHERAL_DEVICE_TYPE       (0x00)
/* Direct Access Block Device : Magnetic and Flash Drives*/
#define PERIPHERAL_QUALIFIER_SHIFT   (5)
#define PERIPHERAL_QUALIFIER         (0)
#define SPC_VERSION                  (4)/*SPC3 is 5; SPC2 is 4*/
#define REMOVABLE_MEDIUM_BIT         (1)
#define REMOVABLE_MEDIUM_BIT_SHIFT   (7)
#define ADDITIONAL_LENGTH            (0x20)
/*set to 0x20H if returning 36 bytes of inquiry data*/

#define SUPPORT_DISK_LOCKING_MECHANISM  (0) /*1: TRUE; 0:FALSE*/
#define IMPLEMENTING_DISK_DRIVE         (0) /*1: TRUE; 0:FALSE*/

#define CDC_DESC_ENDPOINT_COUNT   (CIC_ENDP_COUNT+(DATA_CLASS_SUPPORT & 0x01) \
    *DIC_ENDP_COUNT)

/* Msd descriptor */
#define BCD_USB_VERSION                  (0x0200)

#define REMOTE_WAKEUP_SUPPORT            (0)/*1: TRUE, 0: FALSE*/
#define REMOTE_WAKEUP_SHIFT              (5)
#define MSD_CC                           (0x08)
/* Implementation Specific Macros */
#define MSC_DESC_ENDPOINT_COUNT             (2)

#define BULK_IN_ENDPOINT                    (1)
#define BULK_IN_ENDP_PACKET_SIZE            (32)/* max supported is 64 for FS and 512 for HS*/
#define BULK_OUT_ENDPOINT                   (2)
#define BULK_OUT_ENDP_PACKET_SIZE           (32)/* max supported is 64 and 512 for HS*/

#if (BULK_OUT_ENDP_PACKET_SIZE < 32)
#error "BULK_OUT_ENDP_PACKET_SIZE cannot be less than 32"
#endif

#define MSD_CONFIG_DESC_SIZE             (CONFIG_ONLY_DESC_SIZE + \
    IFACE_ONLY_DESC_SIZE + \
    ENDP_ONLY_DESC_SIZE * MSC_DESC_ENDPOINT_COUNT)

#define IAD_DESC_SIZE                       (0x08)
#define USB_IFACE_ASSOCIATION_DESCRIPTOR    (0x0B)
#define CONFIG_DESC_SIZE                  (CDC_CONFIG_DESC_SIZE + MSD_CONFIG_DESC_SIZE +\
    IAD_CONFIG_DESC_SIZE - 9)
#define COMP_CLASS_UNIT_COUNT               (2)

#define COMPOSITE_DESC_ENDPOINT_COUNT (CDC_DESC_ENDPOINT_COUNT+MSC_DESC_ENDPOINT_COUNT)


/******************************************************************************
* Types
*****************************************************************************/
#pragma pack (push)
#pragma pack (1)

typedef const struct _USB_LANGUAGE
{
    uint_16 const language_id;
    uint_8 const ** lang_desc;
    uint_8 const * lang_desc_size;
} USB_LANGUAGE;

typedef struct _USB_ALL_LANGUAGES
{
    uint_8 const *languages_supported_string;
    uint_8 const languages_supported_size;
    USB_LANGUAGE usb_language[USB_MAX_SUPPORTED_INTERFACES];

}USB_ALL_LANGUAGES;

typedef struct _USB_ENDPOINTS
{
    /* Number of non control Endpoints */
    uint_8 count;
    /* Array of Endpoints Structures */
    USB_EP_STRUCT ep[COMPOSITE_DESC_ENDPOINT_COUNT];
}USB_ENDPOINTS,_PTR_ USB_ENDPOINTS_PTR;


typedef struct _DEV_ARCHITECTURE_STRUCT
{
    /* Class count */
    uint_8 cl_count;
    /* Endpoint count */
    uint_8 ep_count;
    /* Sub-class info */
    uint_8* value[COMP_CLASS_UNIT_COUNT];
}DEV_ARCHITECTURE_STRUCT, _PTR_ DEV_ARCHITECTURE_STRUCT_PTR;

#pragma pack (pop)
#if 1
#if defined(__CWCC__)
#pragma options align=reset
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma pack()
#endif
#endif

/******************************************************************************
* Global Functions
*****************************************************************************/
extern uint_8 USB_Desc_Get_Descriptor(
uint_8 controller_ID,
uint_8 type,
uint_8 str_num,
uint_16 index,
uint_8_ptr *descriptor,
USB_PACKET_SIZE *size);

extern uint_8 USB_Desc_Get_Interface(
uint_8 controller_ID,
uint_8 interface,
uint_8_ptr alt_interface);


extern uint_8 USB_Desc_Set_Interface(
uint_8 controller_ID,
uint_8 interface,
uint_8 alt_interface);

extern boolean USB_Desc_Valid_Configation(
uint_8 controller_ID,
uint_16 config_val);

extern boolean USB_Desc_Valid_Interface(
uint_8 controller_ID,
uint_8 interface);

extern boolean USB_Desc_Remote_Wakeup(uint_8 controller_ID);

extern void* USB_Desc_Get_Endpoints(uint_8 controller_ID);

extern uint_8 USB_Desc_Get_Line_Coding(
uint_8 controller_ID,
uint_8 interface,
uint_8_ptr *coding_data);

extern uint_8 USB_Desc_Set_Line_Coding(
uint_8 controller_ID,
uint_8 interface,
uint_8_ptr *coding_data);

extern uint_8 USB_Desc_Get_Abstract_State(
uint_8 controller_ID,
uint_8 interface,
uint_8_ptr *feature_data);
extern uint_8 USB_Desc_Get_Country_Setting(
uint_8 controller_ID,
uint_8 interface,
uint_8_ptr *feature_data);
extern uint_8 USB_Desc_Set_Abstract_State(
uint_8 controller_ID,
uint_8 interface,
uint_8_ptr *feature_data);
extern uint_8 USB_Desc_Set_Country_Setting(
uint_8 controller_ID,
uint_8 interface,
uint_8_ptr *feature_data);

extern void* USB_Desc_Get_Class_Architecture(
uint_8 controller_ID      /* [IN] Controller ID */
);

#endif

/* EOF */
