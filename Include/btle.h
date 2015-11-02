/*
  Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.

  The information contained herein is confidential property of Nordic Semiconductor. The use,
  copying, transfer or disclosure of such information is prohibited except by express written
  agreement with Nordic Semiconductor.
 */
/** @file

    @brief BTLE types

  @note NOTE WARNING
  This file is used to auto-generate a C# compatible file to allow a hci_coder
  DLL to be produced from the LL source code which will implement the HCI in a
  C# program.  There are several limitations placed on the format and name
  spacing used in this interface file to ensure compatiblity with the
  converter script and C#.  The converter script dependencies can be modified
  by updating this file and the btle_convert script simultaneously.  The C#
  dependencies are not flexible and must be followed.

  btle_convert dependencies:
  - All command parameter structures are prefixed with "btle_cmd_param" OR "nrf_cmd_param"
  - All event parameter structres are prefixed with "btle_ev_param" OR "nrf_ev_param"
  - There are no structs defined inside other structures
  - ALL array sizes must be defined as constants using one of the following formats:
    "#define BTLE*__SIZE", or
    "#define NRF*__SIZE"

  C# dependencies:
  - There are no arrays of structures
  - There are no 2-dimensional arrays
*/
#ifndef BTLE_H__
#define BTLE_H__

#include <stdint.h>
//#include "hci_fields.h"

// These constants are used as array sizes in BTLE/NRF commands and events
// See file comment "btle_convert dependencies"
#define BTLE_LE_EVENT_MASK__SIZE                  (8)
#define BTLE_ENCRYPTED_DATA__SIZE                 (16)
#define BTLE_RANDOM_VECTOR__SIZE                  (8)
#define BTLE_ENCRYPTION_KEY__SIZE                 (16)
#define BTLE_DEVICE_ADDRESS__SIZE                 (6)
#define BTLE_IRK__SIZE                            (16)
#define BTLE_PLAINTEXT_DATA__SIZE                 (16)
#define BTLE_CHANNEL_MAP__SIZE                    (5)
#define BTLE_ADVERTISING_DATA__SIZE               (31)
#define BTLE_SCAN_RESPONSE_DATA__SIZE             (31)
#define BTLE_PDU_PAYLOAD__SIZE                    (27)
#define BTLE_ENC_DIVERSIFIER__SIZE                (2)
#define BTLE_ENC_SKD_SIZE                         (16)
#define BTLE_ENC_IV_SIZE                          (8)

/* UNUSED */
/// @brief The maximum length (in octets) of an BTLE message.
//#define BTLE_MSG_BUFFER_MAX_SIZE    (50)
//#define BTLE_CMD_PACKET_MAX_SIZE    BTLE_MSG_BUFFER_MAX_SIZE
//#define BTLE_DATA_PACKET_MAX_SIZE   BTLE_MSG_BUFFER_MAX_SIZE
//#define BTLE_EVENT_PACKET_MAX_SIZE  (70)
//#define BTLE_EVENT_HEADER_SIZE                                (2)


#define BTLE_EV_CMD_CMPLT__SUPPORTED_COMMANDS__SIZE           (64)
#define BTLE_EV_CMPLT_PACKETS__NUMBER_OF_HANDLES__SIZE   (1)
//#define BTLE_EV_LE_ADVERTISING_REPORT__NUM_REPORTS__SIZE  (1)
#define BTLE_CMD_PARAM__LE_FEATURES__SIZE                     (8)
#define BTLE_EV_CMD_CMPLT__LE_STATES__SIZE                    (4)
#define BTLE_CMD_PARAM__EVENT_MASK__SIZE                      (8)

#define NRF_VERSION_INFO_STRING__SIZE (40)   /* Size chosen somewhat arbitrarily - large enough to hold some info but smaller than the current largest event */
                                            /* Keep in sync with hci_field.h */

//Feature set mask, offset & shift definitions
#define BTLE_LE_FEATURES__ENC_FEATURE_BIT_OFFSET                  (0x00)
#define BTLE_LE_FEATURES__ENC_FEATURE_BIT_MASK                    (0x01)
#define BTLE_LE_FEATURES__ENC_FEATURE_BIT_SHIFT                   (0x00)



/** @brief BTLE RF channel index type used by RF PHY commands (_SHALL_ be fully compatible with the type in the HCI). */
typedef uint8_t btle_rf_channel_t;

/** @brief BTLE handle type (_SHALL_ be fully compatible with the type in the HCI).


    This is the connection handle for BR */
typedef uint16_t  btle_handle_t;

/** @brief BTLE hardware code type (_SHALL_ be fully compatible with the type in the HCI). */
typedef uint8_t         btle_hardware_codes_t;

/** @brief BTLE link type (_SHALL_ be fully compatible with the type in the HCI). */
typedef enum
{
  BTLE_LINK_TYPE_SYNCHRONOUS_BUFFER_OVERFLOW = 0x00, ///< (Voice Channels).
  BTLE_LINK_TYPE_ACL_BUFFER_OVERFLOW = 0x01          ///< (Data Channels).
  /* 0x02-0xFF are reserved for future use. */
} btle_link_type_t;

/** @brief BTLE advertiser interval type (_SHALL_ be fully compatible with the type in the HCI). */
typedef uint16_t btle_adv_interval_t;

/** @brief BTLE scan interval type (_SHALL_ be fully compatible with the type in the HCI). */
typedef uint16_t btle_scan_interval_t;

/** @brief BTLE connection interval type (_SHALL_ be fully compatible with the type in the HCI). */
typedef uint16_t btle_conn_interval_t;

/** @brief BTLE connection handle type (_SHALL_ be fully compatible with the type in the HCI). */
typedef uint16_t btle_connection_handle_t;

/** @brief BTLE data length type (_SHALL_ be fully compatible with the type in the HCI). */
typedef uint16_t btle_data_length_t;

/** @brief BTLE scan window type (_SHALL_ be fully compatible with the type in the HCI). */
typedef uint16_t btle_scan_window_t;

/** @brief BTLE connection latency type (_SHALL_ be fully compatible with the type in the HCI). */
typedef uint16_t  btle_conn_latency_t;

/** @brief BTLE supervision timeout type (_SHALL_ be fully compatible with the type in the HCI). */
typedef uint16_t  btle_supervision_timeout_t;

/** @brief BTLE connection estimate type (_SHALL_ be fully compatible with the type in the HCI). */
typedef uint16_t  btle_ce_t;

/** @brief BTLE power level type (_SHALL_ be fully compatible with the type in the HCI). */
typedef uint8_t btle_tx_power_level_t;

typedef uint8_t btle_cmd_buffer_depth_t;

/** @brief PB flag in HCI data packet */
typedef enum
{
  BTLE_DATA_PACKET_PB_START              = 0x00, /**< start of a non-automatically-flushable L2CAP packet */
  BTLE_DATA_PACKET_PB_CONTINUE           = 0x01, /**< Continuing fragment packet of Higher Layer Message */
  BTLE_DATA_PACKET_PB_START_AUTO_FLUSH   = 0x02  /**< start of auto flush L2CAP packet */
                                                 /**< 0x03 reseverd */
} blte_data_pb_flag_t;

/** @brief Broadcast (BC) flag in HCI data packet */
typedef enum
{
  BTLE_DATA_PACKET_BC_P_TO_P             = 0x00  /**< Point-to-point */
                                                 /**< 0x01 reserved */
                                                 /**< 0x10 reseverd */
                                                 /**< 0x11 reseverd */
} blte_data_bc_flag_t;

/** @brief Radio output power levels. */
typedef enum
{
  NRF_OUTPUT_POWER_PLUS_4DBM   =  4,   /**< Output power set to  4dBm  */
  NRF_OUTPUT_POWER_0DBM        =  0,   /**< Output power set to  0dBm (default) */
  NRF_OUTPUT_POWER_MINUS_4DBM  = -4,   /**< Output power set to -4dBm  */
  NRF_OUTPUT_POWER_MINUS_8DBM  = -8,   /**< Output power set to -8dBm  */
  NRF_OUTPUT_POWER_MINUS_12DBM = -12,  /**< Output power set to -12dBm */
  NRF_OUTPUT_POWER_MINUS_16DBM = -16,  /**< Output power set to -16dBm */
  NRF_OUTPUT_POWER_MINUS_20DBM = -20,  /**< Output power set to -20dBm */
  NRF_OUTPUT_POWER_MINUS_30DBM = -30,  /**< Output power set to -30dBm */
  NRF_OUTPUT_POWER_MINUS_40DBM = -40   /**< Output power set to -40dBm (Note: -40dBm will not actually give -40dBm, but will instead be remapped to -30dBm) */
} nrf_output_power_t;

/** @brief BTLE flow control mode type (_SHALL_ be fully compatible with the type in the HCI). */
typedef enum
{
  BTLE_FLOW_CONTROL_OFF = 0,
  BTLE_FLOW_CONTROL_MODE1,
  BTLE_FLOW_CONTROL_MODE2,
  BTLE_FLOW_CONTROL_ON
} btle_flow_control_t;

/** @brief BTLE RF PHY test packet type (_SHALL_ be fully compatible with the type in the HCI). */
typedef enum
{
  BTLE_RF_TEST_PACKET_PRBS9  = 0,
  BTLE_RF_TEST_PACKET_0X0F   = 1,
  BTLE_RF_TEST_PACKET_0X55   = 2,
  BTLE_RF_TEST_PACKET_PRBS15 = 3,
  BTLE_RF_TEST_PACKET_0XFF   = 4,
  BTLE_RF_TEST_PACKET_0X00   = 5,
  BTLE_RF_TEST_PACKET_0XF0   = 6,
  BTLE_RF_TEST_PACKET_0XAA   = 7
} btle_rf_test_packet_type_t;

/** @brief BTLE initiator whitelist mode type (_SHALL_ be fully compatible with the type in the HCI). */
typedef enum
{
  BTLE_INITIATOR_FILTER_SKIP_WHITELIST = 0,
  BTLE_INITIATOR_FILTER_USE_WHITELIST
} btle_initiator_filter_policy_t;

typedef enum
{
  BTLE_SCAN_MODE_DISABLE = 0,
  BTLE_SCAN_MODE_ENABLE
} btle_scan_mode_t;

/** @brief BTLE scanner filter mode type (_SHALL_ be fully compatible with the type in the HCI). */
typedef enum
{
  BTLE_SCAN_DUPLICATE_FILTER_DISABLE = 0,
  BTLE_SCAN_DUPLICATE_FILTER_ENABLE
} btle_scan_filter_mode_t;

/** @brief BTLE encryption mode type (_SHALL_ be fully compatible with the type in the HCI). */
typedef enum
{
  BTLE_ENCRYPTION_OFF = 0,
  BTLE_ENCRYPTION_ON
} btle_encryption_mode_t;

/** @brief BTLE advertiser type (_SHALL_ be fully compatible with the type in the HCI). */
typedef enum
{
  BTLE_ADV_TYPE_ADV_IND = 0,
  BTLE_ADV_TYPE_DIRECT_IND,
  BTLE_ADV_TYPE_SCAN_IND,
  BTLE_ADV_TYPE_NONCONN_IND
} btle_adv_types_t;

/** @brief BTLE connection role type (_SHALL_ be fully compatible with the type in the HCI). */
typedef enum
{
  BTLE_CONNECTION_ROLE_MASTER = 0,
  BTLE_CONNECTION_ROLE_SLAVE
} btle_connection_role_t;

/** @brief BTLE advertiser enable type (_SHALL_ be fully compatible with the type in the HCI). */
typedef enum
{
  BTLE_ADV_DISABLE = 0,
  BTLE_ADV_ENABLE
} btle_adv_mode_t;

/** @brief BTLE clock accuracy type (_SHALL_ be fully compatible with the type in the HCI). */
typedef enum
{
  BTLE_CLOCK_ACCURACY_500_PPM = 0x00,
  BTLE_CLOCK_ACCURACY_250_PPM = 0x01, /**<  (default) */
  BTLE_CLOCK_ACCURACY_150_PPM = 0x02,
  BTLE_CLOCK_ACCURACY_100_PPM = 0x03,
  BTLE_CLOCK_ACCURACY_75_PPM  = 0x04,
  BTLE_CLOCK_ACCURACY_50_PPM  = 0x05,
  BTLE_CLOCK_ACCURACY_30_PPM  = 0x06,
  BTLE_CLOCK_ACCURACY_20_PPM  = 0x07,
  BTLE_CLOCK_ACCURACY_SIZE    = 0x08
} btle_clock_accuracy_t;

/** @brief BTLE address type (_SHALL_ be fully compatible with the type in the HCI). */
typedef enum
{
  BTLE_ADDR_TYPE_PUBLIC = 0x00,
  BTLE_ADDR_TYPE_RANDOM = 0x01
} btle_address_type_t;

/** @brief BTLE device discovery channel map (_SHALL_ be fully compatible with the type in the HCI).
    @note used as bit fields in ll_adv
*/
typedef enum
{
  BTLE_CHANNEL_MAP_RESERVED     = 0,
  BTLE_CHANNEL_MAP_INDEX_37     = 0x01,
  BTLE_CHANNEL_MAP_INDEX_38     = 0x02,
  BTLE_CHANNEL_MAP_INDEX_37_38  = 0x03,
  BTLE_CHANNEL_MAP_INDEX_39     = 0x04,
  BTLE_CHANNEL_MAP_INDEX_37_39  = 0x05,
  BTLE_CHANNEL_MAP_INDEX_38_39  = 0x06,
  BTLE_CHANNEL_MAP_ALL          = 0x07
} btle_dd_channel_map_t;

/** @brief BTLE advertiser filter policy type (_SHALL_ be fully compatible with the type in the HCI). 
*/
typedef enum
{
  BTLE_ADV_FILTER_ALLOW_ANY = 0,    ///< Allow Scan Request from Any, Allow Connect Request from Any (Default)
  BTLE_ADV_FILTER_ALLOW_LEVEL1,     ///< Allow Scan Request from WhiteList Only, Allow Connect Request from Any
  BTLE_ADV_FILTER_ALLOW_LEVEL2,     ///< Allow Scan Request from Any, Allow Connect Request from WhiteList Only
  BTLE_ADV_FILTER_ALLOW_WHITELISTED ///< Allow Scan Request from WhiteList Only, Allow Connect Request from WhiteList Only
} btle_adv_filter_policy_t;

/** @brief BTLE scanning types (_SHALL_ be fully compatible with the type in the HCI). */
typedef enum
{
  BTLE_SCAN_TYPE_PASSIVE = 0,
  BTLE_SCAN_TYPE_ACTIVE
} btle_scan_types_t;

/** @brief BTLE scanner filter policy type (_SHALL_ be fully compatible with the type in the HCI). */
typedef enum
{
  BTLE_SCAN_FILTER_ACCEPT_ANY = 0,
  BTLE_SCAN_FILTER_ACCEPT_WHITELISTED
} btle_scan_filter_policy_t;

/* NOTE: The HCI status code values are visible in the GAP interface as SVC parameters and
   event cause values, and should therefore be visible to the application developer.
   A mirror of the enum values below is exported in the file ble_hci.h as #define values
   (enum types are avoided in values visible to the application, as the size is compiler dependent)
   and the BTLE_ prefix is replaced by BLE_HCI_
*/
/** @brief BTLE status codes (_SHALL_ be fully compatible with the HCI status codes). 
    @todo Replace use of these codes with define constants in ble_hci.h and remove enum definition
*/
typedef enum
{
  BTLE_STATUS_CODE_SUCCESS = 0x00,
  BTLE_STATUS_CODE_UNKNOWN_BTLE_COMMAND = 0x01,
  BTLE_STATUS_CODE_UNKNOWN_CONNECTION_IDENTIFIER = 0x02 ,
/*0x03 Hardware Failure
0x04 Page Timeout
*/
  BTLE_AUTHENTICATION_FAILURE = 0x05,
  BTLE_STATUS_CODE_PIN_OR_KEY_MISSING = 0x06,
  BTLE_MEMORY_CAPACITY_EXCEEDED = 0x07,
  BTLE_CONNECTION_TIMEOUT = 0x08,
/*0x09 Connection Limit Exceeded
0x0A Synchronous Connection Limit To A Device Exceeded
0x0B ACL Connection Already Exists*/
  BTLE_STATUS_CODE_COMMAND_DISALLOWED = 0x0C,
/*0x0D Connection Rejected due to Limited Resources
0x0E Connection Rejected Due To Security Reasons
0x0F Connection Rejected due to Unacceptable BD_ADDR
0x10 Connection Accept Timeout Exceeded
0x11 Unsupported Feature or Parameter Value*/
  BTLE_STATUS_CODE_INVALID_BTLE_COMMAND_PARAMETERS = 0x12,
  BTLE_REMOTE_USER_TERMINATED_CONNECTION = 0x13,
  BTLE_REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES = 0x014,
  BTLE_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF = 0x15,
  BTLE_LOCAL_HOST_TERMINATED_CONNECTION = 0x16,
/*
0x17 Repeated Attempts
0x18 Pairing Not Allowed
0x19 Unknown LMP PDU
*/
  BTLE_UNSUPPORTED_REMOTE_FEATURE = 0x1A,
/*
0x1B SCO Offset Rejected
0x1C SCO Interval Rejected
0x1D SCO Air Mode Rejected*/
  BTLE_STATUS_CODE_INVALID_LMP_PARAMETERS = 0x1E,
  BTLE_STATUS_CODE_UNSPECIFIED_ERROR = 0x1F,
/*0x20 Unsupported LMP Parameter Value
0x21 Role Change Not Allowed
*/
  BTLE_STATUS_CODE_LMP_RESPONSE_TIMEOUT = 0x22,
/*0x23 LMP Error Transaction Collision*/
  BTLE_STATUS_CODE_LMP_PDU_NOT_ALLOWED = 0x24,
/*0x25 Encryption Mode Not Acceptable
0x26 Link Key Can Not be Changed
0x27 Requested QoS Not Supported
*/
  BTLE_INSTANT_PASSED = 0x28,
  BTLE_PAIRING_WITH_UNIT_KEY_UNSUPPORTED = 0x29,
  BTLE_DIFFERENT_TRANSACTION_COLLISION = 0x2A,
/*
0x2B Reserved
0x2C QoS Unacceptable Parameter
0x2D QoS Rejected
0x2E Channel Classification Not Supported
0x2F Insufficient Security
0x30 Parameter Out Of Mandatory Range
0x31 Reserved
0x32 Role Switch Pending
0x33 Reserved
0x34 Reserved Slot Violation
0x35 Role Switch Failed
0x36 Extended Inquiry Response Too Large
0x37 Secure Simple Pairing Not Supported By Host.
0x38 Host Busy - Pairing
0x39 Connection Rejected due to No Suitable Channel Found*/
  BTLE_CONTROLLER_BUSY  = 0x3A,
  BTLE_CONN_INTERVAL_UNACCEPTABLE = 0x3B,
  BTLE_DIRECTED_ADVERTISER_TIMEOUT = 0x3C,
  BTLE_CONN_TERMINATED_DUE_TO_MIC_FAILURE = 0x3D,
  BTLE_CONN_FAILED_TO_BE_ESTABLISHED = 0x3E,
} btle_status_codes_t;

typedef enum
{
  BTLE_EVENT_NONE = 0x00,
  BTLE_EVENT_READ_REMOTE_VERSION_INFORMATION_COMPLETE,
  BTLE_EVENT_HARDWARE_ERROR,
  BTLE_EVENT_DATA_BUFFER_OVERFLOW,
  BTLE_EVENT_LE_ADVERTISING_REPORT,
  BTLE_EVENT_LE_CONNECTION_COMPLETE,
  BTLE_EVENT_LE_READ_REMOTE_USED_FEATURES_COMPLETE,
  BTLE_EVENT_LE_CONNECTION_UPDATE_COMPLETE,
  BTLE_EVENT_LE_LONG_TERM_KEY_REQUESTED,
  BTLE_EVENT_FLUSH_OCCURRED,
  BTLE_EVENT_DISCONNECTION_COMPLETE,
  BTLE_EVENT_ENCRYPTION_CHANGE,
  BTLE_EVENT_ENCRYPTION_KEY_REFRESH_COMPLETE,
  BTLE_EVENT_NUMBER_OF_COMPLETED_PACKETS,
  BTLE_EVENT_COMMAND_COMPLETE,
  BTLE_EVENT_COMMAND_STATUS,
/* Begin - Vendor Specific Events. */
  BTLE_VS_EVENT_NRF_LL_EVENT_BUFFER_OVERFLOW,
  BTLE_VS_EVENT_NRF_LL_EVENT_WINLIM_AUTO_OFF_OCCURRED,
  BTLE_VS_EVENT_NRF_LL_EVENT_RSSI_CHANGED,
  BTLE_VS_EVENT_NRF_LL_EVENT_CONNECTION_COMPLETE,
  BTLE_VS_EVENT_NRF_LL_EVENT_SCAN_REQ_REPORT,
/* End - Vendor Specific Events. */
} btle_event_code_t;


typedef enum
{
  BTLE_CMD_NONE = 0x00,
  BTLE_CMD_UNKNOWN,
  BTLE_CMD_RESET,
  BTLE_CMD_SET_EVENT_MASK,
  BTLE_CMD_LE_SET_EVENT_MASK,
  BTLE_CMD_LE_READ_BUFFER_SIZE,
  BTLE_CMD_LE_READ_LOCAL_SUPPORTED_FEATURES,
  BTLE_CMD_LE_READ_SUPPORTED_STATES,
  BTLE_CMD_LE_WRITE_LOCAL_USED_FEATURES,
  BTLE_CMD_LE_READ_WHITE_LIST_SIZE,
  BTLE_CMD_LE_CLEAR_WHITE_LIST,
  BTLE_CMD_LE_ADD_DEVICE_TO_WHITE_LIST,
  BTLE_CMD_LE_REMOVE_DEVICE_FROM_WHITE_LIST,
  BTLE_CMD_LE_TEST_END,
  BTLE_CMD_READ_BD_ADDR,
  BTLE_CMD_READ_LOCAL_VERSION_INFORMATION,
  BTLE_CMD_READ_REMOTE_VERSION_INFORMATION,
  BTLE_CMD_READ_LOCAL_SUPPORTED_COMMANDS,
  BTLE_CMD_READ_LOCAL_SUPPORTED_FEATURES,
  BTLE_CMD_READ_RSSI,
  BTLE_CMD_HOST_BUFFER_SIZE,
  BTLE_CMD_SET_CONTROLLER_TO_HOST_FLOW_CONTROL,
  BTLE_CMD_HOST_NUMBER_OF_COMPLETED_PACKETS,
  BTLE_CMD_LE_WRITE_RANDOM_ADDRESS,
  BTLE_CMD_LE_WRITE_ADVERTISING_PARAMETERS,
  BTLE_CMD_LE_READ_ADVERTISING_CHANNEL_TX_POWER,
  BTLE_CMD_LE_WRITE_ADVERTISING_DATA,
  BTLE_CMD_LE_WRITE_ADVERTISE_ENABLE,
  BTLE_CMD_LE_TRANSMITTER_TEST,
  BTLE_CMD_LE_WRITE_SCAN_PARAMETERS,
  BTLE_CMD_LE_WRITE_SCAN_ENABLE,
  BTLE_CMD_LE_RECEIVER_TEST,
  BTLE_CMD_LE_WRITE_SCAN_RESPONSE_DATA,
  BTLE_CMD_LE_CREATE_CONNECTION,
  BTLE_CMD_LE_CREATE_CONNECTION_CANCEL,
  BTLE_CMD_LE_CONNECTION_UPDATE,
  BTLE_CMD_LE_SET_HOST_CHANNEL_CLASSIFICATION,
  BTLE_CMD_LE_READ_CHANNEL_CLASSIFICATION,
  BTLE_CMD_LE_READ_REMOTE_USED_FEATURES,
  BTLE_CMD_FLUSH,
  BTLE_CMD_DISCONNECT,
  BTLE_CMD_READ_TRANSMIT_POWER_LEVEL,
  BTLE_CMD_LE_ENCRYPT,
  BTLE_CMD_LE_RAND,
  BTLE_CMD_LE_START_ENCRYPTION,
  BTLE_CMD_LE_LONG_TERM_KEY_REQUESTED_REPLY,
  BTLE_CMD_LE_LONG_TERM_KEY_REQUESTED_NEGATIVE_REPLY,
  BTLE_CMD_READ_BUFFER_SIZE,
/* Begin - Vendor Specific Commands. */
  NRF_CMD_SET_CLOCK_PARAMETERS,
  NRF_CMD_SET_TRANSMIT_POWER_LEVEL,
  NRF_CMD_SET_BD_ADDR,
  NRF_CMD_CONFIG_WINDOW_LIMIT,
  NRF_CMD_CONFIG_ACTIVE_SIGNAL,
  NRF_CMD_CONFIG_TRANSMIT_WINDOW,  ///< @note Not available over HCI
  NRF_CMD_CONFIG_APPLICATION_LATENCY,
  NRF_CMD_GET_VERSION_INFO,
  NRF_CMD_CONFIG_RSSI,
  NRF_CMD_EVENT_SLEEP_CONFIGURE,
  NRF_CMD_CLEAR_RESOLVABLE_ADDRESSES,
  NRF_CMD_ADD_RESOLVABLE_ADDRESS,
  NRF_CMD_CONFIG_LOCAL_CONN_LATENCY,
/* End - Vendor Specific Commands. */
} btle_cmd_opcode_t;

/*
  Begin - Return parameters to be used by events.
 */
typedef struct
{
  btle_status_codes_t       status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  uint16_t                  packet_length;
  uint8_t                   packet_buffer_depth;
} btle_ev_param_le_read_buffer_size_t;

typedef struct
{
  btle_status_codes_t      status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  btle_connection_handle_t connection_handle;
  uint8_t       channel_map[BTLE_CHANNEL_MAP__SIZE];
} btle_ev_param_le_read_channel_classification_t;

typedef struct
{
  btle_status_codes_t      status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  btle_connection_handle_t connection_handle;
} btle_ev_param_le_long_term_key_requested_negative_reply_t;

typedef struct
{
  btle_status_codes_t      status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  btle_connection_handle_t connection_handle;
} btle_ev_param_le_long_term_key_requested_reply_t;

typedef struct
{
  btle_connection_handle_t  connection_handle;
  uint8_t                   long_term_key[BTLE_ENCRYPTION_KEY__SIZE];
} btle_cmd_param_le_long_term_key_requested_reply_t;

typedef struct
{
  btle_connection_handle_t connection_handle;
} btle_cmd_param_le_long_term_key_requested_negative_reply_t;

typedef struct
{
  btle_status_codes_t  status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  uint8_t   le_features[BTLE_CMD_PARAM__LE_FEATURES__SIZE];
} btle_ev_param_le_read_local_supported_features_t;

typedef struct
{
  btle_status_codes_t  status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  uint8_t                 whitelist_size;
} btle_ev_param_le_read_whitelist_size_t;

typedef struct
{
  btle_status_codes_t  status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  int8_t                  transmit_power_level;
} btle_ev_param_le_read_advertising_channel_tx_power_t;

typedef struct
{
  btle_status_codes_t  status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
} btle_ev_param_le_write_advertise_enable_t;

typedef struct
{
  btle_status_codes_t  status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  uint8_t encrypted_data[BTLE_ENCRYPTED_DATA__SIZE];
} btle_ev_param_le_encrypt_t;

typedef struct
{
  btle_status_codes_t    status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  uint8_t random_number[BTLE_RANDOM_VECTOR__SIZE];
} btle_ev_param_le_rand_t;

typedef struct
{
  btle_status_codes_t  status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  uint16_t                number_of_packets;
} btle_ev_param_le_test_end_t;

typedef struct
{
  btle_status_codes_t  status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  uint8_t   bd_addr[BTLE_DEVICE_ADDRESS__SIZE];
} btle_ev_param_read_bd_addr_t;

typedef struct
{
  btle_status_codes_t  status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  uint8_t      lmp_features[BTLE_CMD_PARAM__LE_FEATURES__SIZE];
} btle_ev_param_read_local_supported_features_t;

typedef struct
{
  btle_status_codes_t  status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  uint8_t                 supported_commands[BTLE_EV_CMD_CMPLT__SUPPORTED_COMMANDS__SIZE];
} btle_ev_param_read_local_supported_commands_t;

typedef struct
{
  btle_status_codes_t  status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  btle_handle_t        handle;
  int8_t                  rssi;
} btle_ev_param_read_rssi_t;

typedef struct
{
  btle_status_codes_t  status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  uint8_t                 hci_version;
  uint16_t                hci_revision;
  uint8_t                 lmp_version;
  uint16_t                manufacturer_name;
  uint16_t                lmp_subversion;
} btle_ev_param_read_local_version_information_t;

typedef struct
{
  btle_status_codes_t      status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  btle_connection_handle_t connection_handle;
} btle_ev_param_flush_t;

typedef struct
{
  btle_status_codes_t      status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  btle_connection_handle_t connection_handle;
  int8_t                      transmit_power_level;
} btle_ev_param_read_transmit_power_level_t;

typedef struct
{
  btle_status_codes_t  status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  uint16_t                hc_acl_data_packet_length;
  uint8_t                 hc_synchronous_data_packet_length;
  uint16_t                hc_total_num_acl_data_packets;
  uint16_t                hc_total_num_synchronous_data_packets;
} btle_ev_param_read_buffer_size_t;

typedef struct
{
  btle_status_codes_t       status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
} btle_ev_param_status_only_command_t;

typedef struct
{
  btle_status_codes_t      status;
  btle_connection_handle_t connection_handle;
  uint8_t                     lmp_version;
  uint16_t                    manufacturer_name;
  uint16_t                    lmp_subversion;
} btle_ev_param_read_remote_version_information_complete_t;

typedef struct
{
  btle_hardware_codes_t hardware_code; ///< the code for the errorus hardware.
} btle_ev_param_hardware_error_t;

typedef struct
{
  btle_link_type_t link_type; ///< the type of the link.
} btle_ev_param_data_buffer_overflow_t;

typedef struct
{
  btle_handle_t  handle; ///< the handle that was flushed.
} btle_ev_param_flush_occurred_t;


typedef enum
{
  BTLE_REPORT_TYPE_ADV_IND = 0x00,
  BTLE_REPORT_TYPE_ADV_DIRECT_IND = 0x01,
  BTLE_REPORT_TYPE_ADV_SCAN_IND = 0x02,
  BTLE_REPORT_TYPE_ADV_NONCONN_IND = 0x03,
  BTLE_REPORT_TYPE_SCAN_RSP = 0x04
  /* 0x06-0xFF    Reserved for Future Use */
} btle_report_event_type_t;

typedef struct
{
  btle_cmd_buffer_depth_t cmd_buffer_depth;
  uint16_t                hci_opcode;
  btle_cmd_opcode_t       btle_opcode;
} btle_ev_param_unknown_command_t;

typedef struct
{
  uint8_t                     num_reports;
  btle_report_event_type_t    event_type;
  btle_address_type_t         address_type;
  uint8_t                     address[BTLE_DEVICE_ADDRESS__SIZE];
  uint8_t                     length_data;
  uint8_t                     report_data[BTLE_ADVERTISING_DATA__SIZE];
  uint8_t                     rssi;
} btle_ev_param_le_advertising_report_t;

typedef struct
{
  btle_status_codes_t        status;
  btle_connection_handle_t   connection_handle;
  btle_connection_role_t     role;
  btle_address_type_t        peer_address_type;
  uint8_t                    peer_address[BTLE_DEVICE_ADDRESS__SIZE];
  btle_conn_interval_t       conn_interval;
  btle_conn_latency_t        conn_latency;
  btle_supervision_timeout_t supervision_timeout;
  btle_clock_accuracy_t      clock_accuracy;
} btle_ev_param_le_connection_complete_t;

typedef struct
{
  btle_status_codes_t        status;
  btle_connection_handle_t   connection_handle;
  btle_conn_interval_t       conn_interval;
  btle_conn_latency_t        conn_latency;
  btle_supervision_timeout_t supervision_timeout;
} btle_ev_param_le_connection_update_complete_t;

typedef struct
{
  btle_status_codes_t  status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  uint8_t                   le_states[BTLE_EV_CMD_CMPLT__LE_STATES__SIZE];
} btle_ev_param_le_read_supported_states_t;

typedef struct
{
  btle_status_codes_t      status;
  btle_connection_handle_t connection_handle;
  uint8_t                  le_features[BTLE_CMD_PARAM__LE_FEATURES__SIZE];
} btle_ev_param_le_read_remote_used_features_complete_t;

typedef struct
{
  btle_connection_handle_t  connection_handle;
  uint8_t                   random_number[BTLE_RANDOM_VECTOR__SIZE];
  uint8_t                   encryption_diversifier[BTLE_ENC_DIVERSIFIER__SIZE];
} btle_ev_param_le_long_term_key_requested_t;

typedef struct
{
  btle_status_codes_t      status;  ///< the status according to the HCI status and error codes.
  btle_connection_handle_t connection_handle; ///< the connection handle that was disconnected.
  btle_status_codes_t      reason;  ///< the reason for disconnection according to the HCI status and error codes.
} btle_ev_param_disconnection_complete_t;

typedef struct
{
  uint8_t                     number_of_handles;                                                                     ///< the number of handles in the list.
  btle_connection_handle_t    connection_handle_array[BTLE_EV_CMPLT_PACKETS__NUMBER_OF_HANDLES__SIZE];           ///< the list of handles.
  uint16_t                    hc_num_of_completed_packets_array[BTLE_EV_CMPLT_PACKETS__NUMBER_OF_HANDLES__SIZE]; ///< the list of counters for every handle.
} btle_ev_param_number_of_completed_packets_t;

typedef struct
{
  btle_status_codes_t      status;  ///< the status according to the HCI status and error codes.
  btle_connection_handle_t connection_handle; ///< the connection handle for which encryption was enabled/disabled.
  btle_encryption_mode_t   encryption_enabled; ///< zero if disabled, one if enabled.
} btle_ev_param_encryption_change_t;

typedef struct
{
  btle_status_codes_t      status;  ///< the status according to the HCI status and error codes.
  btle_connection_handle_t connection_handle; ///< the connection handle where the encryption key was refreshed.
} btle_ev_param_encryption_key_refresh_complete_t;

typedef struct
{
  btle_event_code_t related_event_code;
  btle_cmd_opcode_t related_op_code;
} nrf_ev_param_event_buffer_overflow_t;

typedef struct
{
  btle_status_codes_t  status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  uint8_t nrf_version_info_string[NRF_VERSION_INFO_STRING__SIZE];
} nrf_ev_param_version_info_t;

typedef struct
{
  btle_connection_handle_t connection_handle;
  int8_t rssi;
} nrf_ev_param_rssi_changed_t;

typedef struct
{
  btle_connection_handle_t connection_handle;
} nrf_ev_param_winlim_auto_off_occurred_t;

typedef struct
{
  btle_status_codes_t        status;
  btle_connection_handle_t   connection_handle;
  btle_connection_role_t     role;
  btle_address_type_t        peer_address_type;
  uint8_t                    peer_address[BTLE_DEVICE_ADDRESS__SIZE];
  btle_conn_interval_t       conn_interval;
  btle_conn_latency_t        conn_latency;
  btle_supervision_timeout_t supervision_timeout;
  btle_clock_accuracy_t      clock_accuracy;
  uint8_t                    irk_match :1;
  uint8_t                    irk_match_idx  :7;
} nrf_ev_param_connection_complete_t;

typedef struct
{
  btle_status_codes_t       status;
  btle_cmd_buffer_depth_t   cmd_buffer_depth;
  uint16_t                  latency;
} nrf_ev_param_config_local_conn_latency_t;

typedef struct
{
  uint8_t                   num_reports;
  uint8_t                   channel;
  btle_address_type_t       address_type;
  uint8_t                   address[BTLE_DEVICE_ADDRESS__SIZE];
  uint8_t                   rssi;
} nrf_ev_param_le_scan_req_report_t;

/*
  End - Return parameters to be used by events.
 */

/*
  Begin - Parameters passed with the commnds.
 */


typedef struct
{
  uint16_t          hci_opcode;
  btle_cmd_opcode_t btle_opcode;
} btle_cmd_param_unknown_command_t;

typedef struct
{
  nrf_output_power_t transmit_power_level;
} nrf_cmd_param_set_transmit_power_level_t;

typedef struct
{
  btle_clock_accuracy_t     sleep_clock_accuracy;
} nrf_cmd_param_set_clock_parameters_t;

typedef struct
{
  uint8_t   address[BTLE_DEVICE_ADDRESS__SIZE];
} nrf_cmd_param_set_bd_addr_t;

typedef enum
{
  NRF_ACTIVE_SIGNAL_DISABLE = 0,            ///< Active signal shall be disabled
  NRF_ACTIVE_SIGNAL_ENABLE_ACTIVE_HIGH,     ///< Active signal shall be enabled with active high polarity
  NRF_ACTIVE_SIGNAL_ENABLE_ACTIVE_LOW       ///< Active signal shall be enabled with active low polarity
} nrf_active_signal_enable_mode_t;

typedef struct
{
  nrf_active_signal_enable_mode_t enable_mode;             ///< disable OR enable with polarity [high | low]
  uint8_t                         signal_to_tick_distance; ///< distance active signal will occur before pretick; in 312.5us units; valid range [0..63] --> [0..20000us]
} nrf_cmd_param_config_active_signal_t;

typedef enum
{
  NRF_WIN_LIM_0_PPM = 0,
  NRF_WIN_LIM_10_PPM = 10,
  NRF_WIN_LIM_20_PPM = 20,
  NRF_WIN_LIM_30_PPM = 30,
  NRF_WIN_LIM_40_PPM = 40,
  NRF_WIN_LIM_50_PPM = 50,
  NRF_WIN_LIM_75_PPM = 75,
  NRF_WIN_LIM_100_PPM = 100,
  NRF_WIN_LIM_USE_SCA = 0xFF
} nrf_window_limit_t;

#define NRF_WIN_LIM_DRP_PKT_THRSH_USE_TIMEOUT (0xFF)
#define NRF_WIN_LIM_AUTO_OFF_NEVER            (0xFF)

typedef struct
{
  nrf_window_limit_t    limit;
  uint8_t               dropped_pkt_threshold;
  uint8_t               auto_off_count;
} nrf_cmd_param_config_window_limit_t;

#define NRF_TRANSMIT_WINDOW_SIZE_MAX          (8)

typedef struct
{
  uint16_t  offset;   ///< Transmit Window Offset [0ms:1.25ms:connInterval] --> [0..3200]
  uint8_t   size;     ///< Transmit Window Size [1.25ms:1.25ms:MIN(10ms, connInterval-1)] --> [1..8]
} nrf_cmd_param_config_transmit_window_t;

typedef enum
{
  NRF_APP_LATENCY_DISABLE = 0,
  NRF_APP_LATENCY_ENABLE = 1
} nrf_application_latency_mode_t;

typedef struct
{
  btle_connection_handle_t connection_handle;
  nrf_application_latency_mode_t mode;         ///< Enable or disable application latency
  uint16_t latency;                            ///< Number of slave latency events until listening for data packets
} nrf_cmd_param_config_application_latency_t;

typedef struct
{
  btle_connection_handle_t connection_handle;
  uint16_t latency;                            ///< Number of slave latency events until syncing with peer.
} nrf_cmd_param_config_local_conn_latency_t;

typedef enum
{
  NRF_RSSI_DISABLE = 0,
  NRF_RSSI_ENABLE = 1
} nrf_rssi_mode_t;

typedef enum
{
  NRF_EVENT_SLEEP_CFG_DISABLED = 0,
  NRF_EVENT_SLEEP_CFG_ENABLED = 1
} nrf_event_sleep_cfg_t;

typedef enum
{
  NRF_RSSI_EVENT_ON_CHANGE_DISABLE = 0,
  NRF_RSSI_EVENT_ON_CHANGE_ENABLE = 1
} nrf_rssi_event_on_change_mode_t;

typedef struct
{
  btle_connection_handle_t connection_handle;
  nrf_rssi_mode_t mode;                            ///< Enable or disable RSSI measurements
  nrf_rssi_event_on_change_mode_t event_on_change; ///< Enable or disable automatic events on RSSI change
} nrf_cmd_param_config_rssi_t;

typedef struct
{
  nrf_event_sleep_cfg_t event_sleep_cfg;           ///< The event sleep feature configuration.
} nrf_cmd_param_event_sleep_configure_t;

typedef struct
{
  uint8_t event_mask[BTLE_LE_EVENT_MASK__SIZE];
} btle_cmd_param_le_set_event_mask_t;

typedef struct
{
  uint8_t event_mask[BTLE_CMD_PARAM__EVENT_MASK__SIZE];
} btle_cmd_param_set_event_mask_t;

typedef struct
{
  uint8_t le_features[BTLE_CMD_PARAM__LE_FEATURES__SIZE];
} btle_cmd_param_le_write_local_used_features_t;

typedef struct
{
  btle_address_type_t  address_type;
  uint8_t   address[BTLE_DEVICE_ADDRESS__SIZE];
} btle_cmd_param_le_add_device_to_whitelist_t;

typedef struct
{
  uint8_t   irk[BTLE_IRK__SIZE];
} nrf_cmd_param_add_resolvable_address_t;

typedef struct
{
  btle_address_type_t  address_type;
  uint8_t    address[BTLE_DEVICE_ADDRESS__SIZE];
} btle_cmd_param_le_remove_device_from_whitelist_t;

typedef struct
{
  uint16_t  host_acl_data_packet_length;
  uint8_t   host_synchronous_data_packet_length;
  uint16_t  host_total_num_acl_data_packets;
  uint16_t  host_total_num_synchronous_data_packets;
} btle_cmd_param_host_buffer_size_t;

typedef struct
{
  btle_flow_control_t  flow_control_enable;
} btle_cmd_param_set_controller_to_host_flow_control_t;

typedef struct
{
  uint8_t random_address[BTLE_DEVICE_ADDRESS__SIZE];
} btle_cmd_param_le_write_random_address_t;

typedef struct
{
  btle_adv_interval_t      interval_min;
  btle_adv_interval_t      interval_max;
  btle_adv_types_t         type;
  btle_address_type_t      own_address_type;
  btle_address_type_t      direct_address_type;
  uint8_t       direct_address[BTLE_DEVICE_ADDRESS__SIZE];
  btle_dd_channel_map_t    channel_map;
  btle_adv_filter_policy_t filter_policy;
} btle_cmd_param_le_write_advertising_parameters_t;

typedef struct
{
  uint8_t data_length;
  uint8_t advertising_data[BTLE_ADVERTISING_DATA__SIZE];
} btle_cmd_param_le_write_advertising_data_t;

typedef struct
{
  btle_adv_mode_t enable;
} btle_cmd_param_le_write_advertise_enable_t;

typedef struct
{
  btle_rf_channel_t          tx_frequency;
  uint8_t                       length_of_test_data;
  btle_rf_test_packet_type_t packet_payload;
} btle_cmd_param_le_transmitter_test_t;

typedef struct
{
  btle_scan_types_t          scan_type;
  btle_scan_interval_t       scan_interval;
  btle_scan_window_t         scan_window;
  btle_address_type_t        own_address_type;
  btle_scan_filter_policy_t  scanning_filter_policy;
} btle_cmd_param_le_write_scan_parameters_t;

typedef struct
{
  btle_scan_mode_t         scan_enable;
  btle_scan_filter_mode_t  scan_filter_duplicates;
} btle_cmd_param_le_write_scan_enable_t;


typedef struct
{
  btle_rf_channel_t rx_frequency;
} btle_cmd_param_le_receiver_test_t;

typedef struct
{
  uint8_t data_length;
  uint8_t response_data[BTLE_SCAN_RESPONSE_DATA__SIZE];
} btle_cmd_param_le_write_scan_response_data_t;

typedef struct
{
  btle_scan_interval_t           le_scan_interval;
  btle_scan_window_t             le_scan_window;
  btle_initiator_filter_policy_t filter_policy;
  btle_address_type_t            peer_address_type;
  uint8_t                        peer_address[BTLE_DEVICE_ADDRESS__SIZE];
  btle_address_type_t            own_address_type;
  btle_conn_interval_t           conn_interval_min;
  btle_conn_interval_t           conn_interval_max;
  btle_conn_latency_t            conn_latency;
  btle_supervision_timeout_t     supervision_timeout;
  btle_ce_t                      minimum_ce_length;
  btle_ce_t                      maximum_ce_length;
} btle_cmd_param_le_create_connection_t;

typedef struct
{
  btle_connection_handle_t     connection_handle;
  btle_conn_interval_t         conn_interval_min;
  btle_conn_interval_t         conn_interval_max;
  btle_conn_latency_t          conn_latency;
  btle_supervision_timeout_t   supervision_timeout;
  btle_ce_t                    minimum_ce_length;
  btle_ce_t                    maximum_ce_length;
} btle_cmd_param_le_connection_update_t;

typedef struct
{
  uint8_t channel_map[BTLE_CHANNEL_MAP__SIZE];
} btle_cmd_param_le_set_host_channel_classification_t;

typedef struct
{
  btle_connection_handle_t connection_handle;
} btle_cmd_param_le_read_channel_classification_t;

typedef struct
{
  btle_handle_t  handle;
} btle_cmd_param_read_rssi_t;

typedef struct
{
  btle_connection_handle_t connection_handle;
} btle_cmd_param_le_read_remote_used_features_t;

typedef struct
{
  btle_connection_handle_t connection_handle;
} btle_cmd_param_flush_t;

typedef struct
{
  btle_connection_handle_t connection_handle;
  btle_status_codes_t       reason;
} btle_cmd_param_disconnect_t;

typedef struct
{
  btle_connection_handle_t connection_handle;
} btle_cmd_param_read_remote_version_information_t;

typedef struct
{
  btle_connection_handle_t connection_handle;
  btle_tx_power_level_t    type;
} btle_cmd_param_read_transmit_power_level_t;

typedef struct
{
  uint8_t key[BTLE_ENCRYPTION_KEY__SIZE];
  uint8_t plaintext_data[BTLE_PLAINTEXT_DATA__SIZE];
} btle_cmd_param_le_encrypt_t;

typedef struct
{
  btle_connection_handle_t  connection_handle;
  uint8_t                   random_number[BTLE_RANDOM_VECTOR__SIZE];
  uint8_t                   encrypted_diversifier[BTLE_ENC_DIVERSIFIER__SIZE];
  uint8_t                   long_term_key[BTLE_ENCRYPTION_KEY__SIZE];
} btle_cmd_param_le_start_encryption_t;

typedef struct
{
  uint8_t                   number_of_handles;
} btle_cmd_param_host_number_of_completed_packets_t;

typedef struct
{
  btle_connection_handle_t  connection_handle;
  btle_data_length_t        data_length;
  blte_data_pb_flag_t       packet_boundary_flag;
  blte_data_bc_flag_t       broadcast_flag;
  uint8_t   payload[BTLE_PDU_PAYLOAD__SIZE];
} btle_data_t;

typedef struct
{
  btle_event_code_t           event_code;
  btle_cmd_opcode_t           opcode;
  union
  {
    btle_ev_param_unknown_command_t                            unknown_command;
    btle_ev_param_status_only_command_t                        status_only_command;
    btle_ev_param_le_read_buffer_size_t                        le_read_buffer_size;
    btle_ev_param_le_read_channel_classification_t             read_channel_classification;
    btle_ev_param_le_read_local_supported_features_t           le_read_local_supported_features;
    btle_ev_param_le_read_whitelist_size_t                     le_read_whitelist_size;
    btle_ev_param_le_read_advertising_channel_tx_power_t       le_read_advertising_channel_tx_power;
    btle_ev_param_le_write_advertise_enable_t                  le_write_advertise_enable;
    btle_ev_param_le_encrypt_t                                 le_encrypt;
    btle_ev_param_le_rand_t                                    le_rand;
    btle_ev_param_le_test_end_t                                le_test_end;
    btle_ev_param_read_local_version_information_t             read_local_version_information;
    btle_ev_param_read_bd_addr_t                               read_bd_addr;
    btle_ev_param_read_local_supported_features_t              read_local_supported_features;
    //btle_ev_param_read_local_supported_commands_t              read_local_supported_commands;
    btle_ev_param_read_rssi_t                                  read_rssi;
    btle_ev_param_flush_t                                      flush;
    btle_ev_param_read_transmit_power_level_t                  read_transmit_power_level;
    btle_ev_param_read_buffer_size_t                           read_buffer_size;
    btle_ev_param_read_remote_version_information_complete_t   read_remote_version_information_complete_event;
    btle_ev_param_hardware_error_t                             hardware_error_event;
    btle_ev_param_data_buffer_overflow_t                       data_buffer_overflow_event;
    btle_ev_param_flush_occurred_t                             flush_occurred_event;
    btle_ev_param_le_advertising_report_t                      le_advertising_report_event;
    btle_ev_param_le_connection_complete_t                     le_connection_complete_event;
    btle_ev_param_le_connection_update_complete_t              le_connection_update_complete_event;
    btle_ev_param_le_read_supported_states_t                   le_read_supported_states;
    btle_ev_param_le_long_term_key_requested_negative_reply_t  le_long_term_key_requested_negative_reply;
    btle_ev_param_le_long_term_key_requested_reply_t           le_long_term_key_requested_reply;
    btle_ev_param_le_read_remote_used_features_complete_t      le_read_remote_used_features_complete_event;
    btle_ev_param_le_long_term_key_requested_t                 le_long_term_key_requested_event;
    btle_ev_param_disconnection_complete_t                     disconnection_complete_event;
    btle_ev_param_number_of_completed_packets_t                number_of_completed_packets_event;
    btle_ev_param_encryption_change_t                          encryption_change_event;
    btle_ev_param_encryption_key_refresh_complete_t            encryption_key_refresh_complete_event;
/* Begin - Vendor Specific Events. */
    nrf_ev_param_event_buffer_overflow_t                       nrf_buffer_overflow_event;
    nrf_ev_param_version_info_t                                nrf_version_info_event;
    nrf_ev_param_winlim_auto_off_occurred_t                    nrf_winlim_auto_off_occurred_event;
    nrf_ev_param_rssi_changed_t                                nrf_rssi_changed_event;
    nrf_ev_param_connection_complete_t                         nrf_connection_complete_event;
    nrf_ev_param_config_local_conn_latency_t                   nrf_config_local_conn_latency;
    nrf_ev_param_le_scan_req_report_t                          nrf_scan_req_report_event;
/* End - Vendor Specific Events. */
  } params;
} btle_event_t;


typedef struct
{
  btle_cmd_opcode_t           opcode;
  union
  {
    btle_cmd_param_unknown_command_t                            unknown_command;
    btle_cmd_param_set_event_mask_t                             set_event_mask;
    btle_cmd_param_le_set_event_mask_t                          le_set_event_mask;
    btle_cmd_param_le_write_local_used_features_t               le_write_local_used_features;
    btle_cmd_param_le_add_device_to_whitelist_t                 le_add_device_to_whitelist;
    btle_cmd_param_le_remove_device_from_whitelist_t            le_remove_device_from_whitelist;
    btle_cmd_param_host_buffer_size_t                           host_buffer_size;
    btle_cmd_param_set_controller_to_host_flow_control_t        set_controller_to_host_flow_control;
    btle_cmd_param_le_write_random_address_t                    le_write_random_address;
    btle_cmd_param_le_write_advertising_parameters_t            le_write_advertising_parameters;
    btle_cmd_param_le_write_advertising_data_t                  le_write_advertising_data;
    btle_cmd_param_le_write_advertise_enable_t                  le_write_advertise_enable;
    btle_cmd_param_le_transmitter_test_t                        le_transmitter_test;
    btle_cmd_param_le_write_scan_parameters_t                   le_write_scan_parameters;
    btle_cmd_param_le_write_scan_enable_t                       le_write_scan_enable;
    btle_cmd_param_le_receiver_test_t                           le_receiver_test;
    btle_cmd_param_le_write_scan_response_data_t                le_write_scan_response_data;
    btle_cmd_param_le_create_connection_t                       le_create_connection;
    btle_cmd_param_le_connection_update_t                       le_connection_update;
    btle_cmd_param_le_set_host_channel_classification_t         le_set_host_channel_classification;
    btle_cmd_param_read_rssi_t                                  read_rssi;
    btle_cmd_param_le_read_channel_classification_t             le_read_channel_classification;
    btle_cmd_param_le_read_remote_used_features_t               le_read_remote_used_features;
    btle_cmd_param_flush_t                                      flush;
    btle_cmd_param_disconnect_t                                 disconnect;
    btle_cmd_param_read_remote_version_information_t            read_remote_version;
    btle_cmd_param_read_transmit_power_level_t                  read_transmit_power_level;
    btle_cmd_param_le_encrypt_t                                 le_encrypt;
    btle_cmd_param_le_start_encryption_t                        le_start_encryption;
    btle_cmd_param_le_long_term_key_requested_reply_t           le_long_term_key_requested_reply;
    btle_cmd_param_le_long_term_key_requested_negative_reply_t  le_long_term_key_requested_negative_reply;
    btle_cmd_param_host_number_of_completed_packets_t           host_number_of_completed_packets;
    nrf_cmd_param_set_clock_parameters_t                        nrf_set_clock_parameters;
    nrf_cmd_param_set_transmit_power_level_t                    nrf_set_transmit_power_level;
    nrf_cmd_param_set_bd_addr_t                                 nrf_set_bd_addr;
    nrf_cmd_param_config_window_limit_t                         nrf_config_window_limit;
    nrf_cmd_param_config_active_signal_t                        nrf_config_active_signal;
    nrf_cmd_param_config_transmit_window_t                      nrf_config_transmit_window;
    nrf_cmd_param_config_application_latency_t                  nrf_config_application_latency;
    nrf_cmd_param_config_local_conn_latency_t                   nrf_config_local_conn_latency;
    nrf_cmd_param_config_rssi_t                                 nrf_config_rssi;
    nrf_cmd_param_event_sleep_configure_t                       nrf_event_sleep_configure;
    nrf_cmd_param_add_resolvable_address_t                      nrf_add_resolvable_address;
  } params;
} btle_cmd_t;

/** @brief Struct for advertiser and observer reports */
typedef struct
{
  uint32_t valid_packets;
  uint32_t invalid_packets;
  btle_event_t event;
} nrf_report_t;

#endif /*BTLE_H__*/
