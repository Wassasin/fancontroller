#pragma once

#include <stdint.h>

typedef union {
    struct {
        uint8_t _pad1[2];
        uint8_t bytes[30];
    } __attribute__((packed));
    struct {
        uint8_t _pad2[2];
        uint16_t hdr;
        union {
            uint32_t obj[7];
            struct {
                uint16_t exthdr;
                uint8_t data[26];
            };
        };
    } __attribute__((packed));
} __attribute__((packed)) pd_msg;

#define PD_HDR_MSGTYPE_SHIFT 0
#define PD_HDR_MSGTYPE (0x1F << PD_HDR_MSGTYPE_SHIFT)
#define PD_HDR_DATAROLE_SHIFT 5
#define PD_HDR_DATAROLE (0x1 << PD_HDR_DATAROLE_SHIFT)
#define PD_HDR_SPECREV_SHIFT 6
#define PD_HDR_SPECREV (0x3 << PD_HDR_SPECREV_SHIFT)
#define PD_HDR_POWERROLE_SHIFT 8
#define PD_HDR_POWERROLE (1 << PD_HDR_POWERROLE_SHIFT)
#define PD_HDR_MESSAGEID_SHIFT 9
#define PD_HDR_MESSAGEID (0x7 << PD_HDR_MESSAGEID_SHIFT)
#define PD_HDR_NUMOBJ_SHIFT 12
#define PD_HDR_NUMOBJ (0x7 << PD_HDR_NUMOBJ_SHIFT)
#define PD_HDR_EXT (1 << 15)

#define PD_MSGTYPE_GET(msg) (((msg)->hdr & PD_HDR_MSGTYPE) >> PD_HDR_MSGTYPE_SHIFT)

// Control message
#define PD_MSGTYPE_GOODCRC 0x01
#define PD_MSGTYPE_GOTOMIN 0x02
#define PD_MSGTYPE_ACCEPT 0x03
#define PD_MSGTYPE_REJECT 0x04
#define PD_MSGTYPE_PING 0x05
#define PD_MSGTYPE_PS_RDY 0x06
#define PD_MSGTYPE_GET_SOURCE_CAP 0x07
#define PD_MSGTYPE_GET_SINK_CAP 0x08
#define PD_MSGTYPE_DR_SWAP 0x09
#define PD_MSGTYPE_PR_SWAP 0x0A
#define PD_MSGTYPE_VCONN_SWAP 0x0B
#define PD_MSGTYPE_WAIT 0x0C
#define PD_MSGTYPE_SOFT_RESET 0x0D
#define PD_MSGTYPE_NOT_SUPPORTED 0x10
#define PD_MSGTYPE_GET_SOURCE_CAP_EXTENDED 0x11
#define PD_MSGTYPE_GET_STATUS 0x12
#define PD_MSGTYPE_FR_SWAP 0x13
#define PD_MSGTYPE_GET_PPS_STATUS 0x14
#define PD_MSGTYPE_GET_COUNTRY_CODES 0x15
#define PD_MSGTYPE_GET_SINK_CAP_EXTENDED 0x16
#define PD_MSGTYPE_GET_SOURCE_INFO 0x17
#define PD_MSGTYPE_GET_REVISION 0x18

// Data message
#define PD_MSGTYPE_SOURCE_CAPABILITIES 0x01
#define PD_MSGTYPE_REQUEST 0x02
#define PD_MSGTYPE_BIST 0x03
#define PD_MSGTYPE_SINK_CAPABILITIES 0x04
#define PD_MSGTYPE_BATTERY_STATUS 0x05
#define PD_MSGTYPE_ALERT 0x06
#define PD_MSGTYPE_GET_COUNTRY_INFO 0x07
#define PD_MSGTYPE_ENTER_USB 0x08
#define PD_MSGTYPE_EPR_REQUEST 0x09
#define PD_MSGTYPE_EPR_MODE 0x0A
#define PD_MSGTYPE_SOURCE_INFO 0x0B
#define PD_MSGTYPE_REVISION 0x0C
#define PD_MSGTYPE_VENDOR_DEFINED 0x0F

// Extended message
#define PD_MSGTYPE_SOURCE_CAPABILITIES_EXTENDED 0x01
#define PD_MSGTYPE_STATUS 0x02
#define PD_MSGTYPE_GET_BATTERY_CAP 0x03
#define PD_MSGTYPE_GET_BATTERY_STATUS 0x04
#define PD_MSGTYPE_BATTERY_CAPABILITIES 0x05
#define PD_MSGTYPE_GET_MANUFACTURER_INFO 0x06
#define PD_MSGTYPE_MANUFACTURER_INFO 0x07
#define PD_MSGTYPE_SECURITY_REQUEST 0x08
#define PD_MSGTYPE_SECURITY_RESPONSE 0x09
#define PD_MSGTYPE_FIRMWARE_UPDATE_REQUEST 0x0A
#define PD_MSGTYPE_FIRMWARE_UPDATE_RESPONSE 0x0B
#define PD_MSGTYPE_PPS_STATUS 0x0C
#define PD_MSGTYPE_COUNTRY_INFO 0x0D
#define PD_MSGTYPE_COUNTRY_CODES 0x0E
#define PD_MSGTYPE_SINK_CAPABILITIES_EXTENDED 0x0F
#define PD_MSGTYPE_EXTENDED_CONTROL 0x10
#define PD_MSGTYPE_EPR_SOURCE_CAPABILITIES 0x11
#define PD_MSGTYPE_VENDOR_DEFINED_EXTENDED 0x1E

#define PD_DATAROLE_UFP (0x0 << PD_HDR_DATAROLE_SHIFT)
#define PD_DATAROLE_DFP (0x1 << PD_HDR_DATAROLE_SHIFT)

#define PD_SPECREV_1_0 (0x0 << PD_HDR_SPECREV_SHIFT)
#define PD_SPECREV_2_0 (0x1 << PD_HDR_SPECREV_SHIFT)
#define PD_SPECREV_3_0 (0x2 << PD_HDR_SPECREV_SHIFT)

#define PD_POWERROLE_SINK (0x0 << PD_HDR_POWERROLE_SHIFT)
#define PD_POWERROLE_SOURCE (0x1 << PD_HDR_POWERROLE_SHIFT)

#define PD_MESSAGEID_GET(msg) (((msg)->hdr & PD_HDR_MESSAGEID) >> PD_HDR_MESSAGEID_SHIFT)

#define PD_NUMOBJ(n) (((n) << PD_HDR_NUMOBJ_SHIFT) & PD_HDR_NUMOBJ)
#define PD_NUMOBJ_GET(msg) (((msg)->hdr & PD_HDR_NUMOBJ) >> PD_HDR_NUMOBJ_SHIFT)

#define PD_EXTHDR_DATA_SIZE_SHIFT 0
#define PD_EXTHDR_DATA_SIZE (0x1FF << PD_EXTHDR_DATA_SIZE_SHIFT)
#define PD_EXTHDR_REQUEST_CHUNK_SHIFT 10
#define PD_EXTHDR_REQUEST_CHUNK (1 << PD_EXTHDR_REQUEST_CHUNK_SHIFT)
#define PD_EXTHDR_CHUNK_NUMBER_SHIFT 11
#define PD_EXTHDR_CHUNK_NUMBER (0xF << PD_EXTHDR_CHUNK_NUMBER_SHIFT)
#define PD_EXTHDR_CHUNKED_SHIFT 15
#define PD_EXTHDR_CHUNKED (1 << PD_EXTHDR_CHUNKED_SHIFT)

#define PD_DATA_SIZE(n) (((n) << PD_EXTHDR_DATA_SIZE_SHIFT) & PD_EXTHDR_DATA_SIZE)
#define PD_DATA_SIZE_GET(msg) (((msg)->exthdr & PD_EXTHDR_DATA_SIZE) >> PD_EXTHDR_DATA_SIZE_SHIFT)

#define PD_CHUNK_NUMBER(n) (((n) << PD_EXTHDR_CHUNK_NUMBER_SHIFT) & PD_EXTHDR_CHUNK_NUMBER)
#define PD_CHUNK_NUMBER_GET(msg) (((msg)->exthdr & PD_EXTHDR_CHUNK_NUMBER) >> PD_EXTHDR_CHUNK_NUMBER_SHIFT)

#define PD_PDO_TYPE_SHIFT 30
#define PD_PDO_TYPE (0x3 << PD_PDO_TYPE_SHIFT)

#define PD_PDO_TYPE_FIXED ((unsigned)(0x0 << PD_PDO_TYPE_SHIFT))
#define PD_PDO_TYPE_BATTERY ((unsigned)(0x1 << PD_PDO_TYPE_SHIFT))
#define PD_PDO_TYPE_VARIABLE ((unsigned)(0x2 << PD_PDO_TYPE_SHIFT))
#define PD_PDO_TYPE_AUGMENTED ((unsigned)(0x3 << PD_PDO_TYPE_SHIFT))

#define PD_APDO_TYPE_SHIFT 28
#define PD_APDO_TYPE (0x3 << PD_APDO_TYPE_SHIFT)

#define PD_APDO_TYPE_PPS (0x0 << PD_APDO_TYPE_SHIFT)
#define PD_APDO_TYPE_AVS (0x1 << PD_APDO_TYPE_SHIFT)

#define PD_PDO_SRC_FIXED_DUAL_ROLE_PWR_SHIFT 29
#define PD_PDO_SRC_FIXED_DUAL_ROLE_PWR (1 << PD_PDO_SRC_FIXED_DUAL_ROLE_PWR_SHIFT)
#define PD_PDO_SRC_FIXED_USB_SUSPEND_SHIFT 28
#define PD_PDO_SRC_FIXED_USB_SUSPEND (1 << PD_PDO_SRC_FIXED_USB_SUSPEND_SHIFT)
#define PD_PDO_SRC_FIXED_UNCONSTRAINED_SHIFT 27
#define PD_PDO_SRC_FIXED_UNCONSTRAINED (1 << PD_PDO_SRC_FIXED_UNCONSTRAINED_SHIFT)
#define PD_PDO_SRC_FIXED_USB_COMMS_SHIFT 26
#define PD_PDO_SRC_FIXED_USB_COMMS (1 << PD_PDO_SRC_FIXED_USB_COMMS_SHIFT)
#define PD_PDO_SRC_FIXED_DUAL_ROLE_DATA_SHIFT 25
#define PD_PDO_SRC_FIXED_DUAL_ROLE_DATA (1 << PD_PDO_SRC_FIXED_DUAL_ROLE_DATA_SHIFT)
#define PD_PDO_SRC_FIXED_UNCHUNKED_EXT_MSG_SHIFT 24
#define PD_PDO_SRC_FIXED_UNCHUNKED_EXT_MSG (1 << PD_PDO_SRC_FIXED_UNCHUNKED_EXT_MSG_SHIFT)
#define PD_PDO_SRC_FIXED_EPR_CAPABLE_SHIFT 23
#define PD_PDO_SRC_FIXED_EPR_CAPABLE (1 << PD_PDO_SRC_FIXED_EPR_CAPABLE_SHIFT)
#define PD_PDO_SRC_FIXED_PEAK_CURRENT_SHIFT 20
#define PD_PDO_SRC_FIXED_PEAK_CURRENT (0x3 << PD_PDO_SRC_FIXED_PEAK_CURRENT_SHIFT)
#define PD_PDO_SRC_FIXED_VOLTAGE_SHIFT 10
#define PD_PDO_SRC_FIXED_VOLTAGE (0x3FF << PD_PDO_SRC_FIXED_VOLTAGE_SHIFT)
#define PD_PDO_SRC_FIXED_CURRENT_SHIFT 0
#define PD_PDO_SRC_FIXED_CURRENT (0x3FF << PD_PDO_SRC_FIXED_CURRENT_SHIFT)

#define PD_PDO_SRC_FIXED_CURRENT_GET(pdo) (((pdo) & PD_PDO_SRC_FIXED_CURRENT) >> PD_PDO_SRC_FIXED_CURRENT_SHIFT)

#define PD_PDO_SRC_FIXED_VOLTAGE_GET(pdo) (((pdo) & PD_PDO_SRC_FIXED_VOLTAGE) >> PD_PDO_SRC_FIXED_VOLTAGE_SHIFT)

#define PD_APDO_PPS_MAX_VOLTAGE_SHIFT 17
#define PD_APDO_PPS_MAX_VOLTAGE (0xFF << PD_APDO_PPS_MAX_VOLTAGE_SHIFT)
#define PD_APDO_PPS_MIN_VOLTAGE_SHIFT 8
#define PD_APDO_PPS_MIN_VOLTAGE (0xFF << PD_APDO_PPS_MIN_VOLTAGE_SHIFT)
#define PD_APDO_PPS_CURRENT_SHIFT 0
#define PD_APDO_PPS_CURRENT (0x7F << PD_APDO_PPS_CURRENT_SHIFT)
#define PD_APDO_AVS_MAX_VOLTAGE (0x1FF << PD_APDO_PPS_MAX_VOLTAGE_SHIFT)
#define PD_APDO_AVS_MAX_POWER (0xFF << PD_APDO_PPS_CURRENT_SHIFT)

#define PD_APDO_PPS_MAX_VOLTAGE_GET(pdo) (((pdo) & PD_APDO_PPS_MAX_VOLTAGE) >> PD_APDO_PPS_MAX_VOLTAGE_SHIFT)
#define PD_APDO_PPS_MIN_VOLTAGE_GET(pdo) (((pdo) & PD_APDO_PPS_MIN_VOLTAGE) >> PD_APDO_PPS_MIN_VOLTAGE_SHIFT)
#define PD_APDO_AVS_MAX_VOLTAGE_GET(pdo) (((pdo) & PD_APDO_AVS_MAX_VOLTAGE) >> PD_APDO_PPS_MAX_VOLTAGE_SHIFT)
#define PD_APDO_AVS_MAX_POWER_GET(pdo) (((pdo) & PD_APDO_AVS_MAX_POWER) >> PD_APDO_PPS_CURRENT_SHIFT)

#define PD_APDO_PPS_MAX_VOLTAGE_SET(v) (((v) << PD_APDO_PPS_MAX_VOLTAGE_SHIFT) & PD_APDO_PPS_MAX_VOLTAGE)
#define PD_APDO_PPS_MIN_VOLTAGE_SET(v) (((v) << PD_APDO_PPS_MIN_VOLTAGE_SHIFT) & PD_APDO_PPS_MIN_VOLTAGE)

#define PD_APDO_PPS_CURRENT_GET(pdo) ((uint8_t)(((pdo) & PD_APDO_PPS_CURRENT) >> PD_APDO_PPS_CURRENT_SHIFT))

#define PD_APDO_PPS_CURRENT_SET(i) (((i) << PD_APDO_PPS_CURRENT_SHIFT) & PD_APDO_PPS_CURRENT)

#define PD_PDO_SNK_FIXED_DUAL_ROLE_PWR_SHIFT 29
#define PD_PDO_SNK_FIXED_DUAL_ROLE_PWR (1 << PD_PDO_SNK_FIXED_DUAL_ROLE_PWR_SHIFT)
#define PD_PDO_SNK_FIXED_HIGHER_CAP_SHIFT 28
#define PD_PDO_SNK_FIXED_HIGHER_CAP (1 << PD_PDO_SNK_FIXED_HIGHER_CAP_SHIFT)
#define PD_PDO_SNK_FIXED_UNCONSTRAINED_SHIFT 27
#define PD_PDO_SNK_FIXED_UNCONSTRAINED (1 << PD_PDO_SNK_FIXED_UNCONSTRAINED_SHIFT)
#define PD_PDO_SNK_FIXED_USB_COMMS_SHIFT 26
#define PD_PDO_SNK_FIXED_USB_COMMS (1 << PD_PDO_SNK_FIXED_USB_COMMS_SHIFT)
#define PD_PDO_SNK_FIXED_DUAL_ROLE_DATA_SHIFT 25
#define PD_PDO_SNK_FIXED_DUAL_ROLE_DATA (1 << PD_PDO_SNK_FIXED_DUAL_ROLE_DATA_SHIFT)
#define PD_PDO_SNK_FIXED_VOLTAGE_SHIFT 10
#define PD_PDO_SNK_FIXED_VOLTAGE (0x3FF << PD_PDO_SNK_FIXED_VOLTAGE_SHIFT)
#define PD_PDO_SNK_FIXED_CURRENT_SHIFT 0
#define PD_PDO_SNK_FIXED_CURRENT (0x3FF << PD_PDO_SNK_FIXED_CURRENT_SHIFT)

#define PD_PDO_SNK_FIXED_CURRENT_SET(i) (((i) << PD_PDO_SNK_FIXED_CURRENT_SHIFT) & PD_PDO_SNK_FIXED_CURRENT)

#define PD_PDO_SNK_FIXED_VOLTAGE_SET(v) (((v) << PD_PDO_SNK_FIXED_VOLTAGE_SHIFT) & PD_PDO_SNK_FIXED_VOLTAGE)

#define PD_RDO_OBJPOS_SHIFT 28
#define PD_RDO_OBJPOS (0xF << PD_RDO_OBJPOS_SHIFT)
#define PD_RDO_GIVEBACK_SHIFT 27
#define PD_RDO_GIVEBACK (1 << PD_RDO_GIVEBACK_SHIFT)
#define PD_RDO_CAP_MISMATCH_SHIFT 26
#define PD_RDO_CAP_MISMATCH (1 << PD_RDO_CAP_MISMATCH_SHIFT)
#define PD_RDO_USB_COMMS_SHIFT 25
#define PD_RDO_USB_COMMS (1 << PD_RDO_USB_COMMS_SHIFT)
#define PD_RDO_NO_USB_SUSPEND_SHIFT 24
#define PD_RDO_NO_USB_SUSPEND (1 << PD_RDO_NO_USB_SUSPEND_SHIFT)
#define PD_RDO_UNCHUNKED_EXT_MSG_SHIFT 23
#define PD_RDO_UNCHUNKED_EXT_MSG (1 << PD_RDO_UNCHUNKED_EXT_MSG_SHIFT)
#define PD_RDO_EPR_CAPABLE_SHIFT 22
#define PD_RDO_EPR_CAPABLE (1 << PD_RDO_EPR_CAPABLE_SHIFT)

#define PD_RDO_OBJPOS_SET(i) (((i) << PD_RDO_OBJPOS_SHIFT) & PD_RDO_OBJPOS)
#define PD_RDO_OBJPOS_GET(msg) (((msg)->obj[0] & PD_RDO_OBJPOS) >> PD_RDO_OBJPOS_SHIFT)

#define PD_RDO_FV_CURRENT_SHIFT 10
#define PD_RDO_FV_CURRENT (0x3FF << PD_RDO_FV_CURRENT_SHIFT)
#define PD_RDO_FV_MAX_CURRENT_SHIFT 0
#define PD_RDO_FV_MAX_CURRENT (0x3FF << PD_RDO_FV_MAX_CURRENT_SHIFT)

#define PD_RDO_FV_CURRENT_SET(i) (((i) << PD_RDO_FV_CURRENT_SHIFT) & PD_RDO_FV_CURRENT)
#define PD_RDO_FV_MAX_CURRENT_SET(i) (((i) << PD_RDO_FV_MAX_CURRENT_SHIFT) & PD_RDO_FV_MAX_CURRENT)

#define PD_RDO_FV_MIN_CURRENT_SHIFT 0
#define PD_RDO_FV_MIN_CURRENT (0x3FF << PD_RDO_FV_MIN_CURRENT_SHIFT)

#define PD_RDO_FV_MIN_CURRENT_SET(i) (((i) << PD_RDO_FV_MIN_CURRENT_SHIFT) & PD_RDO_FV_MIN_CURRENT)

#define PD_RDO_PROG_VOLTAGE_SHIFT 9
#define PD_RDO_PROG_VOLTAGE (0x7FF << PD_RDO_PROG_VOLTAGE_SHIFT)
#define PD_RDO_PROG_CURRENT_SHIFT 0
#define PD_RDO_PROG_CURRENT (0x7F << PD_RDO_PROG_CURRENT_SHIFT)

#define PD_RDO_PROG_VOLTAGE_SET(i) (((i) << PD_RDO_PROG_VOLTAGE_SHIFT) & PD_RDO_PROG_VOLTAGE)
#define PD_RDO_PROG_CURRENT_SET(i) (((i) << PD_RDO_PROG_CURRENT_SHIFT) & PD_RDO_PROG_CURRENT)

#define PD_EPR_MODE_ACTION_SHIFT 24
#define PD_EPR_MODE_ACTION (0xFF << PD_EPR_MODE_ACTION_SHIFT)
#define PD_EPR_MODE_DATA_SHIFT 16
#define PD_EPR_MODE_DATA (0xFF << PD_EPR_MODE_DATA_SHIFT)

#define PD_EXTENDED_CONTROL_TYPE_EPR_GET_SOURCE_CAPABILITIES 1
#define PD_EXTENDED_CONTROL_TYPE_GET_SINK_CAPABILITIES 2
#define PD_EXTENDED_CONTROL_TYPE_EPR_KEEPALIVE 3
#define PD_EXTENDED_CONTROL_TYPE_EPR_KEEPALIVE_ACK 4

#define PD_EXTENDED_CONTROL_DATA_UNUSED 0

#define PD_T_CHUNKING_NOT_SUPPORTED (500)
#define PD_T_HARD_RESET_COMPLETE (1 * 1000)
#define PD_T_PS_TRANSITION (5 * 1000)
#define PD_T_SENDER_RESPONSE (2700)
#define PD_T_SINK_REQUEST (1 * 1000)
#define PD_T_TYPEC_SINK_WAIT_CAP (10 * 1000)
#define PD_T_PD_DEBOUNCE (2 * 1000)

#define PD_N_HARD_RESET_COUNT 2

#define PD_MAX_EXT_MSG_LEN 260
#define PD_MAX_EXT_MSG_CHUNK_LEN 26
#define PD_MAX_EXT_MSG_LEGACY_LEN 26

// Conversions
#define PD_MV2PRV(mv)  ((mv) / 20)
#define PD_PDV2MV(pdv) ((pdv)*50)
#define PD_PAV2MV(pav) ((pav)*100)

#define PD_PDI2MA(pdi) ((pdi)*10)
#define PD_PAI2MA(pai) ((pai)*50)

#define PD_MA2CA(ma)   (((ma) + 10 - 1) / 10)
#define PD_MA2PAI(ma)  (((ma) + 50 - 1) / 50)