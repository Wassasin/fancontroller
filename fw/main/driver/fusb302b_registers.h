#pragma once

typedef union {
    struct
    {
        uint8_t revision_id : 2;
        uint8_t product_id : 2;
        uint8_t version_id : 4;
    };
    uint8_t val;
} reg_device_id_t;
#define REG_DEVICE_ID 0x01

typedef union {
    struct
    {
        uint8_t pdwn1 : 1;
        uint8_t pdwn2 : 1;
        uint8_t meas_cc1 : 1;
        uint8_t meas_cc2 : 1;
        uint8_t vconn_cc1 : 1;
        uint8_t vconn_cc2 : 1;
        uint8_t pu_en1 : 1;
        uint8_t pu_en2 : 1;
    };
    uint8_t val;
} reg_switches0_t;
#define REG_SWITCHES0 0x02

typedef union {
    struct
    {
        uint8_t txcc1 : 1;
        uint8_t txcc2 : 1;
        uint8_t auto_crc : 1;
        uint8_t reserved : 1;
        uint8_t datarole : 1;
        uint8_t specrev : 2;
        uint8_t powerrole : 1;
    };
    uint8_t val;
} reg_switches1_t;
#define REG_SWITCHES1 0x03

typedef union {
    struct
    {
        uint8_t tx_start : 1;
        uint8_t auto_pre : 1;
        uint8_t host_cur : 2;
        uint8_t reserved1 : 1;
        uint8_t int_mask : 1;
        uint8_t tx_flush : 1;
        uint8_t reserved2 : 1;
    };
    uint8_t val;
} reg_control0_t;
#define REG_CONTROL0 0x06

typedef union {
    struct
    {
        uint8_t auto_retry : 1;
        uint8_t n_retries : 2;
        uint8_t auto_softreset : 2;
        uint8_t auto_hardreset : 1;
        uint8_t bist_tmode : 1;
        uint8_t send_hard_reset : 1;
        uint8_t reserved : 1;
    };
    uint8_t val;
} reg_control3_t;
#define REG_CONTROL3 0x09

typedef union {
    struct
    {
        uint8_t m_bc_lvl : 1;
        uint8_t m_collision : 1;
        uint8_t m_wake : 1;
        uint8_t m_alert : 1;
        uint8_t m_crc_chk : 1;
        uint8_t m_comp_chng : 1;
        uint8_t m_activity : 1;
        uint8_t m_vbusok : 1;
    };
    uint8_t val;
} reg_mask_t;
#define REG_MASK 0x0A

typedef union {
    struct
    {
        uint8_t pwr_bandgap_wake : 1;
        uint8_t pwr_receiver_ref : 1;
        uint8_t pwr_measure : 1;
        uint8_t pwr_osc : 1;
        uint8_t reserved : 4;
    };
    uint8_t val;
} reg_power_t;
#define REG_POWER 0x0B

typedef union {
    struct
    {
        uint8_t sw_res : 1;
        uint8_t pd_reset : 1;
        uint8_t reserved : 6;
    };
    uint8_t val;
} reg_reset_t;
#define REG_RESET 0x0C

typedef union {
    struct
    {
        uint8_t bc_lvl : 2;
        uint8_t wake : 1;
        uint8_t alert : 1;
        uint8_t crc_chk : 1;
        uint8_t comp : 1;
        uint8_t activity : 1;
        uint8_t vbusok : 1;
    };
    uint8_t val;
} reg_status0_t;
#define REG_STATUS0 0x40