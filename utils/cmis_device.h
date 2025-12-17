/* SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
 * Copyright 2018-2025 Hewlett Packard Enterprise Development LP
 */

/*
 * cmis_device.h - CMIS Device definitions
 * See https://www.oiforum.com/technical-work/implementation-agreements-ias/
 *     for the latest CMIS specification.
 */
#ifndef __CMIS_DEVICE_HEADER__
#define __CMIS_DEVICE_HEADER__

#ifdef __cplusplus
extern "C" {
#endif

#define CMIS_VENDOR_TE                  "TE Connectivity"
#define CMIS_VENDOR_HISENSE             "Hisense"
#define CMIS_VENDOR_FIT                 "FIT HON TENG"
#define CMIS_VENDOR_FT                  "FT HON TENG"
#define CMIS_VENDOR_MELLANOX            "Mellanox"
#define CMIS_VENDOR_MOLEX               "Molex"
#define CMIS_VENDOR_LEONI               "LEONI"
#define CMIS_VENDOR_HITACHI             "Hitachi Metals"
#define CMIS_VENDOR_LUXSHARE            "LUXSHARE-ICT"
#define CMIS_VENDOR_DUST_PHOTONICS      "DustPhotonics"
#define CMIS_VENDOR_FINISAR             "FINISAR CORP."
#define CMIS_VENDOR_HPE                 "HPE"
#define CMIS_VENDOR_CLOUD_LIGHT         "Cloud Light"

#define SFF8024_TYPE_QSFPDD             0x18
#define SFF8024_TYPE_OSFP8X             0x19
#define SFF8024_TYPE_SFPDD              0x1A
#define SFF8024_TYPE_DSFP               0x1B
#define SFF8024_TYPE_MINILINK4X         0x1C
#define SFF8024_TYPE_MINILINK8X         0x1D
#define SFF8024_TYPE_QSFP_PLUS_CMIS     0x1E
#define SFF8024_TYPE_SFPDD_CMIS         0x1F
#define SFF8024_TYPE_SFP_PLUS_CMIS      0x20
#define SFF8024_TYPE_OSFPXD_CMIS        0x21
#define SFF8024_TYPE_ELSFP_CMIS         0x22
#define SFF8024_TYPE_CDFP_PCIE_X4       0x23
#define SFF8024_TYPE_CDFP_PCIE_X8       0x24
#define SFF8024_TYPE_CDFP_PCIE_X16      0x25

/* CMIS Characteristics Byte @ 2 */
#define CMIS_FLAT                       (1 << 7)

/* CMIS Status Byte @ 3 */
#define CMIS_STATE_MASK                 0x0E
#define CMIS_STATE_SHIFT                1
#define CMIS_STATE_RESVD_0              0
#define CMIS_STATE_LOW_PWR              1
#define CMIS_STATE_PWR_UP               2
#define CMIS_STATE_READY                3
#define CMIS_STATE_PWR_DOWN             4
#define CMIS_STATE_FAULT                5
#define CMIS_STATE_RESVD_6              6
#define CMIS_STATE_RESVD_7              7
#define CMIS_INTERRUPT                  (1 << 0)

/* CMIS Status Flags @ 8 */
#define CMIS_CDB_CMD_COMPLETE_2         (1 << 7)
#define CMIS_CDB_CMD_COMPLETE_1         (1 << 6)
#define CMIS_DATA_PATH_FW_ERROR         (1 << 2)
#define CMIS_MODULE_FW_ERROR            (1 << 1)
#define CMIS_MODULE_STATE_CHANGED       (1 << 0)

#ifdef __cplusplus
}
#endif
#endif // __CMIS_DEVICE_HEADER__
