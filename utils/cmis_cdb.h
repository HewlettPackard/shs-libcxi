/* SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
 * Copyright 2018-2025 Hewlett Packard Enterprise Development LP
 */

/*
 * cmis_cdb.h - Command Data Block definitions
 * See https://www.oiforum.com/technical-work/implementation-agreements-ias/
 *     for the latest CMIS specification.
 */
#ifndef __CMIS_CDB_HEADER__
#define __CMIS_CDB_HEADER__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Lower Page ****************************************************************/
/* For CDB Complete Status see CdbCmdCompleteFlag[12] in byte 8 */

/* CDB status */
#define CDB_BLOCK1_STATUS                       37
#define CDB_BLOCK2_STATUS                       38
#define CDB_STS_BUSY_MASK                       (1 << 7)
#define CDB_STS_FAIL_MASK                       (1 << 6)
#define CDB_STS_RESULT_MASK                     0x3F
#define CDB_STS_SUCCESS                         1
/* See spec for failure status codes */

/* PAGE 01h ******************************************************************/
/* CDB advertisement */
#define CDB_ADVERTISEMENT                       163
#define CDB_IMPLEMENTED_MASK                    (3 << 6)
#define CDB_IMPLEMENTED_NONE                    (0 << 6)
#define CDB_IMPLEMENTED_ONE_BANK                (1 << 6)
#define CDB_IMPLEMENTED_BOTH_BANKS              (2 << 6)
#define CDB_IMPLEMENTED_RESERVED                (3 << 6)
#define CDB_BACKGROUND_IMPLEMENTED_MASK         (1 << 5)
#define CDB_AUTO_PAGING_IMPLEMENTED_MASK        (1 << 4)
#define CDB_NUM_EPL_PAGES_MASK                  0x0F

/* CDB maximum bytes per I2C (see spec) */
#define CDB_READ_WRITE_LENGTH_EXT               164

/* CDB command options */
#define CDB_COMMAND_OPTIONS                     165
#define CDB_COMMAND_PROCESSING_ON_STOP          (1 << 7)
#define CDB_EXT_MAX_BUSY_TIME_MASK              0x1F
#define CDB_tNACK_MASK                          CDB_EXT_MAX_BUSY_TIME_MASK

/* CDB command options */
#define CDB_MAX_BUSY_OPTIONS                    166
#define CDB_MAX_BUSY_IS_EXT_MAX_BUSY            (1 << 7)
#define CDB_MAX_BUSY_TIME_SUB80_MASK            0x3F

/* PAGE 9Fh ******************************************************************/
#define CDB_PAGE                                0x9F

#define CDB_START                               128

#define CDB_CMD_MSB                             128
#define CDB_CMD_LSB                             129
#define CDB_CMD_EPL_LENGTH_MSB                  130
#define CDB_CMD_EPL_LENGTH_LSB                  131
#define CDB_CMD_LPL_LENGTH                      132
#define CDB_CMD_CDB_CHK_CODE                    133
#define CDB_CMD_RLPL_LENGTH                     134
#define CDB_CMD_RLPL_CHK_CODE                   135
#define CDB_LPL_START                           136
#define CDB_HDR_SIZE                            (CDB_LPL_START - CDB_CMD_MSB)

#define CDB_LPL_MAXLEN                          120

/* CDB Module Commands */
#define CDB_CMD_QUERY_STATUS                    0x0000
#define CDB_CMD_ENTER_PASSWORD                  0x0001
#define CDB_CMD_CHANGE_PASSWORD                 0x0002
#define CDB_CMD_ENABLE_DISABLE_PASSWORD         0x0003 // Reserved
#define CDB_CMD_ABORT_BACKGROUND_OPERATION      0x0004

/* CDB Feature and Capabilities Commands */
#define CDB_CMD_MODULE_FEATURES                 0x0040
#define CDB_CMD_FW_MGMT_FEATURES                0x0041
#define CDB_CMD_PERFORMANCE_MONITORING          0x0042
#define CDB_CMD_BERT_DIAGNOSTICS_MONITORING     0x0043

/* CDB Firmware Download Commands */
#define CDB_CMD_GET_FW_INFO                     0x0100
#define CDB_CMD_START_FW_DOWNLOAD               0x0101
#define CDB_CMD_ABORT_FW_DOWNLOAD               0x0102
#define CDB_CMD_WRITE_FW_LPL                    0x0103
#define CDB_CMD_WRITE_FW_EPL                    0x0104
#define CDB_CMD_READ_FW_LPL                     0x0105
#define CDB_CMD_READ_FW_EPL                     0x0106
#define CDB_CMD_COMPLETE_FW_DOWNLOAD            0x0107
#define CDB_CMD_COPY_FW_IMAGE                   0x0108
#define CDB_CMD_RUN_FW_IMAGE                    0x0109
#define CDB_CMD_COMMIT_FW_IMAGE                 0x010A

/* CDB_CMD_RUN_FW_IMAGE Run Options */
#define CDB_TRAFFIC_AFF_RUN_INACTIVE_IMG        0x00
#define CDB_ATTEMPT_HITLESS_RUN_INACTIVE_IMG    0x01
#define CDB_TRAFFIC_AFF_RESET_ACTIVE_IMG        0x02
#define CDB_ATTEMPT_HITLESS_RESET_ACTIVE_IMG    0x03

/* Command Data Block */
struct cmis_cdb {
	uint8_t CommandMSB;                         // 128
	uint8_t CommandLSB;                         // 129
	uint8_t EPLLengthMSB;                       // 130
	uint8_t EPLLengthLSB;                       // 131
	uint8_t LPLLength;                          // 132
	uint8_t CDBChkCode;                         // 133
	uint8_t RLPLLength;                         // 134
	uint8_t RLPLChkCode;                        // 135
	uint8_t LocalPayload[CDB_LPL_MAXLEN];       // 136-255
} __attribute__((__packed__));

struct cmis_cdb_advertisement {
	uint8_t p01h_163;
	uint8_t p01h_164;
	uint8_t p01h_165;
	uint8_t p01h_166;
} __attribute__((__packed__));

struct cmis_fw_mgmt_features_payload {
	uint8_t Reserved136;
	uint8_t FWFeaturesFlags;
#define CDB_FW_IMAGE_READBACK_SUPPORTED         (1 << 7)
#define CDB_FW_MAX_DURATION_CODING_10X          (1 << 3)
#define CDB_FW_SKIPPING_ERASED_BLOCKS_SUPPORTED (1 << 2)
#define CDB_FW_COPY_CMD_SUPPORTED               (1 << 1)
#define CDB_FW_ABORT_CMD_SUPPORTED              (1 << 0)
	uint8_t StartCommandPayloadSize;
	uint8_t ErasedByte;
	uint8_t ReadWriteLengthExt;
	uint8_t WriteMechanism;
#define CDB_FW_WRITE_LPL_SUPPORTED              (1 << 0)
#define CDB_FW_WRITE_EPL_SUPPORTED              (1 << 4)
	uint8_t ReadMechanism;
#define CDB_FW_READ_LPL_SUPPORTED               (1 << 0)
#define CDB_FW_READ_EPL_SUPPORTED               (1 << 4)
	uint8_t HitlessRestart;
	uint16_t MaxDurationStart;
	uint16_t MaxDurationAbort;
	uint16_t MaxDurationWrite;
	uint16_t MaxDurationComplete;
	uint16_t MaxDurationCopy;
} __attribute__((__packed__));

struct cmis_fw_info_payload {
	uint8_t FWStatusFlags;
#define CDB_FW_IMG_A_RUNNING                    (1 << 0)
#define CDB_FW_IMG_A_COMMITED_TO_BOOT           (1 << 1)
#define CDB_FW_IMG_A_INVALID                    (1 << 2)
#define CDB_FW_IMG_A_EMPTY                      CDB_FW_IMG_A_INVALID
#define CDB_FW_IMG_A_RSVD                       (1 << 3)
#define CDB_FW_IMG_B_RUNNING                    (1 << 4)
#define CDB_FW_IMG_B_COMMITED_TO_BOOT           (1 << 5)
#define CDB_FW_IMG_B_INVALID                    (1 << 6)
#define CDB_FW_IMG_B_EMPTY                      CDB_FW_IMG_B_INVALID
#define CDB_FW_IMG_B_RSVD                       (1 << 7)
	uint8_t FWInfoPresentFlags;
#define CDB_FW_IMG_A_PRESENT                    (1 << 0)
#define CDB_FW_IMG_B_PRESENT                    (1 << 1)
#define CDB_FW_IMG_FACTORY_PRESENT              (1 << 2)
	uint8_t ImageAMajor;
	uint8_t ImageAMinor;
	uint16_t ImageABuildNumber;
	char ImageAExtra[32];
	uint8_t ImageBMajor;
	uint8_t ImageBMinor;
	uint16_t ImageBBuildNumber;
	char ImageBExtra[32];
	uint8_t ImageFactoryMajor;
	uint8_t ImageFactoryMinor;
	uint16_t ImageFactoryBuildNumber;
	char ImageFactoryExtra[32];
} __attribute__((__packed__));

#ifdef __cplusplus
}
#endif
#endif // __CMIS_CDB_HEADER__
