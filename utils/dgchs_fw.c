/* SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
 * Copyright 2020-2026 Hewlett Packard Enterprise Development LP
 */

/* CMIS CDB (Command Data Block) Firmware Management for Cassini
 *
 * CMIS defines a firmware update method using CDB commands and data for
 * updating firmware in CMIS devices. Protocol compatibility is not 100%
 * guaranteed for new devices so unit testing and possibly fixes or vendor-
 * specific workarounds may be required.
 *
 * The CDB firmware upgrade typically works best when the headshell is in
 * the low power state. We attempt to set low power mode before starting
 * the upgrade unless the --no-reset option is specified.
 *
 * The --no-reset option is useful if the module requires special handling
 * such as staying in full-power mode or needing specific CMIS writes prior
 * to the upgrade.
 *
 * During the upgrade, we typically need exclusive access to the headshell
 * I2C. We use flock() to prevent conflict with other processes but the
 * link driver might interfere with the headshell unless the sysfs file is
 * used to lockout the link driver. Example:
 *   echo Y > /sys/kernel/debug/cxi/cxi0/i2c_locked
 * and
 *   echo N > /sys/kernel/debug/cxi/cxi0/i2c_locked
 * We do this automatically unless the --no-lockout option is specified.
 *
 * CMIS CDB Firmware Upgrade Overview:
 * -----------------------------------
 * The CMIS device advertises support for specific CDB Firmware Management
 * features and supplies parameters that we (the host) need to know in order
 * to successfully perform an upgrade.
 *
 * The vendor supplies a firmware image file (e.g. binary or Intel HEX file)
 * that starts with a firmware header of 112 bytes or less. In theory, this
 * single firmware file and the CMIS CDB device advertisements are all we
 * need to know to be able to do a firmware upgrade. In reality, the vendor
 * *might* require us to perform extra CMIS writes in preparation for the CDB
 * firmware update or they might require deviation from the CMIS
 * specification due to bugs in their existing firmware.
 *
 * For CMIS CDB firmware upgrades, the CMIS device is expected to have
 * storage space for two firmware images, A and B. One of the images is
 * marked as the "boot" image and will be loaded at the next power on or
 * reset of the device. One of the images is "running". The module determines
 * which image is overwritten by the update (typically the one that is not
 * the "running" image). The CDB firmware update process is designed to be
 * fail-safe.
 *
 * In the CMIS specification, the term "committed" indicates the image that
 * is marked as the "boot" image.
 *
 * Minimal CMIS CDB Firmware Upgrade Process:
 * ------------------------------------------
 *
 *  1 - We query the CMIS device for its CDB Firmware Management
 *      features and parameters per 01h:163-166 and CMD0041h Firmware
 *      Management Features.
 *  2 - We read in the firmware file provided by the user.
 *  3 - We send the CMD0101h Start Firmware Download command with
 *      the firmware header.
 *  4 - The CMIS device evaluates the command and the firmware header and
 *      accepts or rejects the command.
 *  5 - We write firmware into the device using CMD0103 Write Firmware
 *      block LPL or CMD0104 Write Firmware Block EPL commands.
 *  6 - The CMIS device accepts or rejects the Write Firmware commands.
 *  7 - After all firmware has been downloaded, we send the CMD0107
 *      Complete Firmware Download command.
 *  8 - The CMIS device validates the firmware and accepts or rejects the
 *      command.
 *  9 - We send the CMD0109h Run Firmware Image command.
 * 10 - The CMIS device begins running the new firmware image. At this
 *      point, the old image is still the "boot" image and any power
 *      cycle or reset of the headshell will run the previous firmware.
 * 11 - We send the CMD010Ah Commit Firmware Image command.
 * 12 - If the new firmware image is successfully running on the CMIS
 *      device, it will be marked as the "boot" image so that it will
 *      run on any subsequent power cycle or reset of the headshell.
 * 13 - The firmware update is complete.
 *
 * The CMIS device is ultimately responsible for validating the firmware
 * image that we load. This validation includes:
 *   1 - The firmware header is the first data we write to the device. The
 *       device parses the header to determine if it is a valid firmware
 *       image that the device can accept. The contents of the header are
 *       vendor / device specific. CMIS does not specify the header
 *       contents. If the device rejects the header, the firmware update
 *       immediately fails.
 *   2 - We load the firmware in multiple I2C writes. In the case of LPL
 *       writes, the I2C data is protected by a simple checksum. In the case
 *       of EPL writes, the I2C data (other than the command itself) is not
 *       protected by any checksum. In either case, the CMIS device is
 *       responsible for validating the integrity of the complete firmware
 *       image after it has been loaded.
 *   3 - If the device accepts the firmware image, we tell the device to
 *       run the firmware. As long as the new firmware runs well enough to
 *       boot and accept the Commit Firmware Image command, it will become
 *       the new "boot" firmware for the device.
 *   4 - If the new firmware fails to boot or fails to function well enough,
 *       the module will either boot into the old image or it will need to
 *       be reset or power cycled so that the old firmware runs and the
 *       upgrade process can be re-attempted.
 *
 * Firmware file format
 * --------------------
 * We only support binary files at this time but see
 * hms-controllers/lib/chfscommon/qdd_cdb.c for Intel HEX file support.
 *
 * The contents of this file were based on
 * hms-controllers/lib/chfscommon/qdd_cdb.c.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include "cmis_cdb.h"
#include "cmis_device.h"
#include "dgchs_i2c.h"

#define APPLICATION_VERSION                     "1.0"
#define DEFAULT_DEVICE_NAME                     "cxi0"

/* Intel Hex Record Types */
#define IHEX_RECTYPE_DATA                       0
#define IHEX_RECTYPE_EOF                        1
#define IHEX_RECTYPE_EXT_SEG_ADDR               2
#define IHEX_RECTYPE_START_SEG_ADDR             3
#define IHEX_RECTYPE_EXT_LINEAR_ADDR            4
#define IHEX_RECTYPE_START_LINEAR_ADDR          5

#define FW_BUF_MALLOC_MAXSIZE                   (2*1024*1024)

#define CDB_RESULT_POLLING_MAX_MS               50

#define CDB_FW_HEADER_SIZE_MAX                  112

/* Note: Using a larger chunk size depends on support from the device
 *       (see page 01h byte 164) and fw_mgmt_features
 *       ReadWriteLengthExt and our block write routine.
 */
#define CDB_CHUNK_SIZE                          8

#define print_out(format, args...) printf(format, ## args)
#define error_out(format, args...) fprintf(stderr, format, ## args)
#define debug_out(format, args...) { if (verbose) fprintf(stderr, format, ## args); }

/* Container for the details of a firmware update for a CMIS device */
struct cmis_fw_update {
	char *device;
	uint32_t password;
	int bus;
	int fh;
	uint8_t sff8024_identifier;
	uint8_t cmis_revision;
	uint16_t active_fw_rev;
	uint16_t inactive_fw_rev;
	uint16_t hardware_rev;
	char vendor[32];
	char part_num[32];
	char serial_num[32];
	char module_revision[4];
	char oem[64];
	char *fw_filename;
	uint8_t *buf;
	uint32_t length;
	struct cmis_cdb_advertisement adv;
	struct cmis_fw_mgmt_features_payload features;
};

/* Message / debug output verbosity */
int verbose;

/**
 * @brief Returns the number milliseconds, since time 0.
 */
static uint64_t
get_curtime_ms(void)
{
	struct timespec tv;

	clock_gettime(CLOCK_MONOTONIC, &tv);
	return tv.tv_sec * 1000 + tv.tv_nsec / 1000000;
}

/**
 * @brief Remove trailing spaces and ctrl codes from input string
 *
 * @param str char array to trim
 */
static void
str_trim_whitespace_r(char *str)
{
	char *end;

	if (*str == 0)
		return;

	end = str + strlen(str) - 1;
	while (end >= str && *end <= ' ')
		end--;

	end[1] = '\0';
}

/* get_cdb_advertisement()
 * Read the 4 bytes from page 01h containing CDB advertisement information
 * and store them in fwu->adv.
 */
static int
get_cdb_advertisement(struct cmis_fw_update *fwu)
{
	int rc;

	/* Read page 01h bytes 163, 164, 165, 166 */
	rc = cmis_read_block(fwu->fh, 1, CDB_ADVERTISEMENT, 4, (uint8_t *)&fwu->adv);
	if (rc < 0) {
		return rc;
	}
	if (verbose) {
		int max_bus_time_ms;

		debug_out("p01h_163 = 0x%02x\n", fwu->adv.p01h_163);
		debug_out("p01h_164 = 0x%02x\n", fwu->adv.p01h_164);
		debug_out("p01h_165 = 0x%02x\n", fwu->adv.p01h_165);
		debug_out("p01h_166 = 0x%02x\n", fwu->adv.p01h_166);

		debug_out("CDB Advertisement:\n");
		debug_out("  Implemented Banks: ");
		switch (fwu->adv.p01h_163 & CDB_IMPLEMENTED_MASK) {
		case CDB_IMPLEMENTED_NONE:
			debug_out("0\n");
			break;
		case CDB_IMPLEMENTED_ONE_BANK:
			debug_out("1\n");
			break;
		case CDB_IMPLEMENTED_BOTH_BANKS:
			debug_out("2\n");
			break;
		default:
			debug_out("Reserved\n");
			break;
		}
		debug_out("  Background Mode Implemented: %s\n",
			(fwu->adv.p01h_163 & CDB_BACKGROUND_IMPLEMENTED_MASK) ? "Yes" : "No");
		debug_out("  Auto Paging Implemented: %s\n",
			(fwu->adv.p01h_163 & CDB_AUTO_PAGING_IMPLEMENTED_MASK) ? "Yes" : "No");
		debug_out("  Command Processing on Stop: %s\n",
			(fwu->adv.p01h_165 & CDB_COMMAND_PROCESSING_ON_STOP) ? "Yes" : "No");
		debug_out("  CdbMaxPagesEPL: %u (encoded)\n",
			fwu->adv.p01h_163 & CDB_NUM_EPL_PAGES_MASK);
		debug_out("  CdbReadWriteLengthExtension: %u (encoded)\n",
			fwu->adv.p01h_164);
		if (fwu->adv.p01h_166 & CDB_MAX_BUSY_IS_EXT_MAX_BUSY) {
			max_bus_time_ms = 160 * (fwu->adv.p01h_165 & CDB_EXT_MAX_BUSY_TIME_MASK);
		} else {
			max_bus_time_ms = 80 - (fwu->adv.p01h_166 & CDB_MAX_BUSY_TIME_SUB80_MASK);
			if (max_bus_time_ms < 0)
				max_bus_time_ms = 0;
		}
		debug_out("  Max Busy Time: %u ms\n", max_bus_time_ms);
	}
	debug_out("\n");

	return rc;
}

/* send_cdb_command()
 * Add/fix the header fields in the provided cdb (payload pre-loaded) and send it.
 */
static int
send_cdb_command(struct cmis_fw_update *fwu, struct cmis_cdb *cdb,
		uint16_t command, uint16_t epl_len, uint8_t lpl_len)
{
	int cdb_len = CDB_HDR_SIZE + lpl_len;
	int remain = cdb_len - 2; /* Don't include Command bytes for now */
	int i, chunksize, rc, fh = fwu->fh;
	uint8_t chksum, addr;
	uint8_t *ptr;

	/* Set/Reset the header bytes */
	cdb->CommandMSB = command >> 8;
	cdb->CommandLSB = command & 0xFF;
	cdb->EPLLengthMSB = epl_len >> 8;
	cdb->EPLLengthLSB = epl_len & 0xFF;
	cdb->LPLLength = lpl_len;
	cdb->CDBChkCode = 0;
	cdb->RLPLLength = 0;
	cdb->RLPLChkCode = 0;

	/* Generate the ChkCode */
	chksum = 0;
	ptr = (uint8_t *)cdb;
	for (i = 0; i < cdb_len; i++) {
		chksum += *ptr++;
	}
	cdb->CDBChkCode = chksum ^ 0xFF;

	if ((rc = cmis_set_page(fh, CDB_PAGE)) < 0) {
		error_out("Unable to set CDB page\n");
		return rc;
	}

	/* Start writing after the COMMAND */
	ptr = &cdb->EPLLengthMSB;
	addr = CDB_CMD_EPL_LENGTH_MSB;
	while (remain > 0) {
		chunksize = remain < CDB_CHUNK_SIZE ? remain : CDB_CHUNK_SIZE;
		rc = cmis_write_raw_block(fh, addr, chunksize, ptr);
		if (rc < 0) {
			return rc;
		}
		remain -= chunksize;
		ptr += chunksize;
		addr += chunksize;
	}

	/* Now write the COMMAND to trigger CDB processing */
	ptr = &cdb->CommandMSB;
	addr = CDB_CMD_MSB;
	rc = cmis_write_raw_block(fh, addr, 2, ptr);
	if (rc > 0) {
		rc = 0;
	}
	return rc;
}

/** wait_for_cdb_complete()
 * CMIS 4.0 added clear-on-read flags indicating CDB command completion in
 * byte 08. This function polls for the CDB1 command completion flag.
 *
 * returns 0 on success or negative errno on error.
 */
static int
wait_for_cdb_complete(struct cmis_fw_update *fwu, int timeout_ms)
{
	uint64_t start = get_curtime_ms();
	uint8_t byte08;
	int rc = 0;

	while (1) {
		rc = cmis_read_byte(fwu->fh, 0, 8, &byte08);
		if (rc < 0) {
			debug_out("%s: Unable to read CDB complete flag (rc=%d)\n", __func__, rc);
		} else if (byte08 & CMIS_CDB_CMD_COMPLETE_1) {
			break;
		}

		if ((get_curtime_ms() - start) > timeout_ms) {
			if (rc == 0)
				rc = -ETIMEDOUT;
			break;
		}

		if (timeout_ms > 1000)
			usleep(100000); /* Wait 100ms and try again */
		else
			usleep(10000);  /* Wait 10ms and try again */
	}
	return rc;
}

/* get_cdb_status()
 * Poll for the CDB1 status byte with possible timeout. We only support CDB1
 * at this time.
 *
 * @param timeout_ms Number of milliseconds to wait for the command to
 *                   complete or fail.
 * @returns The CDB Status byte (see spec for interpretation) with
 *          success being 0x01      (generally)
 *                        0x00-0x3F (broadly)
 *          error   being 0x40-0x7F
 *          or negative errno on timeout or other i/o error
 */
static int
get_cdb_status(struct cmis_fw_update *fwu, int timeout_ms)
{
	uint64_t start = get_curtime_ms();
	uint8_t status;
	int rc = 0;

	/* Wait for the CDB command to complete */
	rc = wait_for_cdb_complete(fwu, timeout_ms);
	if (rc < 0) {
		debug_out("%s: wait_for_cdb_complete() failed (rc=%d)\n", __func__, rc);
		/* Even if that failed, we still try to read the status at least */
		/* once before we report an error or timeout */
	}

	while (1) {
		/* Read the CDB block 1 status byte */
		rc = cmis_read_byte(fwu->fh, 0, CDB_BLOCK1_STATUS, &status);
		if ((rc = cmis_read_byte(fwu->fh, 0, CDB_BLOCK1_STATUS, &status)) < 0) {
			/* NAK expected if module doesn't support 'background cdb processing' */
			status = CDB_STS_BUSY_MASK;
			debug_out("%s: Unable to read CDB status byte (rc=%d)\n", __func__, rc);
		}

		/* If BUSY clear OR FAIL set, we're done */
		if (!(status & CDB_STS_BUSY_MASK) || (status & CDB_STS_FAIL_MASK)) {
			break;
		}
		if ((get_curtime_ms() - start) > timeout_ms) {
			rc = -ETIMEDOUT;
			break;
		}

		if (timeout_ms > 1000)
			usleep(100000); /* Wait 100ms and try again */
		else
			usleep(10000);  /* Wait 10ms and try again */
	}

	if (rc < 0)
		return rc;

	return status;
}

/* get_cdb_response()
 * Read the CDB response out of the device into the provided cdb.
 * @returns 0 on success, -EBADMSG if a chkcode or data interpretation
 *          error occurred or other negative errno.
 */
static int
get_cdb_response(struct cmis_fw_update *fwu, struct cmis_cdb *cdb)
{
	uint8_t *ptr;
	uint8_t chkcode, addr;
	int i, rc, remain, chunksize, retries = 5;

	while (retries--) {
		if ((rc = cmis_set_page(fwu->fh, CDB_PAGE)) < 0) {
			debug_out("Unable to set CDB page\n");
			continue;
		}

		/* Read the header fields */
		ptr = (uint8_t *)cdb;
		addr = CDB_START;
		if ((rc = cmis_read_raw_block(fwu->fh, addr, CDB_HDR_SIZE, ptr)) < 0) {
			continue;
		}

		/* Read the Response Local Payload bytes */
		remain = cdb->RLPLLength;
		ptr = &cdb->LocalPayload[0];
		addr = CDB_LPL_START;
		if (remain > CDB_LPL_MAXLEN) {
			rc = -EINVAL;
			continue;
		}
		while (remain > 0) {
			chunksize = remain < CDB_CHUNK_SIZE ? remain : CDB_CHUNK_SIZE;
			/* Continue reading the CDB */
			if ((rc = cmis_read_raw_block(fwu->fh, addr, chunksize, ptr)) < 0) {
				break;
			}
			remain -= chunksize;
			ptr += chunksize;
			addr += chunksize;
		}
		if (rc < 0) {
			continue;
		}

		/* Calculate and validate the RLPLChkCode */
		chkcode = 0;
		ptr = (uint8_t *)cdb->LocalPayload;
		for (i = 0; i < cdb->RLPLLength; i++) {
			chkcode += *ptr++;
		}
		chkcode ^= 0xFF;
		if (cdb->RLPLChkCode != chkcode) {
			debug_out("%s: RLPLChkCode error (0x%02x != 0x%02x)\n",
						__func__, cdb->RLPLChkCode, chkcode);
			rc = -EBADMSG;
			continue;
		}
		rc = 0;
		break;
	}

	return rc;
}

/* get_start_timeout_ms()
 * Returns a timeout ms value based on the MaxDurationStart value.
 */
static int
get_start_timeout_ms(struct cmis_fw_update *fwu)
{
	int timeout_ms;

	timeout_ms = fwu->features.MaxDurationStart;
	if (fwu->features.FWFeaturesFlags & CDB_FW_MAX_DURATION_CODING_10X)
		timeout_ms *= 10;
	if (!timeout_ms)
		timeout_ms = 10000;

	return timeout_ms;
}

/* get_write_timeout_ms()
 * Returns a timeout ms value based on the MaxDurationWrite value.
 */
static int
get_write_timeout_ms(struct cmis_fw_update *fwu)
{
	int timeout_ms;

	timeout_ms = fwu->features.MaxDurationWrite;
	if (fwu->features.FWFeaturesFlags & CDB_FW_MAX_DURATION_CODING_10X)
		timeout_ms *= 10;
	if (!timeout_ms)
		timeout_ms = 1000;

	return timeout_ms;
}

/* get_complete_timeout_ms()
 * Returns a timeout ms value based on the MaxDurationComplete value.
 */
static int
get_complete_timeout_ms(struct cmis_fw_update *fwu)
{
	int timeout_ms;

	timeout_ms = fwu->features.MaxDurationComplete;
	if (fwu->features.FWFeaturesFlags & CDB_FW_MAX_DURATION_CODING_10X)
		timeout_ms *= 10;
	if (!timeout_ms)
		timeout_ms = 10000;

	return timeout_ms;
}

/* write_epl_data()
 * Write EPL data into the given page. More than 128 bytes of data
 * can be written but that is only useful if the module has
 * CdbAutoPagingSupported=1.
 */
static int
write_epl_data(struct cmis_fw_update *fwu, uint8_t page, uint8_t *ptr, int length)
{
	int remain = length;
	int chunksize, rc;
	uint8_t addr;

	if ((rc = cmis_set_page(fwu->fh, page)) < 0) {
		error_out("Unable to set EPL page 0x%02x\n", page);
		return rc;
	}

	/* Start writing at byte 128 */
	addr = 128;
	while (remain > 0) {
		chunksize = remain < CDB_CHUNK_SIZE ? remain : CDB_CHUNK_SIZE;
		rc = cmis_write_raw_block(fwu->fh, addr, chunksize, ptr);
		if (rc < 0) {
			return rc;
		}
		remain -= chunksize;
		ptr += chunksize;
		addr += chunksize;
	}
	return 0;
}

/* cdb_get_fw_mgmt_features()
 * Use CDB_CMD_FW_MGMT_FEATURES to get the firmware managment features
 * from the device.
 */
static int
cdb_get_fw_mgmt_features(struct cmis_fw_update *fwu)
{
	struct cmis_cdb cdb;
	int rc;

	/* Send CDB_CMD_FW_MGMT_FEATURES (no payload) */
	if ((rc = send_cdb_command(fwu, &cdb, CDB_CMD_FW_MGMT_FEATURES, 0, 0)) < 0) {
		error_out("Sending CMD_FW_MGMT_FEATURES failed (rc=%d)\n", rc);
		return rc;
	}
	if ((rc = get_cdb_status(fwu, 500)) != CDB_STS_SUCCESS) {
		error_out("CMD_FW_MGMT_FEATURES failed (status=%d 0x%02x)\n", rc, (uint8_t)rc);
		if (rc >= 0)
			rc = -EIO;
		return rc;
	}
	if ((rc = get_cdb_response(fwu, &cdb)) < 0) {
		error_out("Failed to get cdb response (rc=%d)\n", rc);
		return rc;
	}
	memcpy((void *)&fwu->features, cdb.LocalPayload, sizeof(fwu->features));

	/* Fixup multibyte fields as needed */
	fwu->features.MaxDurationStart = be16toh(fwu->features.MaxDurationStart);
	fwu->features.MaxDurationAbort = be16toh(fwu->features.MaxDurationAbort);
	fwu->features.MaxDurationWrite = be16toh(fwu->features.MaxDurationWrite);
	fwu->features.MaxDurationComplete = be16toh(fwu->features.MaxDurationComplete);
	fwu->features.MaxDurationCopy = be16toh(fwu->features.MaxDurationCopy);
	debug_out("FW Management Features (CDB Command 0041h):\n");
	debug_out("-------------------------------------------\n");
	debug_out("Reserved (136):          0x%02x\n", fwu->features.Reserved136);
	debug_out("FWFeaturesFlags (137):   0x%02x\n", fwu->features.FWFeaturesFlags);
	debug_out("StartCommandPayloadSize: 0x%02x\n", fwu->features.StartCommandPayloadSize);
	debug_out("ErasedByte:              0x%02x\n", fwu->features.ErasedByte);
	debug_out("ReadWriteLengthExt:      0x%02x\n", fwu->features.ReadWriteLengthExt);
	debug_out("WriteMechanism:          0x%02x\n", fwu->features.WriteMechanism);
	debug_out("ReadMechanism:           0x%02x\n", fwu->features.ReadMechanism);
	debug_out("HitlessRestart:          0x%02x\n", fwu->features.HitlessRestart);
	debug_out("MaxDurationStart:        %u\n", fwu->features.MaxDurationStart);
	debug_out("MaxDurationAbort:        %u\n", fwu->features.MaxDurationAbort);
	debug_out("MaxDurationWrite:        %u\n", fwu->features.MaxDurationWrite);
	debug_out("MaxDurationComplete:     %u\n", fwu->features.MaxDurationComplete);
	debug_out("MaxDurationCopy:         %u\n", fwu->features.MaxDurationCopy);
	debug_out("\n");

	return 0;
}

/* cdb_get_fwinfo()
 * Use CDB_CMD_GET_FW_INFO to get the firmware bank A/B/Factory information
 * from the device.
 */
static int
cdb_get_fwinfo(struct cmis_fw_update *fwu, struct cmis_fw_info_payload *fwinfo)
{
	struct cmis_cdb cdb;
	int rc;

	memset(fwinfo, 0, sizeof(*fwinfo));

	/* Send CDB_CMD_GET_FW_INFO (no payload) */
	if ((rc = send_cdb_command(fwu, &cdb, CDB_CMD_GET_FW_INFO, 0, 0)) < 0) {
		error_out("Sending CMD_GET_FW_INFO failed (rc=%d)\n", rc);
		return rc;
	}
	if ((rc = get_cdb_status(fwu, 500)) != CDB_STS_SUCCESS) {
		error_out("CMD_GET_FW_INFO failed (status=%d 0x%02x)\n", rc, (uint8_t)rc);
		if (rc >= 0)
			rc = -EIO;
		return rc;
	}
	if ((rc = get_cdb_response(fwu, &cdb)) < 0) {
		error_out("Failed to get cdb response (rc=%d)\n", rc);
		return rc;
	}
	memcpy(fwinfo, cdb.LocalPayload, sizeof(*fwinfo));

	fwinfo->ImageABuildNumber = be16toh(fwinfo->ImageABuildNumber);
	fwinfo->ImageBBuildNumber = be16toh(fwinfo->ImageBBuildNumber);
	fwinfo->ImageFactoryBuildNumber = be16toh(fwinfo->ImageFactoryBuildNumber);

	return 0;
}

/* cdb_send_password()
 * Send CDB_CMD_ENTER_PASSWORD with provided 32-bit password value.
 */
static int
cdb_send_password(struct cmis_fw_update *fwu, uint32_t password)
{
	struct cmis_cdb cdb;
	int rc;

	/* Send CDB_CMD_START_FW_DOWNLOAD containing the total byte length (including header) */
	cdb.LocalPayload[0] = (password >> 24) & 0xFF;
	cdb.LocalPayload[1] = (password >> 16) & 0xFF;
	cdb.LocalPayload[2] = (password >>  8) & 0xFF;
	cdb.LocalPayload[3] = password & 0xFF;

	if ((rc = send_cdb_command(fwu, &cdb, CDB_CMD_ENTER_PASSWORD, 0, 4)) < 0) {
		error_out("Sending CDB_CMD_ENTER_PASSWORD failed (rc=%d)\n", rc);
		return rc;
	}
	if ((rc = get_cdb_status(fwu, 500)) != CDB_STS_SUCCESS) {
		error_out("CDB_CMD_ENTER_PASSWORD failed (status=%d 0x%02x)\n", rc, (uint8_t)rc);
		if (rc >= 0)
			rc = -EIO;
		return rc;
	}
	return 0;
}

/* cdb_send_start_firmware_download()
 * Send CDB_CMD_START_FW_DOWNLOAD with the header from the image.
 */
static int
cdb_send_start_firmware_download(struct cmis_fw_update *fwu)
{
	struct cmis_cdb cdb;
	uint8_t header_size = fwu->features.StartCommandPayloadSize;
	uint32_t length = fwu->length;
	uint8_t *ptr = fwu->buf;
	int timeout_ms = get_start_timeout_ms(fwu);
	int rc;

	if ((header_size == 0) || (header_size > CDB_FW_HEADER_SIZE_MAX)) {
		error_out("Illegal header size (%d)\n", header_size);
		return -EINVAL;
	}

	/* Send CDB_CMD_START_FW_DOWNLOAD containing the total byte length (including header) */
	cdb.LocalPayload[0] = (length >> 24) & 0xFF;
	cdb.LocalPayload[1] = (length >> 16) & 0xFF;
	cdb.LocalPayload[2] = (length >>  8) & 0xFF;
	cdb.LocalPayload[3] = length & 0xFF;
	cdb.LocalPayload[4] = 0; // Reserved
	cdb.LocalPayload[5] = 0; // Reserved
	cdb.LocalPayload[6] = 0; // Reserved
	cdb.LocalPayload[7] = 0; // Reserved

	memcpy(&cdb.LocalPayload[8], ptr, header_size);
	if ((rc = send_cdb_command(fwu, &cdb, CDB_CMD_START_FW_DOWNLOAD,
								0, 8 + header_size)) < 0) {
		error_out("Sending CMD_START_FW_DOWNLOAD failed (rc=%d)\n", rc);
		return rc;
	}
	if ((rc = get_cdb_status(fwu, timeout_ms)) != CDB_STS_SUCCESS) {
		error_out("CMD_START_FW_DOWNLOAD failed (status=%d 0x%02x)\n", rc, (uint8_t)rc);
		if (rc >= 0)
			rc = -EIO;
		return rc;
	}
	return 0;
}

/* cdb_send_firmware_via_epl()
 * Write the firmware image using EPL writes.
 */
static int
cdb_send_firmware_via_epl(struct cmis_fw_update *fwu)
{
	struct cmis_cdb cdb;
	uint8_t header_size = fwu->features.StartCommandPayloadSize;
	uint32_t length = fwu->length - header_size;
	uint8_t *ptr = fwu->buf + header_size;
	int timeout_ms = get_write_timeout_ms(fwu);
	int rc, remain;
	uint32_t addr;
	uint8_t epl_pages;

	/* How many EPL pages can we use? */
	epl_pages = fwu->adv.p01h_163 & CDB_NUM_EPL_PAGES_MASK;
	switch (epl_pages) {
	case 5:
		epl_pages = 8;
		break;
	case 6:
		epl_pages = 12;
		break;
	case 7:
		epl_pages = 16;
		break;
	default:
		/* Others are 1-to-1 */
		break;
	}
	if (epl_pages == 0) {
		error_out("Number of EPL pages supported is zero\n");
		return -ENOTSUP;
	}

	addr = 0;
	remain = length;
	while (remain > 0) {
		int size = 0;
		uint8_t page;

		/* Load data into each supported EPL page (0xa0 - 0xaf) */
		for (page = 0xa0; page < (0xa0 + epl_pages); page++) {
			int chunksize = remain;

			if (chunksize > 128)
				chunksize = 128;
			rc = write_epl_data(fwu, page, ptr, chunksize);
			if (rc < 0) {
				error_out("Writing EPL page 0x%02x failed (rc=%d)\n", page, rc);
				return rc;
			}
			ptr += chunksize;
			size += chunksize;
			remain -= chunksize;
		}

		/* Send CDB_CMD_WRITE_FW_EPL starting with the address */
		cdb.LocalPayload[0] = (addr >> 24) & 0xFF;
		cdb.LocalPayload[1] = (addr >> 16) & 0xFF;
		cdb.LocalPayload[2] = (addr >>  8) & 0xFF;
		cdb.LocalPayload[3] = addr & 0xFF;
		if ((rc = send_cdb_command(fwu, &cdb, CDB_CMD_WRITE_FW_EPL,
									size, 4)) < 0) {
			error_out("Sending CMD_WRITE_FW_EPL at address=0x%x failed (rc=%d)\n",
					addr, rc);
			return rc;
		}
		if ((rc = get_cdb_status(fwu, timeout_ms)) != CDB_STS_SUCCESS) {
			error_out("CMD_WRITE_FW_EPL at address=0x%x failed (status=%d)\n",
					addr, rc);
			if (rc >= 0)
				rc = -EIO;
			return rc;
		}
		debug_out("Loaded %d EPL bytes across %d pages targeting addr 0x%06x\n",
					size, epl_pages, addr);
		addr += size;
	}
	return 0;
}

/* cdb_send_firmware_via_lpl()
 * Write the firmware image using LPL writes.
 */
static int
cdb_send_firmware_via_lpl(struct cmis_fw_update *fwu)
{
	struct cmis_cdb cdb;
	uint8_t header_size = fwu->features.StartCommandPayloadSize;
	uint32_t remain = fwu->length - header_size;
	uint8_t *ptr = fwu->buf + header_size;
	int timeout_ms = get_write_timeout_ms(fwu);
	uint32_t addr;
	int rc;

	addr = 0;
	while (remain > 0) {
		int size;

		/* We can put CDB_LPL_MAXLEN bytes in each payload but the first
		 * 4 of them are always the address.
		 */
		if (remain < (CDB_LPL_MAXLEN - 4))
			size = remain;
		else
			size = (CDB_LPL_MAXLEN - 4);

		/* Send CDB_CMD_WRITE_FW_LPL starting with the address */
		cdb.LocalPayload[0] = (addr >> 24) & 0xFF;
		cdb.LocalPayload[1] = (addr >> 16) & 0xFF;
		cdb.LocalPayload[2] = (addr >>  8) & 0xFF;
		cdb.LocalPayload[3] = addr & 0xFF;
		memcpy(&cdb.LocalPayload[4], ptr, size);
		if ((rc = send_cdb_command(fwu, &cdb, CDB_CMD_WRITE_FW_LPL,
									0, 4 + size)) < 0) {
			error_out("Sending CMD_WRITE_FW_LPL at address=0x%x failed (rc=%d)\n",
					addr, rc);
			return rc;
		}
		if ((rc = get_cdb_status(fwu, timeout_ms)) != CDB_STS_SUCCESS) {
			error_out("CMD_WRITE_FW_LPL at address=0x%x failed (status=%d)\n",
					addr, rc);
			if (rc >= 0)
				rc = -EIO;
			return rc;
		}
		debug_out("Loaded %d LPL bytes targeting addr 0x%06x\n", size, addr);
		ptr += size;
		addr += size;
		remain -= size;
	}
	return 0;
}

/* cdb_send_run_firmware_command()
 * Send the RUN_FW_IMAGE command with the given run_option and delay_ms.
 */
static int
cdb_send_run_firmware_command(struct cmis_fw_update *fwu, uint8_t run_option, uint16_t delay_ms)
{
	struct cmis_cdb cdb;
	int timeout_ms = delay_ms;
	int rc;

	/* Send CDB_CMD_RUN_FW_IMAGE */
	cdb.LocalPayload[0] = 0;    /* Reserved */
	cdb.LocalPayload[1] = run_option;
	cdb.LocalPayload[2] = (delay_ms >> 8) & 0xFF;
	cdb.LocalPayload[3] = delay_ms & 0xFF;
	if ((rc = send_cdb_command(fwu, &cdb, CDB_CMD_RUN_FW_IMAGE, 0, 4)) < 0) {
		error_out("Sending CMD_RUN_FW_IMAGE failed (rc=%d)\n", rc);
		return rc;
	}

	if ((rc = get_cdb_status(fwu, 2000)) != CDB_STS_SUCCESS) {
		error_out("CMD_RUN_FW_IMAGE failed (status=%d 0x%02x)\n", rc, (uint8_t)rc);
		if (rc >= 0)
			rc = -EIO;

		/* If they did not supply a timeout, don't consider timeout a failure */
		if (timeout_ms || (rc != -ETIMEDOUT)) {
			return rc;
		}
	}
	return 0;
}

/* dsp_send_start_firmware_download()
 * TE AOC DSP Proprietary
 * Send CDB_CMD_START_FW_DOWNLOAD+0x9000 with the header from the image.
 */
static int
dsp_send_start_firmware_download(struct cmis_fw_update *fwu)
{
	struct cmis_cdb cdb;
	uint8_t header_size = fwu->features.StartCommandPayloadSize;
	uint32_t length = fwu->length;
	uint8_t *ptr = fwu->buf;
	int timeout_ms = get_start_timeout_ms(fwu);
	int rc;

	if ((header_size == 0) || (header_size > CDB_FW_HEADER_SIZE_MAX)) {
		error_out("Illegal header size (%d)\n", header_size);
		return -EINVAL;
	}

	/* Send CDB_CMD_START_FW_DOWNLOAD+0x9000 containing the total byte length (including header) */
	cdb.LocalPayload[0] = (length >> 24) & 0xFF;
	cdb.LocalPayload[1] = (length >> 16) & 0xFF;
	cdb.LocalPayload[2] = (length >>  8) & 0xFF;
	cdb.LocalPayload[3] = length & 0xFF;
	cdb.LocalPayload[4] = 0; // Reserved
	cdb.LocalPayload[5] = 0; // Reserved
	cdb.LocalPayload[6] = 0; // Reserved
	cdb.LocalPayload[7] = 0; // Reserved

	memcpy(&cdb.LocalPayload[8], ptr, header_size);
	if ((rc = send_cdb_command(fwu, &cdb, CDB_CMD_START_FW_DOWNLOAD + 0x9000,
								0, 8 + header_size)) < 0) {
		error_out("Sending DSP CMD_START_FW_DOWNLOAD failed (rc=%d)\n", rc);
		return rc;
	}
	if ((rc = get_cdb_status(fwu, timeout_ms)) != CDB_STS_SUCCESS) {
		error_out("DSP CMD_START_FW_DOWNLOAD failed (status=%d 0x%02x)\n", rc, (uint8_t)rc);
		if (rc >= 0)
			rc = -EIO;
		return rc;
	}
	return 0;
}

/* dsp_send_firmware_via_epl()
 * TE AOC DSP Proprietary
 * Write the DSP firmware image using EPL writes.
 */
static int
dsp_send_firmware_via_epl(struct cmis_fw_update *fwu)
{
	struct cmis_cdb cdb;
	uint8_t header_size = fwu->features.StartCommandPayloadSize;
	uint32_t length = fwu->length - header_size;
	uint8_t *ptr = fwu->buf + header_size;
	int timeout_ms = get_write_timeout_ms(fwu);
	int rc, remain;
	uint32_t addr;
	uint8_t epl_pages;

	/* How many EPL pages can we use? */
	epl_pages = fwu->adv.p01h_163 & CDB_NUM_EPL_PAGES_MASK;
	switch (epl_pages) {
	case 5:
		epl_pages = 8;
		break;
	case 6:
		epl_pages = 12;
		break;
	case 7:
		epl_pages = 16;
		break;
	default:
		/* Others are 1-to-1 */
		break;
	}
	if (epl_pages == 0) {
		error_out("Device does not support EPL pages\n");
		return -ENOTSUP;
	}

	addr = 0;
	remain = length;
	while (remain > 0) {
		int size = 0;
		uint8_t page;

		/* Load data into each supported EPL page (0xa0 - 0xaf) */
		for (page = 0xa0; page < (0xa0 + epl_pages); page++) {
			int chunksize = remain;

			if (chunksize > 128)
				chunksize = 128;
			rc = write_epl_data(fwu, page, ptr, chunksize);
			if (rc < 0) {
				error_out("Writing EPL page 0x%02x failed (rc=%d)\n", page, rc);
				return rc;
			}
			ptr += chunksize;
			size += chunksize;
			remain -= chunksize;
		}

		/* Send CDB_CMD_WRITE_FW_EPL + 0x9000 starting with the address */
		cdb.LocalPayload[0] = (addr >> 24) & 0xFF;
		cdb.LocalPayload[1] = (addr >> 16) & 0xFF;
		cdb.LocalPayload[2] = (addr >>  8) & 0xFF;
		cdb.LocalPayload[3] = addr & 0xFF;
		if ((rc = send_cdb_command(fwu, &cdb, CDB_CMD_WRITE_FW_EPL + 0x9000,
									size, 4)) < 0) {
			error_out("Sending DSP CMD_WRITE_FW_EPL at address=0x%x failed (rc=%d)\n",
					addr, rc);
			return rc;
		}
		if ((rc = get_cdb_status(fwu, timeout_ms)) != CDB_STS_SUCCESS) {
			error_out("DSP CMD_WRITE_FW_EPL at address=0x%x failed (status=%d)\n",
					addr, rc);
			if (rc >= 0)
				rc = -EIO;
			return rc;
		}
		debug_out("Loaded %d DSP EPL bytes across %d pages targeting addr 0x%06x\n",
					size, epl_pages, addr);
		addr += size;
	}
	return 0;
}

/* te_aoc_dsp_firmware_flash()
 * TE AOC DSP Proprietary
 * @brief Flash a TE DSP firmware image to a module via proprietary method.
 *        See ASICFWDG-5046.
 */
static int
te_aoc_dsp_firmware_flash(struct cmis_fw_update *fwu)
{
	struct cmis_cdb cdb;
	int timeout_ms, rc;

	if (!fwu->buf) {
		error_out("BUG: no fwu->buf!\n");
		return -EINVAL;
	}
	if (fwu->length < 0x40) {
		error_out("Only read %u bytes. Invalid file.\n", fwu->length);
		return -EINVAL;
	}

	print_out("Programming %d DSP bytes...\n", fwu->length);

	/* Our first ColorChip AOCs advertised the wrong MaxDurationStart */
	if (get_start_timeout_ms(fwu) < 3000) {
		fwu->features.MaxDurationStart = 3000;
		error_out("Using MaxDurationStart override of %dms\n",
				get_start_timeout_ms(fwu));
	}

	/* CMD0101h + 9000h Start Firmware Download */
	rc = dsp_send_start_firmware_download(fwu);
	if (rc < 0) {
		return rc;
	}

	/* Only EPL writes are supported */
	print_out("Using EPL firmware loading\n");
	/* CMD0104h + 9000h Write Firmware Block EPL */
	rc = dsp_send_firmware_via_epl(fwu);
	if (rc < 0) {
		return rc;
	}

	/* CMD0107h + 9000h Complete Firmware Download (no payload) */
	if ((rc = send_cdb_command(fwu, &cdb, CDB_CMD_COMPLETE_FW_DOWNLOAD + 0x9000, 0, 0)) < 0) {
		error_out("Sending DSP CMD_COMPLETE_FW_DOWNLOAD failed (rc=%d)\n", rc);
		return rc;
	}
	timeout_ms = get_complete_timeout_ms(fwu);
	if ((rc = get_cdb_status(fwu, timeout_ms)) != CDB_STS_SUCCESS) {
		error_out("DSP CMD_COMPLETE_FW_DOWNLOAD failed (status=%d 0x%02x)\n", rc, (uint8_t)rc);
		return rc;
	}

	/* Delay for good measure */
	sleep(3);

	print_out("DSP firmware programming complete. Manual reset required to activate.\n");
	return 0;
}

/* cdb_fw_flash()
 * @brief Flash a CDB CMIS firmware image to a module.
 */
static int
cdb_fw_flash(struct cmis_fw_update *fwu)
{
	struct cmis_fw_info_payload fwinfo;
	struct cmis_cdb cdb;
	int timeout_ms, retry;
	char running_at_start;
	int rc;

	if (!fwu->buf) {
		error_out("BUG: no fwu->buf!\n");
		return -EINVAL;
	}
	if (fwu->length < 0x40) {
		error_out("Only read %u bytes. Invalid file.\n", fwu->length);
		return -EINVAL;
	}

	/* What bank of firmware is running? */
	running_at_start = '?';
	rc = cdb_get_fwinfo(fwu, &fwinfo);
	if (rc) {
		error_out("Warning: Failed to query existing fw info (rc=%d)\n", rc);
	} else {
		if (fwinfo.FWStatusFlags & CDB_FW_IMG_A_RUNNING)
			running_at_start = 'A';
		else if (fwinfo.FWStatusFlags & CDB_FW_IMG_B_RUNNING)
			running_at_start = 'B';
	}

	/* Our first ColorChip AOCs advertised the wrong MaxDurationStart */
	if (!strncmp((char *)fwu->buf, "Color-Chip", 10)) {
		if (get_start_timeout_ms(fwu) < 3000) {
			fwu->features.MaxDurationStart = 3000;
			print_out("Using MaxDurationStart override of %dms\n",
					get_start_timeout_ms(fwu));
		}
	}

	/* Some Molex devices advertise the wrong MaxDurationWrite */
	if (strstr(fwu->vendor, CMIS_VENDOR_MOLEX)) {
		if (get_write_timeout_ms(fwu) < 100) {
			fwu->features.MaxDurationWrite = 100;
			print_out("Using MaxDurationWrite override of %dms\n",
					get_write_timeout_ms(fwu));
		}
	}

	/* Some Vendors might require CDB password entry. Typically evident
	 * if a CDB request fails with CDB Status = 70 (0x46)
	 */
	if (fwu->password != 0) {
		/* CMD0001h Enter Password */
		rc = cdb_send_password(fwu, fwu->password);
		if (rc < 0) {
			print_out("Password not accepted, continuing anyway\n");
		}
	}

	print_out("Programming %u bytes...\n", fwu->length);

	/* CMD0101h Start Firmware Download */
	rc = cdb_send_start_firmware_download(fwu);
	if (rc < 0) {
		return rc;
	}

	/* If EPL writes are supported, use EPL */
	if ((fwu->features.WriteMechanism & CDB_FW_WRITE_EPL_SUPPORTED) &&
		((fwu->adv.p01h_163 & CDB_NUM_EPL_PAGES_MASK) != 0)) {
		print_out("Using EPL firmware loading\n");
		/* CMD0104h Write Firmware Block EPL */
		rc = cdb_send_firmware_via_epl(fwu);
	} else {
		print_out("Using LPL firmware loading\n");
		/* CMD0103h Write Firmware Block LPL */
		rc = cdb_send_firmware_via_lpl(fwu);
	}
	if (rc < 0) {
		return rc;
	}

	/* CMD0107h Complete Firmware Download (no payload) */
	if ((rc = send_cdb_command(fwu, &cdb, CDB_CMD_COMPLETE_FW_DOWNLOAD, 0, 0)) < 0) {
		error_out("Sending CMD_COMPLETE_FW_DOWNLOAD failed (rc=%d)\n", rc);
		return rc;
	}
	timeout_ms = get_complete_timeout_ms(fwu);
	if ((rc = get_cdb_status(fwu, timeout_ms)) != CDB_STS_SUCCESS) {
		error_out("CMD_COMPLETE_FW_DOWNLOAD failed (status=%d 0x%02x)\n", rc, (uint8_t)rc);
		return rc;
	}

	/* Delay for good measure */
	sleep(1);

	/* Try to run the new (inactive) firmware with a supplied delay */
	print_out("Running new firmware image...\n");
	/* CMD0109h Run Firmware Image */
	if ((rc = cdb_send_run_firmware_command(fwu,
				CDB_TRAFFIC_AFF_RUN_INACTIVE_IMG, 200)) < 0) {
		error_out("Running new firmware failed (rc=%d)\n", rc);
		return rc;
	}

	/* Delay for module to restart */
	sleep(3);

	/* If the new firmware fails to run, the following commands might fail and
	 * the module might need to be reset or power cycled (by other means) to
	 * return to the previous firmware version.
	 */
	for (retry = 0; retry < 5; retry++) {
		rc = cdb_get_fwinfo(fwu, &fwinfo);
		if (rc) {
			/* The previous delay might not be adequate for some vendors
			 * so we will loop here to give it more time.
			 */
			sleep(1);
			continue;
		}
		break;
	}
	if (rc) {
		error_out("Failed to query updated fw info (rc=%d)\n", rc);
		return rc;
	}

	/* Which firmware is running now? */
	if (((fwinfo.FWStatusFlags & CDB_FW_IMG_A_RUNNING) &&
				(running_at_start == 'A')) ||
			   ((fwinfo.FWStatusFlags & CDB_FW_IMG_B_RUNNING) &&
				(running_at_start == 'B'))) {
		/* Hisense CDB FW update observation: Abnormal A/B image reporting.
		 * Firmware always appears to be in image 'A'. No CMD_COMMIT_FW_IMAGE
		 * is required - just CDB_CMD_RUN_FW_IMAGE or reset the module.
		 */
		if (strstr(fwu->vendor, CMIS_VENDOR_HISENSE)) {
			print_out("Firmware image %c is (still) running for vendor %s. "
					"Check version number to confirm upgrade result.\n",
					running_at_start, fwu->vendor);
		} else {
			error_out("Previous firmware image %c is still running! Update aborted.\n",
					running_at_start);
			return -EINVAL;
		}
	}

	/* Some Vendors require CDB password entry (after reset) */
	if (fwu->password != 0) {
		rc = cdb_send_password(fwu, fwu->password);
		if (rc < 0) {
			error_out("Password not accepted, continuing anyway\n");
		}
	}

	/* CMD010Ah Commit Firmware Image (no payload) */
	print_out("Committing new firmware image...\n");
	if ((rc = send_cdb_command(fwu, &cdb, CDB_CMD_COMMIT_FW_IMAGE, 0, 0)) < 0) {
		error_out("Sending CMD_COMMIT_FW_IMAGE failed (rc=%d)\n", rc);
		return rc;
	}
	/* Have observed 4 seconds (TE Connectivity). Timeout is likely harmless */
	timeout_ms = 10000;
	if ((rc = get_cdb_status(fwu, timeout_ms)) != CDB_STS_SUCCESS) {
		error_out("Warning: CMD_COMMIT_FW_IMAGE failed (status=%d 0x%02x)\n", rc, (uint8_t)rc);
		/* Timeout here can likely be ignored */
		if (rc != -ETIMEDOUT) {
			return rc;
		}
	}

	print_out("Firmware programming complete\n");
	return 0;
}

/* cdb_fw_revert()
 * @brief Revert to the idle CDB firmware image. This is a subset of the
 *        full firmware flash procedure - see cdb_fw_flash().
 */
static int
cdb_fw_revert(struct cmis_fw_update *fwu)
{
	struct cmis_fw_info_payload fwinfo;
	struct cmis_cdb cdb;
	int timeout_ms, retry;
	char running_at_start;
	int rc;

	/* What bank of firmware is running? */
	running_at_start = '?';
	rc = cdb_get_fwinfo(fwu, &fwinfo);
	if (rc) {
		error_out("Warning: Failed to query existing fw info (rc=%d)\n", rc);
	} else {
		if (fwinfo.FWStatusFlags & CDB_FW_IMG_A_RUNNING)
			running_at_start = 'A';
		else if (fwinfo.FWStatusFlags & CDB_FW_IMG_B_RUNNING)
			running_at_start = 'B';
	}

	/* Some Molex devices advertise the wrong MaxDurationWrite */
	if (strstr(fwu->vendor, CMIS_VENDOR_MOLEX)) {
		if (get_write_timeout_ms(fwu) < 100) {
			fwu->features.MaxDurationWrite = 100;
			print_out("Using MaxDurationWrite override of %dms\n",
					get_write_timeout_ms(fwu));
		}
	}

	/* Some Vendors might require CDB password entry. Typically evident
	 * if a CDB request fails with CDB Status = 70 (0x46)
	 */
	if (fwu->password != 0) {
		/* CMD0001h Enter Password */
		rc = cdb_send_password(fwu, fwu->password);
		if (rc < 0) {
			print_out("Password not accepted, continuing anyway\n");
		}
	}

	/* Try to run the old (inactive) firmware with a supplied delay */
	print_out("Image %c is running. Attempting to switch...\n", running_at_start);
	/* CMD0109h Run Firmware Image */
	if ((rc = cdb_send_run_firmware_command(fwu,
				CDB_TRAFFIC_AFF_RUN_INACTIVE_IMG, 200)) < 0) {
		error_out("Running old firmware failed (rc=%d)\n", rc);
		return rc;
	}

	/* Delay for module to restart */
	sleep(3);

	/* If the new firmware fails to run, the following commands might fail and
	 * the module might need to be reset or power cycled (by other means) to
	 * return to the previous firmware version.
	 */
	for (retry = 0; retry < 5; retry++) {
		rc = cdb_get_fwinfo(fwu, &fwinfo);
		if (rc) {
			/* The previous delay might not be adequate for some vendors
			 * so we will loop here to give it more time.
			 */
			sleep(1);
			continue;
		}
		break;
	}
	if (rc) {
		error_out("Failed to query updated fw info (rc=%d)\n", rc);
		return rc;
	}

	/* Which firmware is running now? */
	if (((fwinfo.FWStatusFlags & CDB_FW_IMG_A_RUNNING) &&
				(running_at_start == 'A')) ||
			   ((fwinfo.FWStatusFlags & CDB_FW_IMG_B_RUNNING) &&
				(running_at_start == 'B'))) {
		/* Hisense CDB FW update observation: Abnormal A/B image reporting.
		 * Firmware always appears to be in image 'A'. No CMD_COMMIT_FW_IMAGE
		 * is required - just CDB_CMD_RUN_FW_IMAGE or reset the module.
		 */
		if (strstr(fwu->vendor, CMIS_VENDOR_HISENSE)) {
			print_out("Firmware image %c is (still) running for vendor %s. "
					"Check version number to confirm upgrade result.\n",
					running_at_start, fwu->vendor);
		} else {
			error_out("Firmware image %c is still running! Update aborted.\n",
					running_at_start);
			return -EINVAL;
		}
	}

	/* Some Vendors require CDB password entry (after reset) */
	if (fwu->password != 0) {
		rc = cdb_send_password(fwu, fwu->password);
		if (rc < 0) {
			error_out("Password not accepted, continuing anyway\n");
		}
	}

	/* CMD010Ah Commit Firmware Image (no payload) */
	print_out("Committing reverted firmware image...\n");
	if ((rc = send_cdb_command(fwu, &cdb, CDB_CMD_COMMIT_FW_IMAGE, 0, 0)) < 0) {
		error_out("Sending CMD_COMMIT_FW_IMAGE failed (rc=%d)\n", rc);
		return rc;
	}
	/* Have observed 4 seconds (TE Connectivity). Timeout is likely harmless */
	timeout_ms = 10000;
	if ((rc = get_cdb_status(fwu, timeout_ms)) != CDB_STS_SUCCESS) {
		error_out("Warning: CMD_COMMIT_FW_IMAGE failed (status=%d 0x%02x)\n", rc, (uint8_t)rc);
		/* Timeout here can likely be ignored */
		if (rc != -ETIMEDOUT) {
			return rc;
		}
	}

	print_out("Firmware revert complete\n");
	return 0;
}

/* cdb_fw_file_import()
 * @brief Read the firmware file into our buffer.
 */
static int
cdb_fw_file_import(struct cmis_fw_update *fwu)
{
	FILE *fp;
	struct stat st;
	size_t readlen;

	fp = fopen(fwu->fw_filename, "rb");
	if (!fp) {
		error_out("Failed to open firmware file %s: %s\n",
				fwu->fw_filename, strerror(errno));
		return -errno;
	}
	if (fstat(fileno(fp), &st) < 0) {
		error_out("Failed to stat firmware file %s: %s\n",
				fwu->fw_filename, strerror(errno));
		fclose(fp);
		return -errno;
	}
	if (st.st_size > FW_BUF_MALLOC_MAXSIZE) {
		error_out("Firmware file %s too large (%u bytes > %u bytes max)\n",
				fwu->fw_filename, (unsigned int)st.st_size, FW_BUF_MALLOC_MAXSIZE);
		fclose(fp);
		return -EFBIG;
	}

	fwu->length = (uint32_t)st.st_size;
	fwu->buf = malloc(fwu->length);
	if (!fwu->buf) {
		error_out("Failed to allocate %u bytes for firmware file %s\n",
				fwu->length, fwu->fw_filename);
		fclose(fp);
		return -ENOMEM;
	}

	readlen = fread(fwu->buf, 1, fwu->length, fp);
	if (readlen != fwu->length) {
		error_out("Failed to read firmware file %s: %s\n",
				fwu->fw_filename, strerror(errno));
		free(fwu->buf);
		fwu->length = 0;
		fclose(fp);
		return -EIO;
	}
	fclose(fp);
	return 0;
}

/* display_firmware_info()
 * @brief Display firmware information for a CMIS device that supports CDB
 */
static int
display_firmware_info(struct cmis_fw_update *fwu)
{
	const char *status_a, *status_a_boot, *status_b, *status_b_boot;
	char extra[33];
	struct cmis_fw_info_payload fwinfo;
	int i, rc;

	rc = cdb_get_fwinfo(fwu, &fwinfo);
	if (rc) {
		return rc;
	}
	debug_out("FWStatusFlags=0x%02x\n", fwinfo.FWStatusFlags);
	debug_out("FWInfoPresentFlags=0x%02x\n", fwinfo.FWInfoPresentFlags);

	/* Assume default strings and update as required */
	status_a = "idle";
	status_a_boot = "-";
	if (fwinfo.FWStatusFlags & CDB_FW_IMG_A_RSVD) {
		status_a = "reserved";
	} else {
		if (fwinfo.FWStatusFlags & CDB_FW_IMG_A_EMPTY)
			status_a = "empty";
		else if (fwinfo.FWStatusFlags & CDB_FW_IMG_A_RUNNING)
			status_a = "running";
		if (fwinfo.FWStatusFlags & CDB_FW_IMG_A_COMMITED_TO_BOOT)
			status_a_boot = "boot";
	}
	status_b = "idle";
	status_b_boot = "-";
	if (fwinfo.FWStatusFlags & CDB_FW_IMG_B_RSVD) {
		status_b = "reserved";
	} else {
		if (fwinfo.FWStatusFlags & CDB_FW_IMG_B_EMPTY)
			status_b = "empty";
		else if (fwinfo.FWStatusFlags & CDB_FW_IMG_B_RUNNING)
			status_b = "running";
		if (fwinfo.FWStatusFlags & CDB_FW_IMG_B_COMMITED_TO_BOOT)
			status_b_boot = "boot";
	}

	/* Output Image A information */
	if (fwinfo.FWInfoPresentFlags & CDB_FW_IMG_A_PRESENT) {
		memcpy(extra, fwinfo.ImageAExtra, 32);
		extra[32] = '\0';
		for (i = 31; (i >= 0) && ((extra[i] <= ' ') || (extra[i] >= 0x7F)); i--)
			extra[i] = '\0';
		error_out("Image A: %-10s %-5s %u.%u.%u %s\n",
					status_a, status_a_boot, fwinfo.ImageAMajor, fwinfo.ImageAMinor,
					fwinfo.ImageABuildNumber, extra);
	} else {
		error_out("Image A: %-10s %-5s\n", status_a, status_a_boot);
	}

	/* Output Image B information */
	if (fwinfo.FWInfoPresentFlags & CDB_FW_IMG_B_PRESENT) {
		memcpy(extra, fwinfo.ImageBExtra, 32);
		extra[32] = '\0';
		for (i = 31; (i >= 0) && ((extra[i] <= ' ') || (extra[i] >= 0x7F)); i--)
			extra[i] = '\0';
		error_out("Image B: %-10s %-5s %u.%u.%u %s\n",
					status_b, status_b_boot, fwinfo.ImageBMajor, fwinfo.ImageBMinor,
					fwinfo.ImageBBuildNumber, extra);
	} else {
		error_out("Image B: %-10s %-5s\n", status_b, status_b_boot);
	}

	/* Output Factory Image information */
	if (fwinfo.FWInfoPresentFlags & CDB_FW_IMG_FACTORY_PRESENT) {
		memcpy(extra, fwinfo.ImageFactoryExtra, 32);
		extra[32] = '\0';
		for (i = 31; (i >= 0) && ((extra[i] <= ' ') || (extra[i] >= 0x7F)); i--)
			extra[i] = '\0';
		error_out("Image Factory: %u.%u.%u %s\n",
					fwinfo.ImageFactoryMajor, fwinfo.ImageFactoryMinor,
					fwinfo.ImageFactoryBuildNumber, extra);
	}
	return 0;
}

/* cmis_query_device()
 * @brief Query some basic information from a CMIS device.
 */
static int
cmis_query_device(struct cmis_fw_update *fwu)
{
	int rc = 0, final_rc = 0;

	rc = cmis_read_byte(fwu->fh, 0, 0, &(fwu->sff8024_identifier));
	if (rc < 0) {
		error_out("Failed to read SFF-8024 Identifier (rc=%d)\n", rc);
		return rc;
	}
	rc = cmis_read_byte(fwu->fh, 0, 1, &(fwu->cmis_revision));
	if (rc < 0) {
		error_out("Failed to read CMIS Revision (rc=%d)\n", rc);
		return rc;
	}

	if (fwu->sff8024_identifier < SFF8024_TYPE_QSFPDD) {
		error_out("Warning: Unexpected SFF-8024 device identifier 0x%02X\n",
				fwu->sff8024_identifier);
	}
	if (fwu->cmis_revision < 0x30) {
		error_out("Managment interface revision 0x%02X is not CMIS compatible\n",
				fwu->cmis_revision);
		return -ENOTSUP;
	}
	if (fwu->cmis_revision < 0x40) {
		error_out("Warning: Unexpected CMIS revision 0x%02X\n",
				fwu->cmis_revision);
	}

	/* Capture some identification */
	rc = cmis_get_vendor(fwu->fh, sizeof(fwu->vendor), fwu->vendor);
	if (rc) {
		error_out("Failed to read device vendor (rc=%d)\n", rc);
		snprintf(fwu->vendor, sizeof(fwu->vendor), "Unknown");
		if (final_rc == 0)
			final_rc = rc;
	}
	str_trim_whitespace_r(fwu->vendor);

	rc = cmis_get_part_num(fwu->fh, sizeof(fwu->part_num), fwu->part_num);
	if (rc) {
		error_out("Failed to read device part number (rc=%d)\n", rc);
		snprintf(fwu->part_num, sizeof(fwu->part_num), "Unknown");
		if (final_rc == 0)
			final_rc = rc;
	}
	str_trim_whitespace_r(fwu->part_num);

	rc = cmis_get_serial_num(fwu->fh, sizeof(fwu->serial_num), fwu->serial_num);
	if (rc) {
		error_out("Failed to read device serial number (rc=%d)\n", rc);
		if (final_rc == 0)
			final_rc = rc;
	}
	str_trim_whitespace_r(fwu->serial_num);

	rc = cmis_get_oem(fwu->fh, sizeof(fwu->oem), fwu->oem);
	if (rc) {
		error_out("Failed to read device OEM info (rc=%d)\n", rc);
		if (final_rc == 0)
			final_rc = rc;
	}
	str_trim_whitespace_r(fwu->oem);

	rc = cmis_get_module_rev(fwu->fh, sizeof(fwu->module_revision), fwu->module_revision);
	if (rc) {
		error_out("Failed to read module revision (rc=%d)\n", rc);
		if (final_rc == 0)
			final_rc = rc;
	}
	str_trim_whitespace_r(fwu->module_revision);

	rc = cmis_get_active_fw_rev(fwu->fh, &fwu->active_fw_rev);
	if (rc) {
		error_out("Failed to read active firmware revision (rc=%d)\n", rc);
		if (final_rc == 0)
			final_rc = rc;
	}

	/* Try to read inactive fw rev and hardware rev (not all devices support these) */
	rc = cmis_get_inactive_fw_rev(fwu->fh, &fwu->inactive_fw_rev);
	if (rc) {
		debug_out("Failed to read inactive firmware revision (rc=%d)\n", rc);
	}
	rc = cmis_get_hardware_rev(fwu->fh, &fwu->hardware_rev);
	if (rc) {
		debug_out("Failed to read hardware revision (rc=%d)\n", rc);
	}

	return final_rc;
}

/* cdb_query_device()
 * @brief Query some basic information from a CMIS device that supports CDB
 */
static int
cdb_query_device(struct cmis_fw_update *fwu)
{
	int rc = 0;
	uint8_t byte02h;

	rc = cmis_read_byte(fwu->fh, 0, 2, &(byte02h));
	if (rc < 0) {
		error_out("Failed to read Byte 02h (rc=%d)\n", rc);
		return rc;
	}

	if (byte02h & CMIS_FLAT) {
		error_out("Device does not support paging. CDB protocol is not possible.\n");
		return -ENOTSUP;
	}

	/* Get some basic information about the CDB capabilities */
	rc = get_cdb_advertisement(fwu);
	if (rc < 0) {
		error_out("Failed to read CDB advertisement (rc=%d)\n", rc);
		return rc;
	}
	if ((fwu->adv.p01h_163 & CDB_IMPLEMENTED_MASK) == CDB_IMPLEMENTED_NONE) {
		error_out("Device does not advertise CDB implemented\n");
		return -ENOTSUP;
	}

	/* Get CDB firmware management features */
	rc = cdb_get_fw_mgmt_features(fwu);
	if (rc) {
		error_out("Failed to get fw management features (rc=%d)\n", rc);
		return rc;
	}

	return rc;
}

/* Display usage information
 */
void help(const char *path)
{
	printf("%s Version %s\n", path, APPLICATION_VERSION);
	printf("CMIS CDB Firmware Update Utility for Slingshot NICs\n");
	printf("Usage: %s [options]\n", path);
	printf("       %s -d <dev>\n", path);
	printf("       %s -d <dev> -f <filename>\n", path);
	printf("\n");
	printf(" -h, --help                 Display this help output\n");
	printf(" -d, --device <dev>         Device name (default: \"cxi0\")\n");
	printf(" -f, --fwfile <filename>    Firmware file to use for the update\n");
	printf(" -p, --password <password>  CMIS device/vendor password (optional)\n");
	printf(" -r, --revert               Revert to the previous CDB firmware image\n");
	printf(" -D, --dsp                  Perform a DSP firmware update (not typical)\n");
	printf(" -n, --no-reset             Do not reset the device before the update\n");
	printf("     --no-lockout           Do not lockout the link driver\n");
	printf(" -v, --verbose              Enable verbose output to STDOUT\n");
	printf(" -V, --version              Display the utility version and exit\n");
	printf("\n");
	printf("If no firmware file is specified and no other firmware action is specified,\n");
	printf("information about the current firmware images will be shown.\n");
	printf("\n");
}

int main(int argc, char *argv[])
{
	int rc = 0;
	int report = 0, dsp_update = 0, revert = 0, no_reset = 0,
	    no_lockout = 0;
	struct cmis_fw_update cmis_fwu;
	struct cmis_fw_update *fwu = &cmis_fwu;
	enum {
		OPT_NO_LOCKOUT = 0x80,
	};

	static struct option param_opts[] = {
		{"device",     required_argument, NULL, 'd' },
		{"fwfile",     required_argument, NULL, 'f' },
		{"help",       no_argument,       NULL, 'h' },
		{"password",   required_argument, NULL, 'p' },
		{"revert",     no_argument,       NULL, 'r' },
		{"no-reset",   no_argument,       NULL, 'n' },
		{"verbose",    no_argument,       NULL, 'v' },
		{"dsp",        no_argument,       NULL, 'D' },
		{"version",    no_argument,       NULL, 'V' },
		{"no-lockout", no_argument,       NULL, OPT_NO_LOCKOUT },
		{NULL,         0,                 NULL, 0   }
	};

	memset(fwu, 0, sizeof(struct cmis_fw_update));
	fwu->device = DEFAULT_DEVICE_NAME;

	while (1) {
		int tmp;
		int optidx = 0;

		tmp = getopt_long(argc, argv, "d:f:hrp:nvDV", param_opts, &optidx);
		if (tmp < 0) {
			break;
		}

		switch (tmp) {
		case 'd':
			fwu->device = optarg;
			break;
		case 'f':
			fwu->fw_filename = optarg;
			break;
		case 'h':
			help(argv[0]);
			return 0;
		case 'r':
			revert = 1;
			break;
		case 'n':
			no_reset = 1;
			break;
		case 'p':
			fwu->password = strtoul(optarg, NULL, 16);
			if (fwu->password == 0) {
				error_out("Invalid password\n");
				return -EINVAL;
			}
			debug_out("Using password: 0x%08x\n", fwu->password);
			break;
		case 'v':
			verbose = 1;
			break;
		case 'D':
			dsp_update = 1;
			break;
		case 'V':
			printf("Version %s\n", APPLICATION_VERSION);
			return 0;
		case OPT_NO_LOCKOUT:
			no_lockout = 1;
			break;
		default:
			error_out("%s: invalid option -- %c\n", argv[0], tmp);
			error_out("Try `%s --help' for more information.\n", argv[0]);
			return -EINVAL;
		}
	}

	/* Test for valid combinations of options */
	if (revert && dsp_update) {
		error_out("Cannot specify both --revert and --dsp options.\n");
		return -EINVAL;
	}
	if (revert && fwu->fw_filename) {
		error_out("Cannot specify both --revert and --fwfile options.\n");
		return -EINVAL;
	}
	if (dsp_update && !fwu->fw_filename) {
		error_out("DSP firmware update requires the --fwfile option.\n");
		return -EINVAL;
	}

	/* Get the I2C bus for the requested device */
	rc = dgchs_get_i2c_bus(fwu->device);
	if (rc < 0) {
		error_out("Unable to get I2C bus for device %s (rc=%d)\n", fwu->device, rc);
		return rc;
	}
	fwu->bus = rc;

	/* Open the bus (includes flock for exclusive access until we close it) */
	rc = dgchs_i2c_open(fwu->bus, I2C_OPEN_TIMEOUT_MS);
	if (rc < 0) {
		error_out("Unable to open I2C bus %d (rc=%d)\n", fwu->bus, rc);
		return rc;
	}
	fwu->fh = rc;
	debug_out("Device %s is using bus i2c-%d\n", fwu->device, fwu->bus);

	/* Gather CMIS identification information */
	rc = cmis_query_device(fwu);
	if (rc < 0)
		goto exit;

	/* Always display the CMIS Info */
	printf("%s CMIS device information:\n", fwu->device);
	printf("  CMIS Rev:      %u.%u\n", (fwu->cmis_revision & 0xF0) >> 4,
										fwu->cmis_revision & 0x0F);
	printf("  Vendor:        %s\n", fwu->vendor);
	printf("  Part Number:   %s\n", fwu->part_num);
	printf("  Serial Number: %s\n", fwu->serial_num);
	printf("  OEM Info:      %s\n", fwu->oem);
	printf("  Module Rev:    %s\n", fwu->module_revision);
	printf("  Active FW:     %u.%u\n", fwu->active_fw_rev >> 8,
									   fwu->active_fw_rev & 0xFF);
	printf("  Inactive FW:   %u.%u\n", fwu->inactive_fw_rev >> 8,
										fwu->inactive_fw_rev & 0xFF);
	printf("  Hardware Rev:  %u.%u\n", fwu->hardware_rev >> 8,
										fwu->hardware_rev & 0xFF);
	printf("  Module State:  %s\n", cmis_module_state_string(
									cmis_get_module_state(fwu->fh)));
	printf("\n");

	/* Ensure CDB capabilities that we need */
	rc = cdb_query_device(fwu);
	if (rc < 0)
		goto exit;

	/* Did they give us a file to read in? */
	if (fwu->fw_filename) {
		/* Import the firmware file into our buffer */
		rc = cdb_fw_file_import(fwu);
		if (rc < 0)
			goto exit;
	}

	/* If we are going to modify firmware... */
	if (revert || fwu->fw_filename) {
		if (no_lockout == 0) {
			/* lockout the link driver from I2C access */
			rc = dgchs_set_i2c_locked(fwu->device);
			if (rc < 0) {
				error_out("Failed to lockout driver access (rc=%d)\n", rc);
				goto exit;
			}
		}

		if (no_reset == 0) {
			/* Reset the device into low power mode */
			printf("Resetting into low power mode...\n");
			rc = cmis_reset_with_low_power_request(fwu->fh);
			if (rc < 0) {
				error_out("Failed reset to low power mode (rc=%d)\n", rc);
				goto exit;
			}
			sleep(10); /* Extra time for the module to fully reset */
		}

		/* Display initial firmware information */
		printf("Initial CDB firmware information:\n");
		rc = display_firmware_info(fwu);
		if (rc < 0) {
			error_out("Failed to determine firmware information (rc=%d).\n"
					"Continuing anyway\n", rc);
		}
		printf("\n");

		if (revert) {
			print_out("Reverting to previous CDB firmware image...\n");
			rc = cdb_fw_revert(fwu);
			if (rc < 0)
				goto exit;
		} else {
			/* Perform the type of firmware update requested */
			if (dsp_update) {
				print_out("Performing DSP firmware update...\n");
				rc = te_aoc_dsp_firmware_flash(fwu);
			} else {
				print_out("Performing CDB firmware update...\n");
				rc = cdb_fw_flash(fwu);
			}
			if (rc < 0)
				goto exit;
		}
	} else {
		/* They just get a single firmware report */
		report = 1;
	}

	if (report) {
		printf("CDB firmware information:\n");
	} else {
		printf("\n");
		printf("Final CDB firmware information:\n");
	}
	rc = display_firmware_info(fwu);
	if (rc < 0) {
		error_out("Failed to determine firmware information (rc=%d)\n", rc);
		goto exit;
	}

exit:
	dgchs_i2c_close(fwu->fh);

	if (no_lockout == 0 && !report) {
		/* Return to normal link driver I2C access */
		rc = dgchs_clear_i2c_locked(fwu->device);
		if (rc < 0) {
			error_out("Failed to re-enable driver access (rc=%d)\n", rc);
		}
	}

	return rc;
}
