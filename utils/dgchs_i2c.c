/* SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
 * Copyright 2026 Hewlett Packard Enterprise Development LP
 */

/*
 * dgchs_i2c.c - Cassini dgc headshell I2C I/O primitives.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include "dgchs_i2c.h"
#include "cmis_device.h"

#define PAGE_SELECT_DELAY_MS        5

/**
 * i2c_bus_flock_lock() - use flock to place an advisory lock on the i2c bus
 *  given the i2c-dev fhandle with timeout. Using timeout_ms=0 will return
 *  immediately with -EWOULDBLOCK if the lock could not be acquired.
 *
 * Returns 0 on success or negative value on error. If non-zero timeout_ms
 *  was specified, will return -ETIMEDOUT if the lock could not be
 *  acquired within the timeout.
 */
static int
i2c_bus_flock_lock(int fhandle, int timeout_ms)
{
	int rc;
	int remain_ms = timeout_ms;

	if (fhandle < 0)
		return -EBADF;

	while ((rc = flock(fhandle, LOCK_EX | LOCK_NB)) != 0) {
		if (errno == EWOULDBLOCK) {
			if (remain_ms > 0) {
				// 10ms retry interval
				remain_ms -= 10;
				usleep(10000);
				continue;
			} else if (timeout_ms) {
				return -ETIMEDOUT;
			}
		}
		if (errno > 0)
			return -errno;
		return errno;
	}
	return 0;
}

/**
 * i2c_bus_flock_unlock() - unlock the i2c bus previously locked by
 *  i2c_bus_flock_lock()
 *
 * Returns 0 on success or negative value on error.
 */
static int
i2c_bus_flock_unlock(int fhandle)
{
	int rc;

	if (fhandle < 0)
		return -EBADF;

	if ((rc = flock(fhandle, LOCK_UN)) != 0) {
		if (errno > 0)
			return -errno;
		return errno;
	}
	return 0;
}

static int
_write_i2c_locked(char *device, int locked)
{
	char filename[64];
	FILE *fp;
	int rc = 0;

	snprintf(filename, sizeof(filename), "/sys/kernel/debug/cxi/%s/i2c_locked", device);
	fp = fopen(filename, "w");

	if (!fp) {
		fprintf(stderr, "unable to open: %s\n", filename);
		return -1;
	}

	if (fwrite(locked ? "Y" : "N", sizeof(char), 1, fp) != 1) {
		fprintf(stderr, "unable to write to: %s\n", filename);
		rc = -1;
	}

	fclose(fp);
	return rc;
}

/**
 * dgchs_set_i2c_locked() - Write 'Y' to the i2c_locked sysfs entry for the
 *  given device so the driver knows not to touch the i2c interface.
 *
 * Returns 0 on success or negative value on error.
 */
int
dgchs_set_i2c_locked(char *device)
{
	return _write_i2c_locked(device, 1);
}

/**
 * dgchs_clear_i2c_locked() - Write 'N' to the i2c_locked sysfs entry for the
 *  given device so the driver resumes normal i2c operations.
 *
 * Returns 0 on success or negative value on error.
 */
int
dgchs_clear_i2c_locked(char *device)
{
	return _write_i2c_locked(device, 0);
}

/**
 * dgchs_i2c_open() - open a dgchs i2c bus device node for communications
 *  given the i2c bus_num with timeout_ms for acquiring the lock. Using
 *  timeout_ms=0 will return immediately with -EWOULDBLOCK if the lock
 *  could not be acquired.
 *
 * Returns int file handle on success or negative error value on failure.
 */
int
dgchs_i2c_open(int bus_num, int timeout_ms)
{
	int fhandle, rc;
	char bus_name[32];

	/* Sanity check bus_num. */
	if ((bus_num < 0) || (bus_num > 999))
		return -EINVAL;

	/* Open the character dev node for IOCTLs. */
	snprintf(bus_name, sizeof(bus_name), "/dev/i2c-%d", bus_num);
	fhandle = open(bus_name, O_RDWR);
	if (fhandle < 0) {
		if (errno > 0)
			return -errno;
		else
			return errno;
	}

	/* Acquire the flock */
	rc = i2c_bus_flock_lock(fhandle, timeout_ms);
	if (rc < 0) {
		close(fhandle);
		return rc;
	}

	return fhandle;
}

/**
 * dgchs_i2c_close() - Unlocks the flock and closes the provided i2c bus
 *  device node.
 *
 * Returns 0 on success or negative value on error.
 */
int
dgchs_i2c_close(int fhandle)
{
	int rc;

	rc = i2c_bus_flock_unlock(fhandle);
	if (rc < 0)
		fprintf(stderr, "i2c_bus_flock_unlock() failed! rc=%d", rc);
	return close(fhandle);
}

/**
 * dgchs_get_i2c_bus() - Get the headshell i2c bus number for the given CXI device.
 *
 * Returns the bus number on success or negative value on error.
 */
int
dgchs_get_i2c_bus(char *device)
{
	char filename[64];
	char hs_busname[64];
	FILE *name;
	int busnum, rc = -ENOENT;

	/* The i2c bus name we are looking for */
	snprintf(hs_busname, sizeof(hs_busname), "%s headshell", device);

	for (busnum = 0; busnum < 256; busnum++) {
		snprintf(filename, sizeof(filename), "/sys/class/i2c-dev/i2c-%d/name", busnum);
		name = fopen(filename, "r");
		if (name) {
			char busname[64];

			if (fgets(busname, sizeof(busname), name)) {
				if (strstr(busname, hs_busname)) {
					rc = busnum;
					fclose(name);
					break;
				}
			}
			fclose(name);
		}
	}
	return rc;
}

/**
 * dgchs_i2c_do_msgs() - Perform one or more I2C transactions.
 *
 * By the nature of the Linux I2C stack, these are guaranteed to be in
 * an uninterrupted sequence.
 *
 * Returns the number of msgs sent on success or negative value on error.
 */
int
dgchs_i2c_do_msgs(int fhandle, struct i2c_msg *msgs, uint8_t num_msg)
{
	struct i2c_rdwr_ioctl_data ioctl_data;
	int rc;

	if (fhandle < 0)
		return -EBADF;
	if (num_msg > I2C_RDWR_IOCTL_MAX_MSGS)
		return -EMSGSIZE;

	memset(&ioctl_data, 0, sizeof(struct i2c_rdwr_ioctl_data));
	ioctl_data.msgs = msgs;
	ioctl_data.nmsgs = num_msg;

	rc = ioctl(fhandle, I2C_RDWR, &ioctl_data);
	if ((rc != num_msg) && (rc > 0)) {
		fprintf(stderr, "I2C_RDWR ioctl failed! Expected num_msg=%d, got num_msg=%d", num_msg, rc);
		rc = -1;
	}
	return rc;
}

/**
 * dgchs_i2cdev_write() - Performs a write to an I2C device address.
 *
 * Returns 1 on success or negative value on error.
 */
int
dgchs_i2cdev_write(int fhandle, uint8_t addr, uint8_t *data, size_t bytes)
{
	struct i2c_msg msg;

	if (fhandle < 0)
		return -EBADF;
	if (((addr < 0x7) || (addr > 0x77)) && (addr != 0))
		return -EINVAL;

	/* Protect against future struct changes. */
	memset(&msg, 0, sizeof(struct i2c_msg));
	msg.addr = addr;
	msg.flags = 0;
	msg.len = bytes;
	msg.buf = data;

	return dgchs_i2c_do_msgs(fhandle, &msg, 1);
}

/**
 * dgchs_i2cdev_read() - Performs a read from an I2C device address.
 *
 * Returns 1 on success or negative value on error.
 */
int
dgchs_i2cdev_read(int fhandle, uint8_t addr, uint8_t *data, size_t bytes)
{
	struct i2c_msg msg;

	if (fhandle < 0)
		return -EBADF;
	if (((addr < 0x7) || (addr > 0x77)) && (addr != 0))
		return -EINVAL;

	/* Protect against future struct changes. */
	memset(&msg, 0, sizeof(struct i2c_msg));
	msg.addr = addr;
	msg.flags = I2C_M_RD;
	msg.len = bytes;
	msg.buf = (uint8_t *)data;

	return dgchs_i2c_do_msgs(fhandle, &msg, 1);
}

/**
 * dgchs_i2cdev_write_read() - Performs a write and read to an I2C device
 *  address.
 *
 * This will utilize the repeated start protocol.
 *
 * Returns 2 on success or negative value on error.
 */
int
dgchs_i2cdev_write_read(int fhandle, uint8_t addr, uint8_t *out_data,
		  size_t out_bytes, uint8_t *in_data, size_t in_bytes)
{
	struct i2c_msg msgs[2];

	if (fhandle < 0)
		return -EBADF;
	if (((addr < 0x7) || (addr > 0x77)) && (addr != 0))
		return -EINVAL;

	/* Setup the write. */
	memset(&msgs[0], 0, sizeof(struct i2c_msg));
	msgs[0].addr = addr;
	msgs[0].flags = 0;
	msgs[0].len = out_bytes;
	msgs[0].buf = out_data;

	/* Setup the read. */
	memset(&msgs[1], 0, sizeof(struct i2c_msg));
	msgs[1].addr = addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = in_bytes;
	msgs[1].buf = in_data;

	return dgchs_i2c_do_msgs(fhandle, msgs, 2);
}

/**
 * dgchs_i2cbus_write() - Performs a non-addressed write to an I2C bus.
 *
 * Returns 1 on success or negative value on error.
 */
int
dgchs_i2cbus_write(int fhandle, uint8_t *data, size_t bytes)
{
	return dgchs_i2cdev_write(fhandle, 0, data, bytes);
}

/**
 * dgchs_i2cbus_read() - Performs a non-addressed read from an I2C bus.
 *
 * Returns 1 on success or negative value on error.
 */
int
dgchs_i2cbus_read(int fhandle, uint8_t *data, size_t bytes)
{
	return dgchs_i2cdev_read(fhandle, 0, data, bytes);
}

/**
 * dgchs_i2cbus_write_read() - Performs an unaddressed write and read to an I2C
 *  bus.
 *
 * This will, by Linux stack definition, utilize the repeated start protocol.
 *
 * Returns 2 on success or negative value on error.
 */
int
dgchs_i2cbus_write_read(int fhandle, uint8_t *out_data, size_t out_bytes,
		  uint8_t *in_data, size_t in_bytes)
{
	return dgchs_i2cdev_write_read(fhandle, 0, out_data, out_bytes, in_data,
				 in_bytes);
}

/* CMIS-specific Routines ***************************************************/

/**
 * @brief Write to the headshell's page select register.
 *        SFF-8636 specification has a single page select byte.
 *        CMIS specification 3.0 has a bank select and page select byte which
 *        are supposed to be written separately (byte transactions).
 *        CMIS specification 4.0+ has a bank select and page select byte which
 *        are supposed to be written together. If we don't update both
 *        bytes we can end up with an invalid bank:page combination and
 *        the module will force it to page0.
 *
 *        If force_byte_writes is set, then we will enforce the 3.0 behavior.
 *
 * @return 0 on success or a negative error on failure.
 */
static int
_cmis_set_page(int fh, uint8_t page, int force_byte_writes)
{
	uint8_t word_buf[3] = { 0x7e, 0, page };
	uint8_t byte_buf[2] = { 0x7f, page };
	int rc = 0, retries = I2C_DEV_WRITE_RETRIES;
	uint8_t data;

	while (retries--) {
		// CMIS v4.0 and later write bank and page selects together
		if (!force_byte_writes)
			rc = dgchs_i2cdev_write(fh, CMISADDR, word_buf, sizeof(word_buf));
		else
			rc = dgchs_i2cdev_write(fh, CMISADDR, byte_buf, sizeof(byte_buf));
		if (rc > 0) {
			rc = 0;
			usleep(PAGE_SELECT_DELAY_MS * 1000);
			break;
		}
		//fprintf(stderr, "%s: rc=%d retries=%d", __func__, rc, retries);
	}

	if (force_byte_writes)
		return rc;

	/* Check if the CMIS 4.0+ procedure worked as expected */
	if (!cmis_read_byte(fh, 0, 0x7f, &data)) {
		if (data != page) {
			/* It didn't; force older byte write behavior */
			return _cmis_set_page(fh, page, 1);
		}
	}

	return rc;
}

int
cmis_set_page(int fh, uint8_t page)
{
	return _cmis_set_page(fh, page, 0);
}

/**
 * @brief Read a byte from the headshell with retries.
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_read_byte(int fh, uint8_t page, uint8_t addr, uint8_t *data)
{
	int rc = 0, retries = I2C_DEV_WRITE_RETRIES;

	// If addr >= 0x80 do page select
	if (addr >= 0x80) {
		rc = cmis_set_page(fh, page);
		// If setting page=0 failed, be lenient
		if (page && (rc < 0))
			return rc;
	}
	while (retries--) {
		rc = dgchs_i2cdev_write_read(fh, CMISADDR, &addr, 1, data, 1);
		if (rc > 0) {
			rc = 0;
			break;
		}
		//fprintf(stderr, "%s: rc=%d retries=%d", __func__, rc, retries);
	}
	return rc;
}

/**
 * @brief Write a byte to the headshell with retries.
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_write_byte(int fh, uint8_t page, uint8_t addr, uint8_t data)
{
	uint8_t buf[2] = { addr, data };
	int rc = 0, retries = I2C_DEV_WRITE_RETRIES;

	if (addr >= 0x80) {
		rc = cmis_set_page(fh, page);
		// If setting page=0 failed, be lenient
		if (page && (rc < 0))
			return rc;
	}

	while (retries--) {
		rc = dgchs_i2cdev_write(fh, CMISADDR, buf, 2);
		if (rc > 0) {
			rc = 0;
			break;
		}
		//fprintf(stderr, "%s: rc=%d retries=%d", __func__, rc, retries);
	}
	return rc;
}

/**
 * @brief Read a block of bytes from the headshell without bank or page select
 * The caller is expected to limit the size of the read length (chunk size)
 * as necessary.
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_read_raw_block(int fh, uint8_t addr, int len, void *data)
{
	int rc = 0, retries = I2C_DEV_WRITE_RETRIES;

	while (retries--) {
		rc = dgchs_i2cdev_write_read(fh, CMISADDR, &addr, 1, data, len);
		if (rc > 0) {
			rc = 0;
			break;
		}
		//fprintf(stderr, "%s: rc=%d retries=%d", __func__, rc, retries);
	}
	return rc;
}

/**
 * @brief Read a block of bytes from the headshell without bank or page select
 * The caller is expected to limit the size of the read length (chunk size)
 * as necessary.
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_read_block(int fh, uint8_t page, uint8_t addr, int len, void *data)
{
	int rc;

	// If the read will include bytes >= 0x80, do page select
	if (addr + len > 0x80) {
		rc = cmis_set_page(fh, page);
		// If setting page=0 failed, be lenient
		if (page && (rc < 0))
			return rc;
	}
	return cmis_read_raw_block(fh, addr, len, data);
}

/**
 * @brief Write a block of bytes to the headshell without bank or page select
 * The caller is expected to limit the size of the write length (chunk size)
 * as necessary. Currently supports writing up to 255 bytes at a time.
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_write_raw_block(int fh, uint8_t addr, int len, void *data)
{
	uint8_t buf[256];
	uint8_t *ptr;
	int i, rc = 0, retries = I2C_DEV_WRITE_RETRIES;

	if (len >= sizeof(buf))
		return -E2BIG;

	/* First byte we write is the addr, followed by the data */
	buf[0] = addr;
	ptr = (uint8_t *)data;
	for (i = 0; i < len; i++) {
		buf[i+1] = *ptr++;
	}

	while (retries--) {
		rc = dgchs_i2cdev_write(fh, CMISADDR, buf, len + 1);
		if (rc > 0) {
			rc = 0;
			break;
		}
		//fprintf(stderr, "%s: rc=%d retries=%d", __func__, rc, retries);
	}
	return rc;
}

/**
 * @brief Read the CMIS vendor string.
 *  The provided string buffer must be at least 17 bytes long.
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_get_vendor(int fh, int len, char *vendor)
{
	uint8_t addr = 129; // Vendor name starts at byte 129 in page 0
	char *ptr = vendor;
	int size, rc;

	/* We require 16 characters plus null terminator */
	if (len <= 16)
		return -EINVAL;

	memset(vendor, 0, len);
	len = 16;

	while (len > 0) {
		size = len > CMIS_CHUNK_SIZE ? CMIS_CHUNK_SIZE : len;
		rc = cmis_read_block(fh, 0, addr, size, ptr);
		if (rc < 0) {
			return rc;
		}
		len -= size;
		ptr += size;
		addr += size;
	}
	return 0;
}

/**
 * @brief Read the CMIS vendor string.
 *  The provided string buffer must be at least 17 bytes long.
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_get_part_num(int fh, int len, char *part_num)
{
	uint8_t addr = 148; // Part number starts at byte 148 in page 0
	char *ptr = part_num;
	int size, rc;

	/* We require 16 characters plus null terminator */
	if (len <= 16)
		return -EINVAL;

	memset(part_num, 0, len);
	len = 16; // Max part number string length

	while (len > 0) {
		size = len > CMIS_CHUNK_SIZE ? CMIS_CHUNK_SIZE : len;
		rc = cmis_read_block(fh, 0, addr, size, ptr);
		if (rc < 0) {
			return rc;
		}
		len -= size;
		ptr += size;
		addr += size;
	}
	return 0;
}

/**
 * @brief Read the CMIS "vendor rev" ASCII bytes.
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_get_module_rev(int fh, int len, char *rev)
{
	/* We require 2 characters plus null terminator */
	if (len <= 3)
		return -EINVAL;

	memset(rev, 0, len);
	len = 2; // Max module revision string length

	// Rev starts at byte 164 in page 0
	return cmis_read_block(fh, 0, 164, len, rev);
}

/**
 * @brief Read the CMIS serial number string.
 *  The provided string buffer must be at least 17 bytes long.
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_get_serial_num(int fh, int len, char *serial_num)
{
	uint8_t addr = 166; // Serial number starts at byte 166 in page 0
	char *ptr = serial_num;
	int size, rc;

	/* We require 16 characters plus null terminator */
	if (len <= 16)
		return -EINVAL;

	memset(serial_num, 0, len);
	len = 16; // Max serial number string length

	while (len > 0) {
		size = len > CMIS_CHUNK_SIZE ? CMIS_CHUNK_SIZE : len;
		rc = cmis_read_block(fh, 0, addr, size, ptr);
		if (rc < 0) {
			return rc;
		}
		len -= size;
		ptr += size;
		addr += size;
	}
	return 0;
}

/**
 * @brief Read the CMIS vendor string.
 *  The provided string buffer must be at least 17 bytes long.
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_get_oem(int fh, int len, char *oem)
{
	uint8_t addr = 224; // OEM starts at byte 224 in page 0
	char *ptr = oem;
	int size, rc;

	/* We require 32 characters plus null terminator */
	if (len <= 32)
		return -EINVAL;

	memset(oem, 0, len);
	len = 32; // Max OEM string length

	while (len > 0) {
		size = len > CMIS_CHUNK_SIZE ? CMIS_CHUNK_SIZE : len;
		rc = cmis_read_block(fh, 0, addr, size, ptr);
		if (rc < 0) {
			return rc;
		}
		len -= size;
		ptr += size;
		addr += size;
	}
	return 0;
}

/**
 * @brief Read the CMIS active firmware revision bytes.
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_get_active_fw_rev(int fh, uint16_t *active_fw_rev)
{
	uint8_t buf[2];
	int rc;

	/* Active FW Rev starts at byte 39 in page 0 */
	rc = cmis_read_block(fh, 0, 39, sizeof(buf), buf);
	if (rc < 0) {
		return rc;
	}
	*active_fw_rev = (buf[0] << 8) | buf[1];

	/* Unprogrammed bytes treated as 0 */
	if (*active_fw_rev == 0xFFFF)
		*active_fw_rev = 0;
	return 0;
}

/**
 * @brief Read the CMIS inactive firmware revision bytes.
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_get_inactive_fw_rev(int fh, uint16_t *inactive_fw_rev)
{
	uint8_t buf[2];
	int rc;

	/* Inactive FW Rev starts at byte 128 in page 1 */
	rc = cmis_read_block(fh, 1, 128, sizeof(buf), buf);
	if (rc < 0) {
		return rc;
	}
	*inactive_fw_rev = (buf[0] << 8) | buf[1];

	/* Unprogrammed bytes treated as 0 */
	if (*inactive_fw_rev == 0xFFFF)
		*inactive_fw_rev = 0;
	return 0;
}

/**
 * @brief Read the CMIS hardware revision bytes.
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_get_hardware_rev(int fh, uint16_t *hardware_rev)
{
	uint8_t buf[2];
	int rc;

	/* Hardware Rev starts at byte 130 in page 1 */
	rc = cmis_read_block(fh, 1, 130, sizeof(buf), buf);
	if (rc < 0) {
		return rc;
	}
	*hardware_rev = (buf[0] << 8) | buf[1];

	/* Unprogrammed bytes treated as 0 */
	if (*hardware_rev == 0xFFFF)
		*hardware_rev = 0;
	return 0;
}

/**
 * @brief Ensure the device is in low power mode by setting the
 * LowPowerRequest bit in byte 26.
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_set_low_power_mode(int fh)
{
	/* Set only the LowPowerRequest bit in byte 26 */
	return cmis_write_byte(fh, 0, 26, (1 << 4));
}

/**
 * @brief Reset the device and also set the low power request bit
 * in byte 26
 *
 * @return 0 on success or a negative error on failure.
 */
int
cmis_reset_with_low_power_request(int fh)
{
	/* Set only the LowPowerRequest and SoftwareReset bits in byte 26 */
	return cmis_write_byte(fh, 0, 26, (1 << 4) | (1 << 3));
}

/**
 * @brief Return the CMIS module state from status byte 3.
 *
 * @return ModuleState value or zero (invalid state) on failure.
 */
uint8_t
cmis_get_module_state(int fh)
{
	uint8_t status = 0;

	cmis_read_byte(fh, 0, 3, &status);

	return (status & CMIS_STATE_MASK) >> CMIS_STATE_SHIFT;
}

/**
 * @brief Return a string representation of the CMIS module state.
 *
 * @return Pointer to static string.
 */
const char *
cmis_module_state_string(uint8_t state)
{
	switch (state) {
	case CMIS_STATE_LOW_PWR:
		return "ModuleLowPower";
	case CMIS_STATE_PWR_UP:
		return "ModulePwrUp";
	case CMIS_STATE_READY:
		return "ModuleReady";
	case CMIS_STATE_PWR_DOWN:
		return "ModulePwrDn";
	case CMIS_STATE_FAULT:
		return "ModuleFault";
	default:
		return "Unknown";
	}
}
