/* SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
 * Copyright 2026 Hewlett Packard Enterprise Development LP
 */

#ifndef __DGCHS_I2C_HEADER__
#define __DGCHS_I2C_HEADER__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define CMISADDR                                0x50
#define I2C_OPEN_TIMEOUT_MS                     1000
#define I2C_DEV_WRITE_RETRIES                   3
#define CMIS_CHUNK_SIZE                         8

extern int dgchs_set_i2c_locked(char *device);
extern int dgchs_clear_i2c_locked(char *device);
extern int dgchs_i2c_open(int bus_num, int timeout_ms);
extern int dgchs_i2c_close(int fhandle);
extern int dgchs_get_i2c_bus(char *device);
extern int dgchs_i2c_do_msgs(int fhandle, struct i2c_msg *msgs, uint8_t num_msg);
extern int dgchs_i2cdev_write(int fhandle, uint8_t addr, uint8_t *data, size_t bytes);
extern int dgchs_i2cdev_read(int fhandle, uint8_t addr, uint8_t *data, size_t bytes);
extern int dgchs_i2cdev_write_read(int fhandle, uint8_t addr, uint8_t *out_data,
			size_t out_bytes, uint8_t *in_data, size_t in_bytes);
extern int dgchs_i2cbus_write(int fhandle, uint8_t *data, size_t bytes);
extern int dgchs_i2cbus_read(int fhandle, uint8_t *data, size_t bytes);
extern int dgchs_i2cbus_write_read(int fhandle, uint8_t *out_data, size_t out_bytes,
			uint8_t *in_data, size_t in_bytes);

extern int cmis_set_page(int fh, uint8_t page);
extern int cmis_read_byte(int fh, uint8_t page, uint8_t addr, uint8_t *data);
extern int cmis_write_byte(int fh, uint8_t page, uint8_t addr, uint8_t data);
extern int cmis_read_raw_block(int fh, uint8_t addr, int len, void *data);
extern int cmis_read_block(int fh, uint8_t page, uint8_t addr, int len, void *data);
extern int cmis_write_raw_block(int fh, uint8_t addr, int len, void *data);
extern int cmis_get_vendor(int fh, int len, char *vendor);
extern int cmis_get_part_num(int fh, int len, char *part_num);
extern int cmis_get_module_rev(int fh, int len, char *rev);
extern int cmis_get_serial_num(int fh, int len, char *serial_num);
extern int cmis_get_oem(int fh, int len, char *oem);
extern int cmis_get_active_fw_rev(int fh, uint16_t *active_fw_rev);
extern int cmis_get_inactive_fw_rev(int fh, uint16_t *inactive_fw_rev);
extern int cmis_get_hardware_rev(int fh, uint16_t *hardware_rev);
extern int cmis_set_low_power_mode(int fh);
extern int cmis_reset_with_low_power_request(int fh);
extern uint8_t cmis_get_module_state(int fh);
extern const char *cmis_module_state_string(uint8_t state);

#ifdef __cplusplus
}
#endif
#endif // __DGCHS_I2C_HEADER__
