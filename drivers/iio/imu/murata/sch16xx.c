// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
//
// This software is released under the BSD license as follows.
// Copyright (c) 2024, Murata Electronics Oy.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following 
// conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials
//       provided with the distribution.
//    3. Neither the name of Murata Electronics Oy nor the names of its    
//       contributors may be used to endorse or promote products derived    
//       from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
// IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/irq.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/iio/sysfs.h>
#include <linux/bitops.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>

/* Dynamic range for accelerometer, if no definition is found on device tree */
static char *default_accel_dyn_range = "DYN1";
module_param(default_accel_dyn_range, charp, 0660);

/* Dynamic range for gyroscope, if no definition is found on device tree */
static char *default_gyro_dyn_range = "DYN1";
module_param(default_gyro_dyn_range, charp, 0660);

/* Dynamic range for accelerometer. If defined, overrides the setting in device tree */
static char *override_accel_dyn_range = "";
module_param(override_accel_dyn_range, charp, 0660);

/* Dynamic range for gyroscope. If defined, overrides the setting in device tree */
static char *override_gyro_dyn_range = "";
module_param(override_gyro_dyn_range, charp, 0660);

/**
 * Frame field masks
 */
#define TA_FIELD_MASK		0xFFC000000000ULL
#define TA_FIELD_SHIFT		38
#define SA_FIELD_MASK		0x7FE000000000ULL
#define SA_FIELD_SHIFT		37
#define DATA_FIELD_MASK		0x00000FFFFF00ULL
#define DATA_FIELD_SHIFT	8
#define CRC_FIELD_MASK		0x0000000000FFULL

#define RW_FIELD_SHIFT		37
#define FT_FIELD_SHIFT		35

/**
 * Status Summary Register bit definitions
 */
#define S_SUM_CMN           0x0080
#define S_SUM_RATE_X        0x0040
#define S_SUM_RATE_Y        0x0020
#define S_SUM_RATE_Z        0x0010
#define S_SUM_ACC_X         0x0008
#define S_SUM_ACC_Y         0x0004
#define S_SUM_ACC_Z         0x0002
#define S_SUM_INIT_RDY      0x0001

/**
 * Saturation Status Summary Register bit definitions
 */
#define S_SUM_SAT_RATE_X1       0x4000
#define S_SUM_SAT_RATE_Y1       0x2000
#define S_SUM_SAT_RATE_Z1       0x1000
#define S_SUM_SAT_ACC_X1        0x0800
#define S_SUM_SAT_ACC_Y1        0x0400
#define S_SUM_SAT_ACC_Z1        0x0200
#define S_SUM_SAT_ACC_X3        0x0100
#define S_SUM_SAT_ACC_Y3        0x0080
#define S_SUM_SAT_ACC_Z3        0x0040
#define S_SUM_SAT_RATE_X2       0x0020
#define S_SUM_SAT_RATE_Y2       0x0010
#define S_SUM_SAT_RATE_Z2       0x0008
#define S_SUM_SAT_ACC_X2        0x0004
#define S_SUM_SAT_ACC_Y2        0x0002
#define S_SUM_SAT_ACC_Z2        0x0001

/**
 * Common Status Register bit definitions
 */
#define S_COM_MCLK              0x0400
#define S_COM_DUAL_CLOCK        0x0200
#define S_COM_DSP               0x0100
#define S_COM_SVM               0x0080
#define S_COM_HV_CP             0x0040
#define S_COM_SUPPLY            0x0020
#define S_COM_TEMP              0x0010
#define S_COM_NMODE             0x0008
#define S_COM_NVM_STS           0x0004
#define S_COM_CMN_STS           0x0002
#define S_COM_CMN_STS_RDY       0x0001

/**
 * Rate Common Status Register bit definitions
 */
#define S_RATE_COM_PRI_AGC      0x0080
#define S_RATE_COM_PRI          0x0040
#define S_RATE_COM_PRI_START    0x0020
#define S_RATE_COM_HV           0x0010
#define S_RATE_COM_SD_STS       0x0004
#define S_RATE_COM_BOND_STS     0x0002
#define S_RATE_COM_STS_RDY      0x0001

/**
 * Rate Status X Register bit definitions
 */
#define S_RATE_X_DEC_SAT        0x0200
#define S_RATE_X_INTP_SAT       0x0100
#define S_RATE_X_STC_DIG        0x0040
#define S_RATE_X_STC_ANA        0x0020
#define S_RATE_X_QC             0x0010

/**
 * Rate Status Y Register bit definitions
 */
#define S_RATE_Y_DEC_SAT        0x0200
#define S_RATE_Y_INTP_SAT       0x0100
#define S_RATE_Y_STC_DIG        0x0040
#define S_RATE_Y_STC_ANA        0x0020
#define S_RATE_Y_QC             0x0010

/**
 * Rate Status Z Register bit definitions
 */
#define S_RATE_Z_DEC_SAT        0x0200
#define S_RATE_Z_INTP_SAT       0x0100
#define S_RATE_Z_STC_DIG        0x0040
#define S_RATE_Z_STC_ANA        0x0020
#define S_RATE_Z_QC             0x0010

/**
 * ACC Status X Register bit definitions
 */
#define S_ACC_X_SAT             0x0400
#define S_ACC_X_DEC_SAT         0x0200
#define S_ACC_X_INTP_SAT        0x0100
#define S_ACC_X_STC_DIG         0x0080
#define S_ACC_X_STC_TCAP        0x0040
#define S_ACC_X_STC_SDD         0x0020
#define S_ACC_X_STC_N           0x0010
#define S_ACC_X_SD_STS          0x0004
#define S_ACC_X_STS             0x0002
#define S_ACC_X_STS_RDY         0x0001

/**
 * ACC Status Y Register bit definitions
 */
#define S_ACC_Y_SAT             0x0400
#define S_ACC_Y_DEC_SAT         0x0200
#define S_ACC_Y_INTP_SAT        0x0100
#define S_ACC_Y_STC_DIG         0x0080
#define S_ACC_Y_STC_TCAP        0x0040
#define S_ACC_Y_STC_SDD         0x0020
#define S_ACC_Y_STC_N           0x0010
#define S_ACC_Y_SD_STS          0x0004
#define S_ACC_Y_STS             0x0002
#define S_ACC_Y_STS_RDY         0x0001

/**
 * ACC Status Z Register bit definitions
 */
#define S_ACC_Z_SAT             0x0400
#define S_ACC_Z_DEC_SAT         0x0200
#define S_ACC_Z_INTP_SAT        0x0100
#define S_ACC_Z_STC_DIG         0x0080
#define S_ACC_Z_STC_TCAP        0x0040
#define S_ACC_Z_STC_SDD         0x0020
#define S_ACC_Z_STC_N           0x0010
#define S_ACC_Z_SD_STS          0x0004
#define S_ACC_Z_STS             0x0002
#define S_ACC_Z_STS_RDY         0x0001

/**
 * Sync active status bit definitions
 */
#define S_SYNC_ACTIVE_RATE1     0x0007
#define S_SYNC_ACTIVE_ACC1      0x0038
#define S_SYNC_ACTIVE_RATE2     0x01C0
#define S_SYNC_ACTIVE_ACC2      0x0E00

/**
 * User interface control bit definitions
 */
#define SPI_SUPPLY_1V8          0x4000
#define FTREE_TDEL_2_5_MS       0x2000
#define SYNC_TOC_TH_10_MS       0x0600
#define SYNC_DEC_EN             0x0100
#define SYNC_INTP_EN            0x0080
#define MISO_SR_CTRL            0x0008
#define DRY_SR_CTRL             0x0004

#define REG_RATE_X1             0x01
#define REG_RATE_Y1             0x02
#define REG_RATE_Z1             0x03
#define REG_ACC_X1              0x04
#define REG_ACC_Y1              0x05
#define REG_ACC_Z1              0x06
#define REG_TEMP                0x10

#define REG_CTRL_RESET          0x36
#define REG_CTRL_RATE           0x28
#define REG_CTRL_ACC12          0x29
#define REG_CTRL_ACC3           0x2A
#define REG_CTRL_USER_IF        0x33
#define REG_CTRL_MODE           0x35

#define REG_CTRL_FILT_RATE      0x25
#define REG_CTRL_FILT_ACC12     0x26

#define REG_STAT_SUM            0x14
#define REG_STAT_SUM_SAT        0x15
#define REG_STAT_COM            0x16
#define REG_STAT_RATE_COM       0x17
#define REG_STAT_RATE_X         0x18
#define REG_STAT_RATE_Y         0x19
#define REG_STAT_RATE_Z         0x1A
#define REG_STAT_ACC_X          0x1B
#define REG_STAT_ACC_Y          0x1C
#define REG_STAT_ACC_Z          0x1D
#define REG_STAT_SYNC_ACTIVE    0x1E
#define REG_STAT_INFO           0x1F

#define REG_COMP_ID             0x3C
#define REG_SN_ID1              0x3D
#define REG_SN_ID2              0x3E
#define REG_SN_ID3              0x3F

#define EN_SENSOR               0x01
#define EOI_CTRL                0x02
#define SOFTRESET_CTRL          0x0A

// HPC1 product codes vs ID register content.
struct product_code {
	int id_value;
	const char *product_code;
};

static struct product_code product_codes[] = {
		{ .id_value = 0x0007, .product_code = "SCH1633-B01" },
		{ .id_value = 0x0017, .product_code = "SCH1633-B13" },
		{ .id_value = 0x0023, .product_code = "SCH16T-K01" },
		{ .id_value = 0x0021, .product_code = "SCH16T-K10" },
		{ .id_value = 0x002E, .product_code = "SCH1633-D01" },
};

enum SCH16XX_SCAN {
	SCH16XX_SCAN_RATE_X,
	SCH16XX_SCAN_RATE_Y,
	SCH16XX_SCAN_RATE_Z,
	SCH16XX_SCAN_ACC_X,
	SCH16XX_SCAN_ACC_Y,
	SCH16XX_SCAN_ACC_Z,
	SCH16XX_SCAN_TEMP,
	SCH16XX_SCAN_TIMESTAMP,
	SCH16XX_SCAN_MAX_COUNT
};


// Maximum transfer count in scan mode
// 6 output registers
// 4 status registers
// 1 sync active status
// 1 dummy frame
#define SCH16XX_MAX_OUTPUT_COUNT 6
#define SCH16XX_MAX_STATUS_COUNT 5
#define SCH16XX_MAX_TRANSFER_COUNT (SCH16XX_MAX_OUTPUT_COUNT + SCH16XX_MAX_STATUS_COUNT + 1)

struct sch16xx_dev
{
	struct spi_device *spi;
	struct gpio_desc *sync_gpio;
	struct gpio_desc *reset_gpio;
	struct spi_transfer transfer[SCH16XX_MAX_TRANSFER_COUNT];
	unsigned int ta;
	__be64 tx[SCH16XX_MAX_TRANSFER_COUNT];
	__be64 rx[SCH16XX_MAX_TRANSFER_COUNT];
	int active_scan_size;
	int transfer_size;
	const struct dynamic_range_params *acc12_range;
	const struct dynamic_range_params *rate_range;
	const struct filter_params *acc12_filter;
	const struct filter_params *rate_filter;
	const char *product_code;
	uint16_t comp_id;
	bool vddio_1v8;
};

struct dynamic_range_params {
	const char *dyn_n;
	int bit_value;
	const char *measurement_range;
	int sensitivity_20bits;
};

static const struct dynamic_range_params rate_params[] = {
		// bits = 0 undefined
		{ .dyn_n = "DYN1", .bit_value = 1, .measurement_range = "300", .sensitivity_20bits = 1600 },
		{ .dyn_n = "DYN2", .bit_value = 2, .measurement_range = "300", .sensitivity_20bits = 1600 },
		{ .dyn_n = "DYN3", .bit_value = 3, .measurement_range = "125", .sensitivity_20bits = 3200 },
		{ .dyn_n = "DYN4", .bit_value = 4, .measurement_range = "62.5", .sensitivity_20bits = 6400 },
		{ 0 }
};

static const struct dynamic_range_params acc12_params[] = {
		// bits = 0 undefined for ACC12
		{ .dyn_n = "DYN1", .bit_value = 1, .measurement_range = "80", .sensitivity_20bits = 3200 },
		{ .dyn_n = "DYN2", .bit_value = 2, .measurement_range = "60", .sensitivity_20bits = 6400 },
		{ .dyn_n = "DYN3", .bit_value = 3, .measurement_range = "30", .sensitivity_20bits = 12800 },
		{ .dyn_n = "DYN4", .bit_value = 4, .measurement_range = "15", .sensitivity_20bits = 25600 },
		{ 0 }
};

static const struct dynamic_range_params *find_dynamic_range_params(const struct dynamic_range_params *params, const char *dyn_n)
{
	int i = 0;
	while (params[i].dyn_n != NULL) {

		if (strcmp(params[i].dyn_n, dyn_n) == 0)
			return &params[i];

		i++;
	}
	return NULL;
}

struct filter_params {
	const char *name;
	int bit_value;
	const char *cutoff;
	int cutoff_hz;
};

static const int sch16xx_filter_freqs[] = { 13, 30, 68, 235, 280, 370 };

static const struct filter_params filter_params[] = {
		{ .name = "LPF0", .bit_value = 0, .cutoff = "68Hz", .cutoff_hz = 68 },
		{ .name = "LPF1", .bit_value = 1, .cutoff = "30Hz", .cutoff_hz = 30 },
		{ .name = "LPF2", .bit_value = 2, .cutoff = "13Hz", .cutoff_hz = 13 },
		{ .name = "LPF3", .bit_value = 3, .cutoff = "280Hz", .cutoff_hz = 280 },
		{ .name = "LPF4", .bit_value = 4, .cutoff = "370Hz", .cutoff_hz = 370 },
		{ .name = "LPF5", .bit_value = 5, .cutoff = "235Hz", .cutoff_hz = 235 },
		{ .name = "LPF6", .bit_value = 6, .cutoff = "TBD", .cutoff_hz = -1 },
		{ .name = "LPF7", .bit_value = 7, .cutoff = "Bypass", .cutoff_hz = -1 },
};

static const struct filter_params *find_filter_params_hz(const struct filter_params *params, int params_size, int cutoff_freq)
{
	int i;
	for (i = 0; i < params_size; i++) {

		if (params[i].cutoff_hz == cutoff_freq) {
			return &params[i];
		}	
	}

	return NULL;
}

static const struct filter_params *find_filter_params(const struct filter_params *params, const char *name)
{
	int i = 0;
	while (params[i].name != NULL) {

		if (strcmp(params[i].name, name) == 0)
			return &params[i];

		i++;
	}
	return NULL;
}

static const char* find_product_code(int id_value)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(product_codes); i++) {

		if (product_codes[i].id_value == id_value)
			return product_codes[i].product_code;
	}
	return NULL;
}

#define SCH16XX_CHANNEL(_type, _modifier, _index, _address) {	\
 	.type = _type,												\
	.channel2 = _modifier,										\
	.modified = 1,												\
	.address = _address,										\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | 				\
		BIT(IIO_CHAN_INFO_SCALE),								\
	.info_mask_shared_by_type =                                 \
			BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),   \
	.info_mask_shared_by_type_available =   					\
			BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),	\
	.scan_index = _index,										\
	.scan_type = {												\
		.sign = 's',											\
		.realbits = 20,											\
		.storagebits = 32,										\
		.shift = 0,												\
		.endianness = IIO_CPU,									\
	}															\
}

static const struct iio_chan_spec sch16xx_channels[] = {
	SCH16XX_CHANNEL(IIO_ANGL_VEL, IIO_MOD_X, SCH16XX_SCAN_RATE_X, REG_RATE_X1),
	SCH16XX_CHANNEL(IIO_ANGL_VEL, IIO_MOD_Y, SCH16XX_SCAN_RATE_Y, REG_RATE_Y1),
	SCH16XX_CHANNEL(IIO_ANGL_VEL, IIO_MOD_Z, SCH16XX_SCAN_RATE_Z, REG_RATE_Z1),
	SCH16XX_CHANNEL(IIO_ACCEL, IIO_MOD_X, SCH16XX_SCAN_ACC_X, REG_ACC_X1),
	SCH16XX_CHANNEL(IIO_ACCEL, IIO_MOD_Y, SCH16XX_SCAN_ACC_Y, REG_ACC_Y1),
	SCH16XX_CHANNEL(IIO_ACCEL, IIO_MOD_Z, SCH16XX_SCAN_ACC_Z, REG_ACC_Z1),
	{
		.type = IIO_TEMP,
		.modified = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = 0,
		.scan_index = SCH16XX_SCAN_TEMP,
		.address = REG_TEMP,
		.scan_type = {
			.sign = 's',
			.realbits = 20,
			.storagebits = 32,
			.shift = 0,
			.endianness = IIO_CPU,
		}
	},
	IIO_CHAN_SOFT_TIMESTAMP(SCH16XX_SCAN_TIMESTAMP),
};

#define SCH16XX_CRC8_POLYNOMIAL 0x2F
DECLARE_CRC8_TABLE(sch16xx_crc8_table);

static uint8_t CRC8(uint64_t frame)
{
	__be64 be64_data = cpu_to_be64(frame);

	return crc8(sch16xx_crc8_table, ((u8*)&be64_data) + 2, 5, 0x42);
}

static bool sch16xx_is_crc_valid(struct spi_device *spi, u64 frame)
{
	u8 crc = CRC8(frame);
	if ((frame & CRC_FIELD_MASK) != crc) {
		dev_err_ratelimited(&spi->dev, "CRC mismatch in frame %012llx (crc=%02x)", frame, crc);
		return false;
	}
	return true;
}

static bool sch16xx_is_response_valid(struct spi_device *spi, u64 response, u64 request)
{
	unsigned int ta, sa;

	ta = (request & TA_FIELD_MASK) >> TA_FIELD_SHIFT;
	sa = (response & SA_FIELD_MASK) >> SA_FIELD_SHIFT;

	if (sa != ta) {
		dev_err_ratelimited(&spi->dev, "Target/source address mismatch. (target=%x, source=%x)", ta, sa);
		return false;
	}
	return true;
}

static u64 append_crc(u64 frame)
{
	return (frame & 0xffffffffff00ULL) | CRC8(frame);
}

static u64 create_read_frame(unsigned int reg_address, unsigned int ta)
{
	unsigned int address = reg_address | (ta << 8);
	u64 request = ((((u64)address) << TA_FIELD_SHIFT) & TA_FIELD_MASK) | (1ULL << FT_FIELD_SHIFT);
	return append_crc(request);
}

static u64 create_write_frame(unsigned int reg_address, unsigned int ta,  u32 data)
{
	unsigned int address = reg_address | (ta << 8);
	u64 request = ((((u64)address) << TA_FIELD_SHIFT) & TA_FIELD_MASK) | (1ULL << FT_FIELD_SHIFT) |
			(1ULL << RW_FIELD_SHIFT) | (((u64)data << DATA_FIELD_SHIFT) & DATA_FIELD_MASK);
	return append_crc(request);
}

static int sch16xx_read_single(struct sch16xx_dev *dev, unsigned int address, u32 *data, bool check_crc)
{
	int ret;
	u64 response;
	__be64 rx = 0;
	struct spi_device *spi = dev->spi;
	u64 request = create_read_frame(address, dev->ta);
	__be64 tx = cpu_to_be64(request);

	struct spi_transfer t[] = {
		{
			.tx_buf = ((u8 *)&tx) + 2,
			.len = (48 / 8),
			.bits_per_word = 8,
			.delay = {
				.value = 10,
				.unit = SPI_DELAY_UNIT_USECS,
			},
			.cs_change = 1,
		},
		{
			.tx_buf = ((u8 *)&tx) + 2,
			.rx_buf = ((u8 *)&rx) + 2,
			.len = (48 / 8),
			.bits_per_word = 8,
			.delay = {
				.value = 10,
				.unit = SPI_DELAY_UNIT_USECS,
			},
		}
	};

	ret = spi_sync_transfer(spi, t, ARRAY_SIZE(t));
	if (ret)
		return ret;

	response = be64_to_cpu(rx);
	*data = (u32)((response & DATA_FIELD_MASK) >> DATA_FIELD_SHIFT);

	//dev_dbg(&spi->dev, "%s: addr: 0x%02x request: %012llx resp: %012llx", __FUNCTION__, address, request, response);

	if (!sch16xx_is_crc_valid(spi, response)) {
		dev_err(&spi->dev, "CRC error");
		return -EPROTO;
	}
	if (!sch16xx_is_response_valid(spi, response, request)) {
		dev_err(&spi->dev, "TA SA mismatch");
		return -EPROTO;
	}
	return 0;
}


static int sch16xx_write_single(struct sch16xx_dev *chip, unsigned int address, u32 data, bool check_crc)
{
	int ret;
	__be64 rx = 0;
	u64 response;
	struct spi_device *spi = chip->spi;
	u64 request = create_write_frame(address, chip->ta, data);
	__be64 tx = cpu_to_be64(request);

	struct spi_transfer t[] = {
		{
			.tx_buf = ((u8*)&tx) + 2,
			.rx_buf = ((u8*)&rx) + 2,
			.len = (48 / 8),
			.bits_per_word = 8,
			.delay = {
				.value = 10,
				.unit = SPI_DELAY_UNIT_USECS,
			},
		},
	};

	ret = spi_sync_transfer(spi, t, ARRAY_SIZE(t));
	if (ret)
		return ret;

	response = be64_to_cpu(rx);

	//dev_dbg(&spi->dev, "%s: addr: 0x%02x data: 0x%04x request: %012llx resp: %012llx", __FUNCTION__,
	//	address, data, request, response);

	if (check_crc && !sch16xx_is_crc_valid(spi, response))
		return -EPROTO;

	return 0;
}

static int sch16xx_reg_access(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int writeval, unsigned int *readval)
{
	struct sch16xx_dev *dev = iio_priv(indio_dev);

	if (readval) {
		return sch16xx_read_single(dev, reg, readval, true);
	} else {
		return sch16xx_write_single(dev, reg, writeval, true);
	}
}

static int sch16xx_read_serial_number(struct sch16xx_dev *dev,
				      char *buf, int size)
{
	int ret = 0;
	u32 sn_id1, sn_id2, sn_id3;
	ret = sch16xx_read_single(dev, REG_SN_ID1, &sn_id1, true);
	if (ret)
		return ret;

	ret = sch16xx_read_single(dev, REG_SN_ID2, &sn_id2, true);
	if (ret)
		return ret;

	ret = sch16xx_read_single(dev, REG_SN_ID3, &sn_id3, true);
	if (ret)
		return ret;

	if (buf != NULL) {
		ret = snprintf(buf, size,"%05d%01X%04XH01",
		 (int)(0xffff & (sn_id2)),
		 (unsigned)(0xf & (sn_id1)),
		 (unsigned)(0xffff & (sn_id3))
		);
	}
	return 0;
}

#define STAT_OK_VALUE 0xFFFF

static int read_status_registers(struct iio_dev *indio_dev, bool *status_failed)
{
	struct sch16xx_dev *sch16xx = iio_priv(indio_dev);
	const unsigned int regs[] = {
		REG_STAT_SUM,
		REG_STAT_SUM_SAT,
		REG_STAT_COM,
		REG_STAT_RATE_COM,
		REG_STAT_RATE_X,
		REG_STAT_RATE_Y,
		REG_STAT_RATE_Z,
		REG_STAT_ACC_X,
		REG_STAT_ACC_Y,
		REG_STAT_ACC_Z
	};
	int i;
	if (status_failed != NULL)
		*status_failed = false;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		unsigned int value;
		int ret = sch16xx_read_single(sch16xx, regs[i], &value, true);

		if (ret)
			return ret;

		if ((status_failed != NULL) && (value != STAT_OK_VALUE)) {
			dev_warn(&indio_dev->dev, "Status register 0x%x = 0x%04x", regs[i], value);
			*status_failed = true;
		}
	}
	return 0;
}

static int sch16xx_reset(struct iio_dev *indio_dev)
{
	struct sch16xx_dev *chip = iio_priv(indio_dev);

	if (chip->reset_gpio) {
		gpiod_set_value_cansleep(chip->reset_gpio, 0);
		msleep(2);
		gpiod_set_value_cansleep(chip->reset_gpio, 1);
		return 0;
	}

	return sch16xx_write_single(chip, REG_CTRL_RESET, SOFTRESET_CTRL, false);
}

static int sch16xx_init(struct iio_dev *indio_dev)
{
	unsigned int value;
	struct sch16xx_dev *chip = iio_priv(indio_dev);
	const struct dynamic_range_params *dyn;
	const struct filter_params *filt;
	int ret;
	bool status_failed;

	// Start up sequence
	//   power on
	//   wait 1ms
	//   wait 32ms <- restart from here if soft reset or EXTRESN
	//   set user controls (filter, decimation, SPI)
	//   write EN_SENSOR = 1
	//   wait 215ms
	//   read all status registers once, no critization
	//   write EOI=1
	//   wait 3ms
	//   read all status registers twice, check for OK status
	//     reset if not OK ->
	//   read all configuration registers and ensure desired (written) values
	//     reset if not OK ->
	// Done

	int retry;
	for (retry = 0; retry < 3; retry++) {

		ret = sch16xx_reset(indio_dev);
		if (ret)
			return ret;
		msleep(32);

		// rate filter
		// do not check the first frame CRC after reset
		filt = chip->rate_filter;
		value = filt->bit_value | filt->bit_value << 3 | filt->bit_value << 6;
		ret = sch16xx_write_single(chip, REG_CTRL_FILT_RATE, value, false);
		if (ret)
			return ret;

		// acc filter
		filt = chip->acc12_filter;
		value = filt->bit_value | filt->bit_value << 3 | filt->bit_value << 6;
		ret = sch16xx_write_single(chip, REG_CTRL_FILT_ACC12, value, true);
		if (ret)
			return ret;

		// rate dynamic range
		dyn = chip->rate_range;
		if (dyn == NULL)
			return -EINVAL;

		value = dyn->bit_value << 12;
		ret = sch16xx_write_single(chip, REG_CTRL_RATE, value, true);
		if (ret)
			return ret;

		// acc dynamic range
		dyn = chip->acc12_range;
		if (dyn == NULL)
			return -EINVAL;

		value = (dyn->bit_value << 12) | (dyn->bit_value << 9);
		ret = sch16xx_write_single(chip, REG_CTRL_ACC12, value, true);
		if (ret)
			return ret;

		// user interface control
		value = FTREE_TDEL_2_5_MS | MISO_SR_CTRL | DRY_SR_CTRL;

		// If sync_gpio is defined, enable SYNC operation
		if (chip->sync_gpio != NULL) {
			value |= SYNC_TOC_TH_10_MS | SYNC_DEC_EN | SYNC_INTP_EN;
		}

		// Set SPI_MISO and DRY buffer supply rate, if VDDIO is 1V8
		if (chip->vddio_1v8)
			value |= SPI_SUPPLY_1V8;

		ret = sch16xx_write_single(chip, REG_CTRL_USER_IF, value, true);
		if (ret)
			return ret;

		// set EN_SENSOR = 1
		ret = sch16xx_write_single(chip, REG_CTRL_MODE, EN_SENSOR, true);
		if (ret)
			return ret;

		msleep(250);// 215);

		ret = read_status_registers(indio_dev, NULL);
		if (ret)
			return ret;

		ret = sch16xx_write_single(chip, REG_CTRL_MODE, EN_SENSOR | EOI_CTRL, true);
		if (ret)
			return ret;
		msleep(3);

		ret = read_status_registers(indio_dev, NULL);
		if (ret)
			return ret;

		ret = read_status_registers(indio_dev, &status_failed);
		if (ret)
			return ret;

		if (status_failed)
			dev_warn(&indio_dev->dev, "Fault in status registers, start attept %d.", retry + 1);
		else
			return 0;
	}
	dev_err(&indio_dev->dev, "Failed to start the sensor.");
	return -EIO;
}

static int sch16xx_single_conversion(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val)
{
	struct sch16xx_dev *sch16xx = iio_priv(indio_dev);
	int value;
	u32 uvalue;

	int ret = sch16xx_read_single(sch16xx, chan->address, &uvalue, true);
	if (ret)
		return ret;

	value = sign_extend32(uvalue, 19);

	*val = value;

	return IIO_VAL_INT;
}

static int sch16xx_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val,
		int *val2, long info)
{
	struct sch16xx_dev *chip = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return sch16xx_single_conversion(indio_dev, chan, val);
	case IIO_CHAN_INFO_SCALE:
	{
		switch (chan->type) {
		case IIO_ANGL_VEL:
			*val = 0;
			*val2 = (IIO_DEGREE_TO_RAD(1000000000) + chip->rate_range->sensitivity_20bits / 2) / chip->rate_range->sensitivity_20bits;
			return IIO_VAL_INT_PLUS_NANO;
		case IIO_ACCEL:
			*val = 0;
			*val2 = (1000000000 + chip->acc12_range->sensitivity_20bits / 2) / chip->acc12_range->sensitivity_20bits;
			return IIO_VAL_INT_PLUS_NANO;
		case IIO_TEMP:
			// sensitivity is 100LSB/degC
			// additionally 20bit data must be shifted right by 4 bits
			*val = 0;
			*val2 = 1000000000 / 100 / 16;
			return IIO_VAL_INT_PLUS_NANO;
		default:
			return -EINVAL;
		}
	}
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
	{
		switch(chan->type) {
		case IIO_ANGL_VEL:
			*val = chip->rate_filter->cutoff_hz;
			return IIO_VAL_INT;
		case IIO_ACCEL:
			*val = chip->acc12_filter->cutoff_hz;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	}

	default:
		return -EINVAL;
	}
}

static int sch16xx_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val,
		int val2, long mask)
{
	int ret;
	struct sch16xx_dev *chip = iio_priv(indio_dev);

	switch(mask) {
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
	{
		const struct filter_params *filter;
		switch(chan->type) {
		case IIO_ANGL_VEL:
			filter = find_filter_params_hz(filter_params, ARRAY_SIZE(filter_params), val);
			if (filter == NULL)
				return -EINVAL;
			chip->rate_filter = filter;
			break;
		case IIO_ACCEL:
			filter = find_filter_params_hz(filter_params, ARRAY_SIZE(filter_params), val);
			if (filter == NULL)
				return -EINVAL;
			chip->acc12_filter = filter;
			break;
		default:
			return -EINVAL;
		}
		break;
	}
	default:
		return -EINVAL;
	}

	// Reinit the sensor to use the new settings
	ret = sch16xx_init(indio_dev);
	
	return ret;
}

static int sch16xx_read_avail(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
	const int **vals, int *type, int *length, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		*vals = sch16xx_filter_freqs;
		*length = ARRAY_SIZE(sch16xx_filter_freqs);
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static ssize_t serial_number_show(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sch16xx_dev *chip = iio_priv(indio_dev);
	char serial_number[20];
	int ret = sch16xx_read_serial_number(chip, serial_number, sizeof(serial_number));

	if (ret)
		return ret;

	return sysfs_emit(buf, "%s\n", serial_number);
}

static IIO_DEVICE_ATTR_RO(serial_number, 0);

static ssize_t accel_range_show(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sch16xx_dev *chip = iio_priv(indio_dev);

	if (chip->acc12_range == NULL)
		return -EINVAL;

	return sysfs_emit(buf, "%s\n", chip->acc12_range->measurement_range);
}

static IIO_DEVICE_ATTR_RO(accel_range, 0);

static ssize_t anglvel_range_show(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sch16xx_dev *chip = iio_priv(indio_dev);

	if (chip->rate_range == NULL)
		return -EINVAL;

	return sysfs_emit(buf, "%s\n", chip->rate_range->measurement_range);
}

static IIO_DEVICE_ATTR_RO(anglvel_range, 0);

static ssize_t product_code_show(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sch16xx_dev *chip = iio_priv(indio_dev);

	if (chip->product_code == NULL) {
		return sysfs_emit(buf, "Unknown (COMP_ID=0x%04x)", chip->comp_id);
	} else {
		return sysfs_emit(buf, "%s\n", chip->product_code);
	}
}

static IIO_DEVICE_ATTR_RO(product_code, 0);

static ssize_t dyn_show(char *buf, size_t size, const struct dynamic_range_params *dyn_table, const struct dynamic_range_params *selected)
{
	int i = 0;
	int len = 0;

	while (dyn_table[i].dyn_n != NULL) {

		bool hit = selected == &dyn_table[i];

		len += snprintf(buf + len, size - len, "%s%s%s%s",
				len == 0 ? "" : " ", hit ? "[" : "",
				dyn_table[i].dyn_n, hit ? "]" : "");

		i++;
	}
	len += snprintf(buf + len, size - len, "\n");

	return len;
}

static ssize_t rate_dyn_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sch16xx_dev *chip = iio_priv(indio_dev);

	return dyn_show(buf, PAGE_SIZE, rate_params, chip->rate_range);
}

static ssize_t rate_dyn_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sch16xx_dev *chip = iio_priv(indio_dev);
	const struct dynamic_range_params *dyn;

	dev_info(dev, "buf= >%s<", buf);

	dyn = find_dynamic_range_params(rate_params, buf);

	if (dyn == NULL)
		return -EINVAL;

	chip->rate_range = dyn;

	// Reinit the sensor to use the new settings
	ret = sch16xx_init(indio_dev);
	if (ret < 0)
		return ret;

	return count;
}

static IIO_DEVICE_ATTR_RW(rate_dyn, 0);

static ssize_t accel_dyn_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sch16xx_dev *chip = iio_priv(indio_dev);

	return dyn_show(buf, PAGE_SIZE, acc12_params, chip->acc12_range);
}

static ssize_t accel_dyn_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sch16xx_dev *chip = iio_priv(indio_dev);

	const struct dynamic_range_params *dyn = find_dynamic_range_params(acc12_params, buf);

	if (dyn == NULL)
		return -EINVAL;

	chip->acc12_range = dyn;

	// Reinit the sensor to use the new settings
	ret = sch16xx_init(indio_dev);
	if (ret < 0)
		return ret;

	return count;
}

static IIO_DEVICE_ATTR_RW(accel_dyn, 0);

static struct attribute *sch16xx_attributes[] = {
	&iio_dev_attr_serial_number.dev_attr.attr,
	&iio_dev_attr_accel_range.dev_attr.attr,
	&iio_dev_attr_anglvel_range.dev_attr.attr,
	&iio_dev_attr_product_code.dev_attr.attr,
	&iio_dev_attr_rate_dyn.dev_attr.attr,
	&iio_dev_attr_accel_dyn.dev_attr.attr,
	NULL,
};

static const struct attribute_group sch16xx_attribute_group = {
	.attrs = sch16xx_attributes,
};

static irqreturn_t sch16xx_trigger_top_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct sch16xx_dev *sch16xx_dev = iio_priv (indio_dev);

	// Set SYNC to freeze output values
	if (sch16xx_dev->sync_gpio != NULL)
		gpiod_set_value(sch16xx_dev->sync_gpio, 1);

	// Record the timestamp at the time of setting SYNC
	pf->timestamp = iio_get_time_ns(indio_dev);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t sch16xx_trigger_bottom_handler(int irq, void *p)
{
	int i, ret;
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct sch16xx_dev *sch16xx_dev = iio_priv (indio_dev);
	u32 data[SCH16XX_MAX_TRANSFER_COUNT];

	ret = spi_sync_transfer(sch16xx_dev->spi, sch16xx_dev->transfer,
				sch16xx_dev->transfer_size);

	// Clear SYNC to unfreeze output
	if (sch16xx_dev->sync_gpio != NULL)
		gpiod_set_value(sch16xx_dev->sync_gpio, 0);

	if (ret) {
		dev_err_ratelimited(&sch16xx_dev->spi->dev, "Error in SPI transfer: %d", ret);
		goto out;
	}
	// Check the result frames validity and copy payload to data[]
	for (i = 0; i < sch16xx_dev->transfer_size - 1; i++) {
		u64 response, request;

		request = be64_to_cpu(sch16xx_dev->tx[i]) >> 16;
		response = be64_to_cpu(sch16xx_dev->rx[i+1]) >> 16;

		if (!sch16xx_is_crc_valid(sch16xx_dev->spi, response)) {
			dev_err_ratelimited(&sch16xx_dev->spi->dev, "CRC error in frame, request=0x%012llx, response=0x%012llx", request, response);
			goto out;
		}

		if (!sch16xx_is_response_valid(sch16xx_dev->spi, response, request)) {
			dev_err_ratelimited(&sch16xx_dev->spi->dev, " Response frame not valid, request=0x%012llx, response=0x%012llx",
				request, response);
			goto out;
		}

		data[i] = (u32)((response & DATA_FIELD_MASK) >> DATA_FIELD_SHIFT);
	}

	if (sch16xx_dev->sync_gpio != NULL && (data[sch16xx_dev->active_scan_size] & (S_SYNC_ACTIVE_ACC1 | S_SYNC_ACTIVE_RATE1)) !=
		((S_SYNC_ACTIVE_ACC1 | S_SYNC_ACTIVE_RATE1))) {
		dev_warn_ratelimited(&sch16xx_dev->spi->dev, "Sync timeout, sync status = 0x%04x", data[sch16xx_dev->active_scan_size]);
	}

	iio_push_to_buffers_with_timestamp (indio_dev, &data, pf->timestamp);

out:
	iio_trigger_notify_done (indio_dev->trig);

	return IRQ_HANDLED;
}

static void prepare_transfer(struct sch16xx_dev *chip, int transfer_num, u64 tx, bool last)
{
	const struct spi_delay delay = {
		.value = 100,
		.unit = SPI_DELAY_UNIT_USECS,
	};
	chip->tx[transfer_num] = cpu_to_be64(tx << 16);
	chip->transfer[transfer_num].tx_buf = &(chip->tx[transfer_num]);
	chip->transfer[transfer_num].len = (48 / 8);
	chip->transfer[transfer_num].bits_per_word = 8;
	chip->transfer[transfer_num].cs_change_delay = delay;
	chip->transfer[transfer_num].cs_change = last ? 0 : 1;
//	chip->transfer[transfer_num].rx_buf = transfer_num == 0 ? NULL : &chip->rx[transfer_num - 1];
	chip->transfer[transfer_num].rx_buf = &chip->rx[transfer_num];
}

static int sch16xx_update_scan_mode(struct iio_dev *indio_dev, const unsigned long *scan_mask)
{
	int chan;
	struct sch16xx_dev *sch16xx_dev = iio_priv(indio_dev);

	sch16xx_dev->transfer_size = 0;
	memset(sch16xx_dev->transfer, 0, sizeof(sch16xx_dev->transfer));

	// Read output channels
	for (chan = 0; chan < indio_dev->num_channels; chan++) {
		int scan_index = indio_dev->channels[chan].scan_index;
		if (test_bit(scan_index, scan_mask)) {
			int address = indio_dev->channels[chan].address;
			prepare_transfer(sch16xx_dev, sch16xx_dev->transfer_size++,
				create_read_frame(address, sch16xx_dev->ta), false);
		}
	}
	sch16xx_dev->active_scan_size = sch16xx_dev->transfer_size;

	// Read status channels
	prepare_transfer(sch16xx_dev, sch16xx_dev->transfer_size++,
				create_read_frame(REG_STAT_SYNC_ACTIVE, sch16xx_dev->ta), false);

	// dummy frame for off-frame protocol
	prepare_transfer(sch16xx_dev, sch16xx_dev->transfer_size++,
				create_read_frame(REG_TEMP, sch16xx_dev->ta), true);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sch16xx_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	return sch16xx_reset(indio_dev);
}

static int sch16xx_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	return sch16xx_init(indio_dev);
}
#endif

static const struct dev_pm_ops sch16xx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sch16xx_suspend, sch16xx_resume)
};

static const struct iio_info sch16xx_info = {
	.read_raw = sch16xx_read_raw,
	.write_raw = sch16xx_write_raw,
	.attrs = &sch16xx_attribute_group,
	.update_scan_mode = sch16xx_update_scan_mode,
	.debugfs_reg_access = sch16xx_reg_access,
	.read_avail = sch16xx_read_avail,
};

static int sch16xx_probe (struct spi_device *spi)
{
	int ret = 0;
	struct iio_dev *iio_dev;
	struct sch16xx_dev *chip;
	const char *property_string;

	crc8_populate_msb(sch16xx_crc8_table, SCH16XX_CRC8_POLYNOMIAL);

	iio_dev = devm_iio_device_alloc (&spi->dev, sizeof(*chip));
	if (!iio_dev)
		goto err;

	chip = iio_priv (iio_dev);
	chip->spi = spi;

	iio_dev->dev.parent = &spi->dev;
	iio_dev->name = "sch16xx";
	iio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
	iio_dev->info = &sch16xx_info;
	iio_dev->channels = sch16xx_channels;
	iio_dev->num_channels = ARRAY_SIZE (sch16xx_channels);

	ret = devm_iio_triggered_buffer_setup (&spi->dev, iio_dev, sch16xx_trigger_top_handler,
						sch16xx_trigger_bottom_handler, NULL);
	if (ret)
		goto err;

	ret = devm_iio_device_register(&spi->dev, iio_dev);
	if (ret)
		goto err;

	if (of_property_read_u32(spi->dev.of_node, "murata,ta", &chip->ta) == 0) {
		if (chip->ta > 3) {
			dev_err(&spi->dev, "Invalid TA value %u, valid values [0...3].", chip->ta);
			return -EINVAL;
		}
	} else {
		chip->ta = 0;
	}
	if (strlen(override_accel_dyn_range) > 0) {
		property_string = override_accel_dyn_range;
	}
	else if (of_property_read_string(spi->dev.of_node, "murata,accel_dyn", &property_string) != 0) {
		property_string = default_accel_dyn_range;
	}

	chip->acc12_range = find_dynamic_range_params(acc12_params, property_string);

	if (chip->acc12_range == NULL) {
		dev_err(&spi->dev, "Accelerometer range '%s' not found.", property_string);
		ret = -EINVAL;
		goto err;
	}

	if (strlen(override_gyro_dyn_range) > 0) {
		property_string = override_gyro_dyn_range;
	}
	else if (of_property_read_string(spi->dev.of_node, "murata,rate_dyn", &property_string) != 0) {
		property_string = default_gyro_dyn_range;
	}

	chip->rate_range = find_dynamic_range_params(rate_params, property_string);

	if (chip->rate_range == NULL) {
		dev_err(&spi->dev, "Gyroscope range '%s' not found.", property_string);
		ret = -EINVAL;
		goto err;
	}

	if (of_property_read_string(spi->dev.of_node, "murata,accel_filter", &property_string) == 0) {
		chip->acc12_filter = find_filter_params(filter_params, property_string);
	} else {
		chip->acc12_filter = find_filter_params(filter_params, "LPF0");
	}
	if (chip->acc12_filter == NULL) {
		dev_err(&spi->dev, "Accelerometer filter parameters not found.");
		ret = -EINVAL;
		goto err;
	}
	if (of_property_read_string(spi->dev.of_node, "murata,rate_filter", &property_string) == 0) {
		chip->rate_filter = find_filter_params(filter_params, property_string);
	} else {
		chip->rate_filter = find_filter_params(filter_params, "LPF0");
	}
	if (chip->rate_filter == NULL) {
		dev_err(&spi->dev, "Gyroscope filter parameters for not found.");
		ret = -EINVAL;
		goto err;
	}

	chip->sync_gpio = devm_gpiod_get_optional(&spi->dev, "sync", GPIOD_OUT_LOW);
	if (IS_ERR(chip->sync_gpio)) {
		dev_err(&spi->dev, "Error reserving SYNC GPIO");
		return PTR_ERR(chip->sync_gpio);
	}
	if (chip->sync_gpio == NULL) {
		dev_info(&spi->dev, "SYNC GPIO not defined, time domain performance degraded.");
	}

	chip->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(chip->reset_gpio)) {
		dev_err(&spi->dev, "Error reserving EXTRESN GPIO");
		return PTR_ERR(chip->reset_gpio);
	}
	if (chip->reset_gpio == NULL) {
		dev_info(&spi->dev, "EXTRESN GPIO not defined, using soft reset instead.");
	}

	chip->vddio_1v8 = of_property_read_bool(spi->dev.of_node, "murata,vddio_1v8");

	{
		unsigned int id;
		ret = sch16xx_read_single(chip, REG_COMP_ID, &id, false);
		if (ret)
			goto err;
		chip->comp_id = id;
		chip->product_code = find_product_code(id);
	}

	ret = sch16xx_init(iio_dev);
	if (ret)
		goto err;

	return ret;
err:
	dev_err(&spi->dev, "Error in %s. %d", __FUNCTION__, ret);
	return ret;
}

static const struct of_device_id sch16xx_match[] =
{
	{ .compatible = "murata,sch16xx" },
	{ },
};

MODULE_DEVICE_TABLE ( of, sch16xx_match);

static struct spi_driver sch16xx_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "sch16xx",
		.pm = &sch16xx_pm_ops,
		.of_match_table = sch16xx_match,
	},
	.probe = sch16xx_probe,
};

module_spi_driver(sch16xx_driver);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("mikko.pynnonen@murata.com");
