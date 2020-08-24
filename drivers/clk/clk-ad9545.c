// SPDX-License-Identifier: GPL-2.0
/*
 * AD9545 Network Clock Generator/Synchronizer
 *
 * Copyright 2020 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <dt-bindings/clock/ad9545.h>
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define AD9545_PRODUCT_ID_LOW		0x0004
#define AD9545_PRODUCT_ID_HIGH		0x0005

#define AD9545_CHIP_ID			0x0121
#define AD9545_SYS_CLK_FB_DIV		0x0200
#define AD9545_SYS_CLK_INPUT		0x0201
#define AD9545_SYS_CLK_REF_FREQ		0x0202
#define AD9545_REF_A_CTRL		0x0300
#define AD9545_REF_B_CTRL		0x0304
#define AD9545_REF_A_RDIV		0x0400
#define AD9545_REF_A_PERIOD		0x0404
#define AD9545_DRIVER_0A_CONF		0x10D7
#define AD9545_DRIVER_1A_CONF		0x14D7
#define AD9545_POWER_DOWN_REF		0x2001

#define AD9545_REF_CTRL_DIF_MSK			GENMASK(3,2)
#define AD9545_REF_CTRL_REFA_MSK		GENMASK(5,4)
#define AD9545_REF_CTRL_REFAA_MSK		GENMASK(7,6)

#define AD9545_REF_X_RDIV(x)			(AD9545_REF_A_RDIV + x * 0x20)
#define AD9545_REF_X_PERIOD(x)			(AD9545_REF_A_PERIOD + x * 0x20)

static const unsigned short AD9545_regs[][2] = {
//	{0x0200, 0x18},	/* System Clock Registers Details */
//	{0x0201, 0x09},
//	{0x0203, 0x00},
//	{0x0204, 0xB0},
//	{0x0205, 0x71},
//	{0x0206, 0x0B},
	{0x0280, 0x05}, /* SYSCLK Compensation Register */
	{0x0282, 0x05},
	{0x0285, 0xF4},
	{0x0286, 0x01}, /* todo in the future ^^^ */
//	{0x0300, 0x10}, /* Reference General A Registers */
//	{0x0304, 0x20}, /* Reference General B Register */
//	{0x0400, 0x31}, /* Reference Input A Registers */
//	{0x0405, 0xE8}, /* REFA nominal period [15:8] */
//	{0x0406, 0x64},
//	{0x0407, 0xA7},
//	{0x0408, 0xB3},
//	{0x0409, 0xB6},
//	{0x040A, 0xE0},
//	{0x040B, 0x0D}, /* REFA nominal period [59:56] */
//	{0x040C, 0x40}, /* REFA offset limit [7:0] */
//	{0x040D, 0x42}, /* REFA offset limit [15:8] */
//	{0x040E, 0x0F}, /* REFA offset limit [23:16] */
//	{0x0420, 0x31}, /* Reference Input AA Registers */
//	{0x0425, 0xE8},
//	{0x0426, 0x76},
//	{0x0427, 0x48},
//	{0x0428, 0x17},
//	{0x0446, 0x64}, /* Reference Input B Registers */
//	{0x0447, 0xA7},
//	{0x0448, 0xB3},
//	{0x0449, 0xB6},
//	{0x044A, 0xE0},
//	{0x044B, 0x0D},
//	{0x0460, 0x00}, /* Reference Input BB Registers */
//	{0x0465, 0x00},
//	{0x0466, 0x64},
//	{0x0467, 0xA7},
//	{0x0468, 0xB3},
//	{0x0469, 0xB6},
//	{0x046A, 0xE0},
//	{0x046B, 0x0D},
	{0x0800, 0x58}, /* Source Profile 0 A Registers */
	{0x0801, 0x1B},
	{0x0803, 0xC8},
	{0x0804, 0xC8},
	{0x0805, 0x88},
	{0x0806, 0x13},
	{0x0808, 0xC8},
	{0x0809, 0xC8},
	{0x0820, 0x88}, /* Source Profile 1 AA Registers */
	{0x0821, 0x13},
	{0x0823, 0xC8},
	{0x0824, 0xC8},
	{0x0825, 0x88},
	{0x0826, 0x13},
	{0x0828, 0xC8},
	{0x0829, 0xC8},
	{0x0883, 0x64}, /* Source Profile 4 NCO 0 Registers */
	{0x0884, 0x64},
	{0x0888, 0x64},
	{0x0889, 0x64},
	{0x08A3, 0xC8}, /* Source Profile 5 NCO 1 Registers */
	{0x08A4, 0xC8},
	{0x08A8, 0xC8},
	{0x08A9, 0xC8},
	{0x1000, 0x55}, /* DPLL Channel 0 Registers */
	{0x1001, 0x55},
	{0x1002, 0x55},
	{0x1003, 0x55},
	{0x1004, 0x55},
	{0x10C4, 0x53}, /* Distribution General 0 Registers */
	{0x10C5, 0x07},
	{0x10C6, 0x90},
	{0x10C7, 0x2F},
	{0x10C8, 0x50},
	{0x10C9, 0x09},
	{0x10CA, 0x90},
	{0x10CB, 0x2F},
	{0x10CC, 0x50},
	{0x10CD, 0x09},
	{0x10D7, 0x02},
	{0x10D8, 0x0C},
	{0x10DB, 0x05},
	{0x1100, 0x0A}, /* Distribution Divider Q0A Registers */
	{0x1201, 0x05}, /* DPLL Translation Profile 0.0 Registers */
	{0x1204, 0x50},
	{0x1205, 0xC3},
	{0x1208, 0xFF},
	{0x1209, 0xFF},
	{0x120A, 0x52},
	{0x120B, 0x07},
	{0x120C, 0xFF},
	{0x120D, 0x05},
	{0x1210, 0x01},
	{0x1213, 0x02},
	{0x1216, 0x01},
	{0x1217, 0x76},
	{0x1400, 0x8E}, /* DPLL Channel 1 Registers */
	{0x1401, 0xE3},
	{0x1402, 0x38},
	{0x1403, 0x8E},
	{0x1404, 0xE8},
	{0x1405, 0x21},
	{0x1481, 0x0C}, /* APLL Channel 1 Registers */
	{0x14C2, 0x90}, /* Distribution General 1 Register */
	{0x14C3, 0x2F},
	{0x14C4, 0x50},
	{0x14C5, 0x09},
	{0x14D7, 0x02},
	{0x14D8, 0x02},
	{0x14DB, 0x05},
	{0x1500, 0x0C}, /* Distribution Divider 1 A Registers */
	{0x1508, 0x07},
	{0x1512, 0x06}, /* Distribution Divider 1 B Registers */
	{0x151A, 0x07},
	{0x1601, 0x03}, /* DPLL Translation Profile 1.0 Registers */
	{0x1604, 0x50},
	{0x1605, 0xC3},
	{0x1608, 0x8F},
	{0x1609, 0x2F},
	{0x160A, 0x50},
	{0x160B, 0x09},
	{0x160C, 0x1F},
	{0x160D, 0x5F},
	{0x160E, 0xA0},
	{0x160F, 0x12},
	{0x2001, 0x0F}, /* IRQ Map DPLL0 Clear Registers - power down references */
	{0x2101, 0x08}, /* Operational Control Channel 0 Registers */
	{0x2103, 0x10},
	{0x2104, 0x10},
	{0x2201, 0x08}, /* Operational Control Channel 1 Registers */
	{0x2804, 0x80}, /* Auxiliary NCO 0 Registers */
	{0x2809, 0x80},
	{0x3001, 0x03}, /* Status Readback Register */
	{0x3002, 0x05},
	{0x3003, 0xB8},
	{0x3004, 0x11},
	{0x3005, 0x0B},
	{0x3006, 0x0B},
	{0x3007, 0x0B},
	{0x3008, 0x0B},
	{0x3100, 0x28}, /* STATUS_READBACK_PLL_0 Registers */
	{0x3101, 0x01},
	{0x3200, 0x28}, /* STATUS_READBACK_PLL_1 Register */
	{0x3201, 0x01},
	{0x3A00, 0x3B}, /* TDC_AUXILIARY_READ Register */
	{0x3A01, 0x44},
	{0x3A02, 0x44},
	{0x3A03, 0xEA},
	{0x3A04, 0xAB},
	{0x3A05, 0x4F},
	{0x3A0A, 0xFF},
	{0x3A0B, 0xFF},
	{0x3A0C, 0xFF},
	{0x3A0D, 0xFF},
	{0x3A0E, 0xFF},
	{0x3A0F, 0x01},
	{0x3A1E, 0xFF},
	{0x3A2A, 0xFF},
};

static const unsigned short register_write_seq[][2] = {
	{0x000F, 0x01}, /* IO_UPDATE */
	{0x2000, 0x02}, /* IRQ Map DPLL0 Clear Register - calibrate all */
	{0x000F, 0x01},
	{0x000F, 0x01},
	{0x000F, 0x01},
	{0x2000, 0x02},
	{0x000F, 0x01},
	{0x000F, 0x01},
};

static const u32 ad9545_hyst_scales_bp[] = {
	0, 3125, 6250, 12500, 25000, 50000, 75000, 87500
};

static const u32 ad9545_out_source_ua[] = {
	7500, 12500, 15000
};

enum ad9545_ref_mode {
	AD9545_SINGLE_ENDED = 0,
	AD9545_DIFFERENTIAL,
};

enum ad9545_single_ended_config {
	AD9545_AC_COUPLED_IF = 0,
	AD9545_DC_COUPLED_1V2,
	AD9545_DC_COUPLED_1V8,
	AD9545_IN_PULL_UP,
};

enum ad9545_diferential_config {
	AD9545_AC_COUPLED = 0,
	AD9545_DC_COUPLED,
	AD9545_DC_COUPLED_LVDS,
};

enum ad9545_output_mode {
	AD9545_SINGLE_DIV_DIF = 0,
	AD9545_SINGLE_DIV,
	AD9545_DUAL_DIV,
};

struct ad9545_out_clk {
	bool				output_used;
	bool				sink_current;
	enum ad9545_output_mode 	output_mode;
	u32 				source_ua;
};

struct ad9545_ref_in_clk {
	u32				r_div_ratio;
	u32				nominal_f_hz;
	bool				ref_used;
	u32				d_tol_ppb;
	u8				monitor_hyst_scale;
	u32				valid_t_ms;
	enum ad9545_ref_mode		mode;
	union {
		enum ad9545_single_ended_config		s_conf;
		enum ad9545_diferential_config		d_conf;
	};
};

struct ad9545_sys_clk {
	bool				sys_clk_freq_doubler;
	bool				sys_clk_crystal;
	u32				ref_freq_mhz[2];
};

struct ad9545_state {
	struct i2c_client		*client;
	struct regmap 			*regmap;
	struct ad9545_sys_clk		sys_clk;
	struct ad9545_ref_in_clk	ref_in_clks[4];
	struct ad9545_out_clk		out_clks[10];
};

static const struct regmap_config ad9545_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8,
};

static int ad9545_parse_dt_inputs(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	bool prop_found;
	int ref_ind;
	u32 val;
	int ret;
	int i;

	fwnode = dev_fwnode(&st->client->dev);

	prop_found = false;
	fwnode_for_each_child_node(fwnode, child) {
		if (!fwnode_property_present(child, "adi,ref-nominal-frequency-hz"))
			continue;

		ret = fwnode_property_read_u32(child, "reg", &ref_ind);
		if (ret < 0)
			return ret;

		if (ref_ind > 3)
			return -EINVAL;

		st->ref_in_clks[ref_ind].ref_used = true;
		if ((prop_found = fwnode_property_present(child, "adi,single-ended-mode"))) {
			st->ref_in_clks[ref_ind].mode = AD9545_SINGLE_ENDED;
			ret = fwnode_property_read_u32(child, "adi,single-ended-mode", &val);
			if (ret < 0)
				return ret;

			st->ref_in_clks[ref_ind].s_conf = val;
		} else if ((prop_found = fwnode_property_present(child, "adi,differential-mode"))) {
			st->ref_in_clks[ref_ind].mode = AD9545_DIFFERENTIAL;
			ret = fwnode_property_read_u32(child, "adi,differential-mode", &val);
			if (ret < 0)
				return ret;

			st->ref_in_clks[ref_ind].d_conf = val;
		}

		if (!prop_found) {
			dev_err(&st->client->dev, "No mode specified for input reference.\n");
			return -EINVAL;
		}

		ret = fwnode_property_read_u32(child, "adi,r-divider-ratio", &val);
		if (!ret)
			st->ref_in_clks[ref_ind].r_div_ratio = val;

		ret = fwnode_property_read_u32(child, "ref-nominal-frequency-hz", &val);
		if (ret < 0)
			return ret;

		if (val == 0)
			return -EINVAL;

		st->ref_in_clks[ref_ind].nominal_f_hz = val;

		ret = fwnode_property_read_u32(child, "adi,ref-dtol-pbb", &val);
		if (ret < 0)
			return ret;
		st->ref_in_clks[ref_ind].d_tol_ppb = val;

		ret = fwnode_property_read_u32(child, "adi,ref-monitor-hysteresis-pbb", &val);
		if (ret < 0)
			return ret;

		for (i = 0; i < ARRAY_SIZE(ad9545_hyst_scales_bp); i++) {
			if (ad9545_hyst_scales_bp[i] == val) {
				st->ref_in_clks[ref_ind].monitor_hyst_scale = i;
				break;
			}
		}
		if (i == ARRAY_SIZE(ad9545_hyst_scales_bp))
			return -EINVAL;

		ret = fwnode_property_read_u32(child, "adi,ref-validation-timer-ms", &val);
		if (ret < 0)
			return ret;

		st->ref_in_clks[ref_ind].valid_t_ms = val;
	}

	return 0;
}

static int ad9545_parse_dt_outputs(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	bool prop_found;
	int ref_ind;
	u32 val;
	int ret;

	fwnode = dev_fwnode(&st->client->dev);

	prop_found = false;
	fwnode_for_each_child_node(fwnode, child) {
		if (!fwnode_property_present(child, "adi,output-mode"))
			continue;

		ret = fwnode_property_read_u32(child, "reg", &ref_ind);
		if (ret < 0)
			return ret;

		if (ref_ind > 9)
			return -EINVAL;

		st->out_clks[ref_ind].output_used = true;

		if (fwnode_property_present(child, "adi,current-sink"))
			st->out_clks[ref_ind].sink_current = true;

		ret = fwnode_property_read_u32(child, "adi,current-source-microamp", &val);
		if (ret < 0)
			return ret;

		st->out_clks[ref_ind].source_ua = val;

		ret = fwnode_property_read_u32(child, "adi,output-mode", &val);
		if (ret < 0)
			return ret;

		st->out_clks[ref_ind].output_mode = val;
	}

	return 0;
}

static int ad9545_parse_dt(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	int ret;

	fwnode = dev_fwnode(&st->client->dev);

	ret = fwnode_property_read_u32_array(fwnode, "adi,ref-frequency-mhz",
					     &st->sys_clk.ref_freq_mhz[0], 2);
	if (ret < 0)
		return ret;

	st->sys_clk.sys_clk_crystal = fwnode_property_present(fwnode, "adi,ref-crystal");

	ret = ad9545_parse_dt_inputs(st);
	if (ret < 0)
		return ret;

	ret = ad9545_parse_dt_outputs(st);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9545_check_id(struct ad9545_state *st)
{
	u32 chip_id;
	u32 val;
	int ret;

	ret = regmap_read(st->regmap, AD9545_PRODUCT_ID_LOW, &val);
	if (ret < 0)
		return ret;

	chip_id = val;
	ret = regmap_read(st->regmap, AD9545_PRODUCT_ID_HIGH, &val);
	if (ret < 0)
		return ret;

	chip_id += val << 8;
	if (chip_id != AD9545_CHIP_ID) {
		dev_err(&st->client->dev, "Unrecognized CHIP_ID 0x%X\n", chip_id);
		return -ENODEV;
	}

	return 0;
}

static int ad9545_sys_clk_setup(struct ad9545_state *st)
{
	u8 div_ratio;
	u8 buf[5];
	u32 fosc;
	int ret;
	u8 val;
	u32 fs;
	int i;

	/*
	 * System frequency must be between 2250 MHz and 2415 MHz.
	 * fs = fosc * K / j
	 * K - feedback divider ratio [4, 255]
	 * j = 1/2 if frequency doubler is enabled
	 */
	fosc = (st->sys_clk.ref_freq_mhz[0] / 1000000 + st->sys_clk.ref_freq_mhz[1] * 4294) / 1000;

	if (st->sys_clk.sys_clk_freq_doubler)
		fosc *= 2;

	div_ratio = 0;
	for (i = 4; i < 256; i++) {
		fs = i * fosc;

		if (fs > 2250 && fs < 2415) {
			div_ratio = i;
			break;
		}
	}

	if (!div_ratio) {
		dev_err(&st->client->dev, "No feedback divider ratio for sys clk PLL found.\n");
		return -EINVAL;
	}

	ret = regmap_write(st->regmap, AD9545_SYS_CLK_FB_DIV, div_ratio);
	if (ret < 0)
		return ret;

	/* enable crystal maintaining amplifier */
	val = 0;
	if (st->sys_clk.sys_clk_crystal)
		val |= BIT(3);

	if (st->sys_clk.sys_clk_freq_doubler)
		val |= BIT(0);

	ret = regmap_write(st->regmap, AD9545_SYS_CLK_INPUT, val);
	if (ret < 0)
		return ret;

	/* write reference frequency provided at XOA, XOB */
	for (i = 0; i < 4; i++)
		buf[i] = (st->sys_clk.ref_freq_mhz[0] >> (i * 8)) & 0xFF;

	buf[4] = st->sys_clk.ref_freq_mhz[1] & 0xFF;
	ret = regmap_bulk_write(st->regmap, AD9545_SYS_CLK_REF_FREQ, buf, 5);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9545_outputs_setup(struct ad9545_state *st)
{
	int out_i;
	u16 addr;
	int ret;
	u8 reg;
	int i;
	int j;

	/* configure current sources */
	for (i = 0; i < 5; i++) {
		if (st->out_clks[i * 2].output_used)
			out_i = i * 2;
		else if (st->out_clks[i * 2].output_used)
			out_i = i * 2 + 1;
		else
			continue;

		reg = 0;
		if (st->out_clks[ref_ind].sink_current)
			reg = 1;

		for (j = 0; j < ARRAY_SIZE(ad9545_out_source_ua); j++)
			if (ad9545_out_source_ua[j] == st->out_clks[out_i].source_ua)
				reg |= FIELD_PREP(GENMASK(2,1), i);

		reg |= FIELD_PREP(GENMASK(4, 3), st->out_clks[out_i].output_mode);

		if (i < 3)
			addr = AD9545_DRIVER_0A_CONF + i;
		else
			addr = AD9545_DRIVER_1A_CONF + (i - 3);

		ret = regmap_write(st->regmap, addr, reg);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9545_input_refs_setup(struct ad9545_state *st)
{
	u64 period_es;
	int ret;
	u32 val;
	u8 reg;
	int i;
	int j;

	/* configure REFA REFAA REFB REFBB */
	for (i = 0; i < 4; i += 2) {
		if (st->ref_in_clks[i].mode == AD9545_DIFFERENTIAL) {
			reg = BIT(0);
			reg |= FIELD_PREP(AD9545_REF_CTRL_DIF_MSK, st->ref_in_clks[i].d_conf);
		} else {
			reg = 0;
			reg |= FIELD_PREP(AD9545_REF_CTRL_REFA_MSK, st->ref_in_clks[i].s_conf);
			reg |= FIELD_PREP(AD9545_REF_CTRL_REFAA_MSK, st->ref_in_clks[i + 1].s_conf);
		}

		ret = regmap_write(st->regmap, AD9545_REF_A_CTRL + i * 2, reg);
		if (ret < 0)
			return ret;
	}

	/* configure refs r dividers */
	for (i = 0; i < 4; i++) {
		/* r-div ratios are mapped from 0 onward */
		val = st->ref_in_clks[i].r_div_ratio - 1;

		for (j = 0; j < 4; j++) {
			reg = (val >> (j * 8)) && 0xFF;

			ret = regmap_write(st->regmap, AD9545_REF_X_RDIV(i) + j, reg);
			if (ret < 0)
				return ret;
		}
	}

	/* specify nominal period on each ref */
	for (i = 0; i < 4; i++) {
		if (!st->ref_in_clks[i].ref_used) {
			/* write nominal period in attoseconds */
			period_es = 1000000000000000000;
			do_div(period_es, st->ref_in_clks[0].nominal_f_hz);

			for (j = 0; j < 8; j++) {
				reg = (period_es >> (j * 8)) && 0xFF;

				ret = regmap_write(st->regmap, AD9545_REF_X_PERIOD(i) + j, reg);
				if (ret < 0)
					return ret;
			}
		}
	}

	/* disable unused references */
	reg = 0;
	for (i = 0; i < 4; i++) {
		if (!st->ref_in_clks[i].ref_used)
			reg |= (1 << i);
	}

	ret = regmap_write(st->regmap, AD9545_POWER_DOWN_REF, reg);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9545_setup(struct ad9545_state *st)
{
	int ret;
	u32 val;
	int i;

	ret = ad9545_sys_clk_setup(st);
	if (ret < 0)
		return ret;

	ret = ad9545_input_refs_setup(st);
	if (ret < 0)
		return ret;

	ret = ad9545_outputs_setup(st);
	if (ret < 0)
		return ret;

	/* write rest of regs */
	for (i = 0; i < ARRAY_SIZE(AD9545_regs); i++) {
		ret = regmap_write(st->regmap, AD9545_regs[i][0], AD9545_regs[i][1]);
		if (ret < 0)
			return ret;
	}

	/* reset chip */
	for (i = 0; i < ARRAY_SIZE(register_write_seq); i++) {
		ret = regmap_write(st->regmap, register_write_seq[i][0], register_write_seq[i][1]);
		if (ret < 0)
			return ret;
	}

	msleep(100);

	/* check locks */
	/* TODO: err on unlocked pll */
	ret = regmap_read(st->regmap, 0x3001, &val);
	if (ret < 0)
		return ret;

	pr_info("DEBUG: ad9545: PLL status reg: %x", val);
	return 0;
}

static int ad9545_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ad9545_state *st;
	int ret;

	st = devm_kzalloc(&client->dev, sizeof(struct ad9545_state), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->client = client;
	st->regmap = devm_regmap_init_i2c(client, &ad9545_regmap_config);
	if (IS_ERR(st->regmap)) {
		dev_err(&client->dev, "devm_regmap_init_i2c failed!\n");
		return PTR_ERR(st->regmap);
	}

	ret = ad9545_check_id(st);
	if (ret < 0)
		return ret;

	ret = ad9545_parse_dt(st);
	if (ret < 0)
		return ret;

	ret = ad9545_setup(st);
	if (ret < 0)
		return ret;

	pr_info("DEBUG: AD9545 driver probed.\n");

	return 0;
}

static const struct of_device_id ad9545_of_match[] = {
	{ .compatible = "adi,ad9545" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad9545_of_match);

static const struct i2c_device_id ad9545_id[] = {
	{"ad9545", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad9545_id);

static struct i2c_driver ad9545_driver = {
	.driver = {
		.name	= "ad9545",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9545_probe,
	.id_table	= ad9545_id,
};
module_i2c_driver(ad9545_driver);

MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9545");
MODULE_LICENSE("GPL v2");
