// SPDX-License-Identifier: GPL-2.0
/*
 * AD9545 Network Clock Generator/Synchronizer
 *
 * Copyright 2020 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef _DT_BINDINGS_CLOCK_AD9545_H_
#define _DT_BINDINGS_CLOCK_AD9545_H_

/* Input Driver Mode
 * Use for adi,single-ended-mode: */
#define DRIVER_MODE_AC_COUPLED_IF	0
#define DRIVER_MODE_DC_COUPLED_1V2	1
#define DRIVER_MODE_DC_COUPLED_1V8	2
#define DRIVER_MODE_IN_PULL_UP		3

/* Input Driver Mode
 * Use for adi,differential-mode: */
#define DRIVER_MODE_AC_COUPLED		0
#define DRIVER_MODE_DC_COUPLED		1
#define DRIVER_MODE_DC_COUPLED_LVDS	2

/* Output Driver Mode
 * Use for adi,output-mode: */
#define DRIVER_MODE_SINGLE_DIV_DIF	0
#define DRIVER_MODE_SINGLE_DIV		1
#define DRIVER_MODE_DUAL_DIV		2

#endif /* _DT_BINDINGS_CLOCK_AD9545_H_ */
