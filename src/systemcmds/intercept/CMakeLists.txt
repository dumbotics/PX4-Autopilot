############################################################################
#
#   Copyright (c) 2026.
#
#   SPDX-License-Identifier: BSD-3-Clause
#
############################################################################

px4_add_module(
	MODULE systemcmds__intercept
	MAIN intercept
	SRCS
		intercept.cpp
	DEPENDS
	)

