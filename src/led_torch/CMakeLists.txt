# Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>
#
# SPDX-License-Identifier: Apache-2.0

include_directories(. ${DROIDMEDIA_INCLUDE_DIRS})

webos_build_nyx_module(LedTorch
                       SOURCES led_torch.c
                       LIBRARIES ${GLIB2_LDFLAGS} ${PMLOG_LDFLAGS} ${NYXLIB_LDFLAGS} ${DROIDMEDIA_LDFLAGS} -lrt -lpthread)
