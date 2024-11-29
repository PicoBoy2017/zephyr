# Copyright (c) 2024 Analog Devices, Inc.
#
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_include_directories(.)
zephyr_library_sources(
  cgms.c
  cgms_db.c
  cgms_racp.c
  cgms_socp.c
)
