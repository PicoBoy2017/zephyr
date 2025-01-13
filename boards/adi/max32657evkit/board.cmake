# Copyright (c) 2024-2025 Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BOARD_MAX32657EVKIT_MAX32657_NS)
  set_property(TARGET runners_yaml_props_target PROPERTY hex_file tfm_merged.hex)
endif()

board_runner_args(openocd --cmd-pre-init "source [find interface/jlink.cfg]")
board_runner_args(openocd --cmd-pre-init "source [find target/max32657.cfg]")
board_runner_args(jlink "--device=MAX32657" "--reset-after-load")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
