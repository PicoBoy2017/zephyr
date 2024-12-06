# Copyright (c) 2024 Analog Devices, Inc.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(adi_cgm)

target_sources(app PRIVATE
  src/main.c
  src/database.c
  src/services/accs.c
  src/services/bas_monitor.c
  src/services/ecs_monitor.c
  src/services/adi_shell.c
  src/led_ind/led_ind.c
)
