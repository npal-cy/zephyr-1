# Copyright (c) 2024 Cypress Semiconductor Corporation.
# SPDX-License-Identifier: Apache-2.0

board_runner_args(openocd "--target-handle=TARGET.cm33")
board_runner_args(openocd "--openocd-search=${ZEPHYR_HAL_INFINEON_MODULE_DIR}/zephyr/blobs/flashloader/TARGET_CYW920829M2EVK-02")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
board_runner_args(jlink "--device=CYW20829_tm")
include (${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
