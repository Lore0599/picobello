// Copyright 2025 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Tim Fischer <fischeti@iis.ee.ethz.ch>

#pragma once

#define SNRT_INIT_BSS
#define SNRT_WAKE_UP
#define SNRT_INIT_TLS
#define SNRT_INIT_CLS
#define SNRT_INIT_LIBS
#define SNRT_CRT0_PRE_BARRIER
#define SNRT_INVOKE_MAIN
#define SNRT_CRT0_POST_BARRIER
#define SNRT_CRT0_EXIT
#define SNRT_CRT0_ALTERNATE_EXIT


static inline volatile uint32_t* snrt_exit_code_destination() {
    return (volatile uint32_t*)snrt_cluster()->peripheral_reg.scratch[0].f.scratch;
}

inline void snrt_exit(int exit_code) {
    *(snrt_exit_code_destination() + snrt_cluster_core_idx()) = (exit_code << 1) | 1;
}

#include "start.h"
