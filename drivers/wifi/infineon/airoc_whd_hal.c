/*
 * Copyright (c) 2023 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <airoc_wifi.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                 Function
 ******************************************************/

/*
 * Implement WHD memory wrappers
 */

void *whd_mem_malloc(size_t size)
{
	return k_malloc(size);
}

void *whd_mem_calloc(size_t nitems, size_t size)
{
	return k_calloc(nitems, size);
}

void whd_mem_free(void *ptr)
{
	k_free(ptr);
}

#ifdef __cplusplus
} /* extern "C" */
#endif
