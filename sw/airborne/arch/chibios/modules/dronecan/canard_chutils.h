/**
 * @file    canard_chutils.h
 * @brief   Various utilities for libcanard use with ChibiOS.
 * @{
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t id[16];
} UniqId_t;

/**
 * @brief   ChibiOS CANRxFrame to Canard CanardCANFrame Type converter.
 */
CanardCANFrame chibiRx2canard(const CANRxFrame frame);

/**
 * @brief   Canard CanardCANFrame to ChibiOS CANTxFrame Type converter.
 */
CANTxFrame canard2chibiTx(const CanardCANFrame* framep);

/**
 * @brief   Unique ID Fetcher.
 * @note    Retrieves the first two bytes of the MCU's unique id.
 */
void getUniqueID(UniqId_t *uid);

#ifdef __cplusplus
}
#endif

/** @} */