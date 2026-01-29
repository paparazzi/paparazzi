/*
 * General purpose key-value store.
 * Copyright (C) 2026 Fabien-B <fabien-b@github.com> 
 * This file is part of paparazzi. See LICENCE file.
 */

#pragma once
#include <stdint.h>
#include <stddef.h>

typedef struct {
    size_t capacity;     // number of slots
    size_t esize;        // size of each value
    uint32_t *keys;      // [capacity]
    uint8_t *values;     // [capacity * esize]
    uint8_t *used;       // [capacity] 0 = free, 1 = occupied
} kv_store_t;


void kv_init(kv_store_t *kv, size_t capacity, size_t esize, uint32_t *keys, void *values, uint8_t *used);
int kv_exists(const kv_store_t *kv, uint32_t key);
int kv_set(kv_store_t *kv, uint32_t key, const void *value);
void *kv_get(const kv_store_t *kv, uint32_t key);
int kv_remove(kv_store_t *kv, uint32_t key);