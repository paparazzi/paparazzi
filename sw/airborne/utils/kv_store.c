/*
 * General purpose key-value store.
 * Copyright (C) 2026 Fabien-B <fabien-b@github.com> 
 * This file is part of paparazzi. See LICENCE file.
 */

#include "kv_store.h"
#include "string.h"

/**
 * @brief Initializes a key-value store.
 * 
 * @param kv Pointer to the kv_store_t structure to initialize.
 * @param capacity The maximum number of key-value pairs the store can hold.
 * @param esize The size in bytes of each value element.
 * @param keys Pointer to an array of uint32_t for storing keys. At least "capacity" long.
 * @param values Pointer to the memory area for storing values. At least "capacity * esize" long.
 * @param used Pointer to an array of uint8_t flags indicating if a slot is used. At least "capacity" long.
 */
void kv_init(kv_store_t *kv, size_t capacity, size_t esize,
            uint32_t *keys, void *values, uint8_t *used) {

  kv->capacity = capacity;
  kv->esize = esize;
  kv->keys = keys;
  kv->values = (uint8_t *)values;
  kv->used = used;

  for (size_t i = 0; i < capacity; i++) {
    kv->used[i] = 0;
  }
}

/**
 * @brief Finds the index of a given key in the store.
 *
 * @param kv Pointer to the kv_store_t structure.
 * @param key The key to search for.
 * @param index Pointer to a size_t where the index will be stored if found (can be NULL).
 * @return 1 if the key is found, 0 otherwise.
 */
static int kv_find(const kv_store_t *kv, uint32_t key, size_t *index) {
  for (size_t i = 0; i < kv->capacity; i++)
  {
    if (kv->used[i] && kv->keys[i] == key)
    {
      if (index)
      {
        *index = i;
      }
      return 1; // found
    }
  }
  return 0; // not found
}

/**
 * @brief Checks if a key exists in the store.
 *
 * @param kv Pointer to the kv_store_t structure.
 * @param key The key to check for existence.
 * @return 1 if the key exists, 0 otherwise.
 */
int kv_exists(const kv_store_t *kv, uint32_t key) {
  return kv_find(kv, key, NULL);
}

/**
 * @brief Sets a value for a given key.
 *
 * If the key already exists, its value is updated. If not, a new key-value pair is inserted
 * into the first available slot. If the store is full, the operation fails.
 *
 * @param kv Pointer to the kv_store_t structure.
 * @param key The key to set.
 * @param value Pointer to the value to store. Value will be memcpy in the store, so the pointer does not need to be valid afterward.
 * @return 0 on success, -1 if the store is full.
 */
int kv_set(kv_store_t *kv, uint32_t key, const void *value) {
  size_t index;

  // Update existing
  if (kv_find(kv, key, &index))
  {
    memcpy(&kv->values[index * kv->esize], value, kv->esize);
    return 0;
  }

  // Insert new
  for (size_t i = 0; i < kv->capacity; i++)
  {
    if (!kv->used[i])
    {
      kv->used[i] = 1;
      kv->keys[i] = key;
      memcpy(&kv->values[i * kv->esize], value, kv->esize);
      return 0;
    }
  }
  
  return -1;  // store full
}

/**
 * @brief Retrieves the value associated with a given key.
 *
 * @param kv Pointer to the kv_store_t structure.
 * @param key The key to retrieve the value for.
 * @return Pointer to the value if the key exists, NULL otherwise.
 */
void* kv_get(const kv_store_t *kv, uint32_t key) {
  size_t index;

  if (!kv_find(kv, key, &index))
    return NULL;

  return &kv->values[index * kv->esize];
}

/**
 * @brief Removes a key-value pair from the store.
 *
 * @param kv Pointer to the kv_store_t structure.
 * @param key The key to remove.
 * @return 0 on success, -1 if the key was not found.
 */
int kv_remove(kv_store_t *kv, uint32_t key) {
  size_t index;

  if (!kv_find(kv, key, &index))
    return -1;

  kv->used[index] = 0;
  return 0;
}
