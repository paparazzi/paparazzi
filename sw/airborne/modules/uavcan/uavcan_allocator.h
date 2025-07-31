#pragma once
#include "inttypes.h"
/**
 *  Dynamic node ID allocation.
 *  See https://dronecan.github.io/Specification/6._Application_level_functions/#dynamic-node-id-allocation
 */

struct uavcan_iface_t;

struct uavcan_unique_id_t {
  uint8_t len;
  uint8_t data[16];
};

struct uavcan_node_mapping_t {
  struct uavcan_unique_id_t unique_id;
  uint8_t allocated_id;
};

struct uavcan_node_mapping_t* uavcan_get_node_id_mapping(const uint8_t id);
void request_node_info(struct uavcan_iface_t *iface);

void uavcan_allocator_init(void);
void uavcan_allocator_periodic(void);
