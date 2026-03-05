#pragma once
#include <inttypes.h>
#include <canard.h>

struct uavcan_iface_t;

typedef struct {} uavcan_event;


typedef void (*uavcan_callback)(struct uavcan_iface_t *iface, CanardRxTransfer *transfer);

void uavcan_init(void);
void uavcan_bind(uint16_t data_type_id, uint64_t data_type_signature, uavcan_event *ev, uavcan_callback cb);
void uavcan_broadcast(struct uavcan_iface_t *iface, uint64_t data_type_signature, uint16_t data_type_id,
                      uint8_t priority, const void *payload, uint16_t payload_len);
