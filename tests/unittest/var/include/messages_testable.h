#include "subsystems/datalink/transport.h"
#include "mcu_periph/link_device.h"
#undef DOWNLINK_SEND_LOG_DATAPACKET

#define DOWNLINK_SEND_LOG_DATAPACKET(_trans, _dev, timestamp, data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10, data_11, data_12) testable_pprz_msg_send_LOG_DATAPACKET(&((_trans).trans_tx), &((_dev).device), AC_ID, timestamp, data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10, data_11, data_12)
void testable_pprz_msg_send_LOG_DATAPACKET(struct transport_tx *trans, struct link_device *dev, uint8_t ac_id, uint32_t *_timestamp, int32_t *_data_1, int32_t *_data_2, int32_t *_data_3, int32_t *_data_4, int32_t *_data_5, int32_t *_data_6, int32_t *_data_7, int32_t *_data_8, int32_t *_data_9, int32_t *_data_10, int32_t *_data_11, int32_t *_data_12);
