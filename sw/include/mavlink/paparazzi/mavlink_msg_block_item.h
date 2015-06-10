// MESSAGE BLOCK_ITEM PACKING

#define MAVLINK_MSG_ID_BLOCK_ITEM 180

typedef struct __mavlink_block_item_t
{
 uint16_t seq; ///< Sequence
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t len; ///< Lenght of name
 char name[50]; ///< The name of the mission block
} mavlink_block_item_t;

#define MAVLINK_MSG_ID_BLOCK_ITEM_LEN 55
#define MAVLINK_MSG_ID_180_LEN 55

#define MAVLINK_MSG_ID_BLOCK_ITEM_CRC 13
#define MAVLINK_MSG_ID_180_CRC 13

#define MAVLINK_MSG_BLOCK_ITEM_FIELD_NAME_LEN 50

#define MAVLINK_MESSAGE_INFO_BLOCK_ITEM { \
	"BLOCK_ITEM", \
	5, \
	{  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_block_item_t, seq) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_block_item_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_block_item_t, target_component) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_block_item_t, len) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 50, 5, offsetof(mavlink_block_item_t, name) }, \
         } \
}


/**
 * @brief Pack a block_item message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param len Lenght of name
 * @param name The name of the mission block
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_block_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t len, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BLOCK_ITEM_LEN];
	_mav_put_uint16_t(buf, 0, seq);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);
	_mav_put_uint8_t(buf, 4, len);
	_mav_put_char_array(buf, 5, name, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BLOCK_ITEM_LEN);
#else
	mavlink_block_item_t packet;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.len = len;
	mav_array_memcpy(packet.name, name, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BLOCK_ITEM_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BLOCK_ITEM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BLOCK_ITEM_LEN, MAVLINK_MSG_ID_BLOCK_ITEM_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BLOCK_ITEM_LEN);
#endif
}

/**
 * @brief Pack a block_item message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param len Lenght of name
 * @param name The name of the mission block
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_block_item_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t len,const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BLOCK_ITEM_LEN];
	_mav_put_uint16_t(buf, 0, seq);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);
	_mav_put_uint8_t(buf, 4, len);
	_mav_put_char_array(buf, 5, name, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BLOCK_ITEM_LEN);
#else
	mavlink_block_item_t packet;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.len = len;
	mav_array_memcpy(packet.name, name, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BLOCK_ITEM_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BLOCK_ITEM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BLOCK_ITEM_LEN, MAVLINK_MSG_ID_BLOCK_ITEM_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BLOCK_ITEM_LEN);
#endif
}

/**
 * @brief Encode a block_item struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param block_item C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_block_item_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_block_item_t* block_item)
{
	return mavlink_msg_block_item_pack(system_id, component_id, msg, block_item->target_system, block_item->target_component, block_item->seq, block_item->len, block_item->name);
}

/**
 * @brief Encode a block_item struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param block_item C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_block_item_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_block_item_t* block_item)
{
	return mavlink_msg_block_item_pack_chan(system_id, component_id, chan, msg, block_item->target_system, block_item->target_component, block_item->seq, block_item->len, block_item->name);
}

/**
 * @brief Send a block_item message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param len Lenght of name
 * @param name The name of the mission block
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_block_item_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t len, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BLOCK_ITEM_LEN];
	_mav_put_uint16_t(buf, 0, seq);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);
	_mav_put_uint8_t(buf, 4, len);
	_mav_put_char_array(buf, 5, name, 50);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLOCK_ITEM, buf, MAVLINK_MSG_ID_BLOCK_ITEM_LEN, MAVLINK_MSG_ID_BLOCK_ITEM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLOCK_ITEM, buf, MAVLINK_MSG_ID_BLOCK_ITEM_LEN);
#endif
#else
	mavlink_block_item_t packet;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.len = len;
	mav_array_memcpy(packet.name, name, sizeof(char)*50);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLOCK_ITEM, (const char *)&packet, MAVLINK_MSG_ID_BLOCK_ITEM_LEN, MAVLINK_MSG_ID_BLOCK_ITEM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLOCK_ITEM, (const char *)&packet, MAVLINK_MSG_ID_BLOCK_ITEM_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_BLOCK_ITEM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_block_item_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t len, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, seq);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);
	_mav_put_uint8_t(buf, 4, len);
	_mav_put_char_array(buf, 5, name, 50);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLOCK_ITEM, buf, MAVLINK_MSG_ID_BLOCK_ITEM_LEN, MAVLINK_MSG_ID_BLOCK_ITEM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLOCK_ITEM, buf, MAVLINK_MSG_ID_BLOCK_ITEM_LEN);
#endif
#else
	mavlink_block_item_t *packet = (mavlink_block_item_t *)msgbuf;
	packet->seq = seq;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->len = len;
	mav_array_memcpy(packet->name, name, sizeof(char)*50);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLOCK_ITEM, (const char *)packet, MAVLINK_MSG_ID_BLOCK_ITEM_LEN, MAVLINK_MSG_ID_BLOCK_ITEM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BLOCK_ITEM, (const char *)packet, MAVLINK_MSG_ID_BLOCK_ITEM_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE BLOCK_ITEM UNPACKING


/**
 * @brief Get field target_system from block_item message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_block_item_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field target_component from block_item message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_block_item_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field seq from block_item message
 *
 * @return Sequence
 */
static inline uint16_t mavlink_msg_block_item_get_seq(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field len from block_item message
 *
 * @return Lenght of name
 */
static inline uint8_t mavlink_msg_block_item_get_len(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field name from block_item message
 *
 * @return The name of the mission block
 */
static inline uint16_t mavlink_msg_block_item_get_name(const mavlink_message_t* msg, char *name)
{
	return _MAV_RETURN_char_array(msg, name, 50,  5);
}

/**
 * @brief Decode a block_item message into a struct
 *
 * @param msg The message to decode
 * @param block_item C-struct to decode the message contents into
 */
static inline void mavlink_msg_block_item_decode(const mavlink_message_t* msg, mavlink_block_item_t* block_item)
{
#if MAVLINK_NEED_BYTE_SWAP
	block_item->seq = mavlink_msg_block_item_get_seq(msg);
	block_item->target_system = mavlink_msg_block_item_get_target_system(msg);
	block_item->target_component = mavlink_msg_block_item_get_target_component(msg);
	block_item->len = mavlink_msg_block_item_get_len(msg);
	mavlink_msg_block_item_get_name(msg, block_item->name);
#else
	memcpy(block_item, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_BLOCK_ITEM_LEN);
#endif
}
