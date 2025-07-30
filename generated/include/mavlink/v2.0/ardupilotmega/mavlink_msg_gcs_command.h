#pragma once
// MESSAGE GCS_COMMAND PACKING

#define MAVLINK_MSG_ID_GCS_COMMAND 411


typedef struct __mavlink_gcs_command_t {
 uint32_t time_boot_ms; /*<  Timestamp (milliseconds since boot)*/
 uint32_t command_type; /*<  Type of command sent from GCS*/
} mavlink_gcs_command_t;

#define MAVLINK_MSG_ID_GCS_COMMAND_LEN 8
#define MAVLINK_MSG_ID_GCS_COMMAND_MIN_LEN 8
#define MAVLINK_MSG_ID_411_LEN 8
#define MAVLINK_MSG_ID_411_MIN_LEN 8

#define MAVLINK_MSG_ID_GCS_COMMAND_CRC 84
#define MAVLINK_MSG_ID_411_CRC 84



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GCS_COMMAND { \
    411, \
    "GCS_COMMAND", \
    2, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gcs_command_t, time_boot_ms) }, \
         { "command_type", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_gcs_command_t, command_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GCS_COMMAND { \
    "GCS_COMMAND", \
    2, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gcs_command_t, time_boot_ms) }, \
         { "command_type", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_gcs_command_t, command_type) }, \
         } \
}
#endif

/**
 * @brief Pack a gcs_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms  Timestamp (milliseconds since boot)
 * @param command_type  Type of command sent from GCS
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint32_t command_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, command_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_COMMAND_LEN);
#else
    mavlink_gcs_command_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.command_type = command_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GCS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_COMMAND_LEN, MAVLINK_MSG_ID_GCS_COMMAND_CRC);
}

/**
 * @brief Pack a gcs_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms  Timestamp (milliseconds since boot)
 * @param command_type  Type of command sent from GCS
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_command_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint32_t command_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, command_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_COMMAND_LEN);
#else
    mavlink_gcs_command_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.command_type = command_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_COMMAND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GCS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_COMMAND_LEN, MAVLINK_MSG_ID_GCS_COMMAND_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GCS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_COMMAND_LEN);
#endif
}

/**
 * @brief Pack a gcs_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms  Timestamp (milliseconds since boot)
 * @param command_type  Type of command sent from GCS
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint32_t command_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, command_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_COMMAND_LEN);
#else
    mavlink_gcs_command_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.command_type = command_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GCS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_COMMAND_LEN, MAVLINK_MSG_ID_GCS_COMMAND_CRC);
}

/**
 * @brief Encode a gcs_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gcs_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gcs_command_t* gcs_command)
{
    return mavlink_msg_gcs_command_pack(system_id, component_id, msg, gcs_command->time_boot_ms, gcs_command->command_type);
}

/**
 * @brief Encode a gcs_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gcs_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gcs_command_t* gcs_command)
{
    return mavlink_msg_gcs_command_pack_chan(system_id, component_id, chan, msg, gcs_command->time_boot_ms, gcs_command->command_type);
}

/**
 * @brief Encode a gcs_command struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gcs_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_command_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gcs_command_t* gcs_command)
{
    return mavlink_msg_gcs_command_pack_status(system_id, component_id, _status, msg,  gcs_command->time_boot_ms, gcs_command->command_type);
}

/**
 * @brief Send a gcs_command message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms  Timestamp (milliseconds since boot)
 * @param command_type  Type of command sent from GCS
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gcs_command_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint32_t command_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, command_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_COMMAND, buf, MAVLINK_MSG_ID_GCS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_COMMAND_LEN, MAVLINK_MSG_ID_GCS_COMMAND_CRC);
#else
    mavlink_gcs_command_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.command_type = command_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_GCS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_COMMAND_LEN, MAVLINK_MSG_ID_GCS_COMMAND_CRC);
#endif
}

/**
 * @brief Send a gcs_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gcs_command_send_struct(mavlink_channel_t chan, const mavlink_gcs_command_t* gcs_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gcs_command_send(chan, gcs_command->time_boot_ms, gcs_command->command_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_COMMAND, (const char *)gcs_command, MAVLINK_MSG_ID_GCS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_COMMAND_LEN, MAVLINK_MSG_ID_GCS_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_GCS_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gcs_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint32_t command_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, command_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_COMMAND, buf, MAVLINK_MSG_ID_GCS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_COMMAND_LEN, MAVLINK_MSG_ID_GCS_COMMAND_CRC);
#else
    mavlink_gcs_command_t *packet = (mavlink_gcs_command_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->command_type = command_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_COMMAND, (const char *)packet, MAVLINK_MSG_ID_GCS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GCS_COMMAND_LEN, MAVLINK_MSG_ID_GCS_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE GCS_COMMAND UNPACKING


/**
 * @brief Get field time_boot_ms from gcs_command message
 *
 * @return  Timestamp (milliseconds since boot)
 */
static inline uint32_t mavlink_msg_gcs_command_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field command_type from gcs_command message
 *
 * @return  Type of command sent from GCS
 */
static inline uint32_t mavlink_msg_gcs_command_get_command_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a gcs_command message into a struct
 *
 * @param msg The message to decode
 * @param gcs_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_gcs_command_decode(const mavlink_message_t* msg, mavlink_gcs_command_t* gcs_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gcs_command->time_boot_ms = mavlink_msg_gcs_command_get_time_boot_ms(msg);
    gcs_command->command_type = mavlink_msg_gcs_command_get_command_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GCS_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_GCS_COMMAND_LEN;
        memset(gcs_command, 0, MAVLINK_MSG_ID_GCS_COMMAND_LEN);
    memcpy(gcs_command, _MAV_PAYLOAD(msg), len);
#endif
}
