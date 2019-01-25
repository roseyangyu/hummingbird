#pragma once
// MESSAGE ESC_RADS PACKING

#define MAVLINK_MSG_ID_ESC_RADS 300

MAVPACKED(
typedef struct __mavlink_esc_rads_t {
 float motor0_fil; /*< Motor 0 RADS Filtered*/
 float motor1_fil; /*< Motor 1 RADS Filtered*/
 float motor2_fil; /*< Motor 2 RADS Filtered*/
 float motor3_fil; /*< Motor 3 RADS Filtered*/
 float motor0_raw; /*< Motor 0 RADS Raw*/
 float motor1_raw; /*< Motor 1 RADS Raw*/
 float motor2_raw; /*< Motor 2 RADS Raw*/
 float motor3_raw; /*< Motor 3 RADS Raw*/
}) mavlink_esc_rads_t;

#define MAVLINK_MSG_ID_ESC_RADS_LEN 32
#define MAVLINK_MSG_ID_ESC_RADS_MIN_LEN 32
#define MAVLINK_MSG_ID_300_LEN 32
#define MAVLINK_MSG_ID_300_MIN_LEN 32

#define MAVLINK_MSG_ID_ESC_RADS_CRC 246
#define MAVLINK_MSG_ID_300_CRC 246



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ESC_RADS { \
    300, \
    "ESC_RADS", \
    8, \
    {  { "motor0_fil", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_esc_rads_t, motor0_fil) }, \
         { "motor1_fil", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_esc_rads_t, motor1_fil) }, \
         { "motor2_fil", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_esc_rads_t, motor2_fil) }, \
         { "motor3_fil", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_esc_rads_t, motor3_fil) }, \
         { "motor0_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_esc_rads_t, motor0_raw) }, \
         { "motor1_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_esc_rads_t, motor1_raw) }, \
         { "motor2_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_esc_rads_t, motor2_raw) }, \
         { "motor3_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_esc_rads_t, motor3_raw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ESC_RADS { \
    "ESC_RADS", \
    8, \
    {  { "motor0_fil", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_esc_rads_t, motor0_fil) }, \
         { "motor1_fil", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_esc_rads_t, motor1_fil) }, \
         { "motor2_fil", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_esc_rads_t, motor2_fil) }, \
         { "motor3_fil", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_esc_rads_t, motor3_fil) }, \
         { "motor0_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_esc_rads_t, motor0_raw) }, \
         { "motor1_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_esc_rads_t, motor1_raw) }, \
         { "motor2_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_esc_rads_t, motor2_raw) }, \
         { "motor3_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_esc_rads_t, motor3_raw) }, \
         } \
}
#endif

/**
 * @brief Pack a esc_rads message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor0_fil Motor 0 RADS Filtered
 * @param motor1_fil Motor 1 RADS Filtered
 * @param motor2_fil Motor 2 RADS Filtered
 * @param motor3_fil Motor 3 RADS Filtered
 * @param motor0_raw Motor 0 RADS Raw
 * @param motor1_raw Motor 1 RADS Raw
 * @param motor2_raw Motor 2 RADS Raw
 * @param motor3_raw Motor 3 RADS Raw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esc_rads_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float motor0_fil, float motor1_fil, float motor2_fil, float motor3_fil, float motor0_raw, float motor1_raw, float motor2_raw, float motor3_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_RADS_LEN];
    _mav_put_float(buf, 0, motor0_fil);
    _mav_put_float(buf, 4, motor1_fil);
    _mav_put_float(buf, 8, motor2_fil);
    _mav_put_float(buf, 12, motor3_fil);
    _mav_put_float(buf, 16, motor0_raw);
    _mav_put_float(buf, 20, motor1_raw);
    _mav_put_float(buf, 24, motor2_raw);
    _mav_put_float(buf, 28, motor3_raw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_RADS_LEN);
#else
    mavlink_esc_rads_t packet;
    packet.motor0_fil = motor0_fil;
    packet.motor1_fil = motor1_fil;
    packet.motor2_fil = motor2_fil;
    packet.motor3_fil = motor3_fil;
    packet.motor0_raw = motor0_raw;
    packet.motor1_raw = motor1_raw;
    packet.motor2_raw = motor2_raw;
    packet.motor3_raw = motor3_raw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_RADS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESC_RADS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ESC_RADS_MIN_LEN, MAVLINK_MSG_ID_ESC_RADS_LEN, MAVLINK_MSG_ID_ESC_RADS_CRC);
}

/**
 * @brief Pack a esc_rads message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor0_fil Motor 0 RADS Filtered
 * @param motor1_fil Motor 1 RADS Filtered
 * @param motor2_fil Motor 2 RADS Filtered
 * @param motor3_fil Motor 3 RADS Filtered
 * @param motor0_raw Motor 0 RADS Raw
 * @param motor1_raw Motor 1 RADS Raw
 * @param motor2_raw Motor 2 RADS Raw
 * @param motor3_raw Motor 3 RADS Raw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esc_rads_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float motor0_fil,float motor1_fil,float motor2_fil,float motor3_fil,float motor0_raw,float motor1_raw,float motor2_raw,float motor3_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_RADS_LEN];
    _mav_put_float(buf, 0, motor0_fil);
    _mav_put_float(buf, 4, motor1_fil);
    _mav_put_float(buf, 8, motor2_fil);
    _mav_put_float(buf, 12, motor3_fil);
    _mav_put_float(buf, 16, motor0_raw);
    _mav_put_float(buf, 20, motor1_raw);
    _mav_put_float(buf, 24, motor2_raw);
    _mav_put_float(buf, 28, motor3_raw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_RADS_LEN);
#else
    mavlink_esc_rads_t packet;
    packet.motor0_fil = motor0_fil;
    packet.motor1_fil = motor1_fil;
    packet.motor2_fil = motor2_fil;
    packet.motor3_fil = motor3_fil;
    packet.motor0_raw = motor0_raw;
    packet.motor1_raw = motor1_raw;
    packet.motor2_raw = motor2_raw;
    packet.motor3_raw = motor3_raw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_RADS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESC_RADS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ESC_RADS_MIN_LEN, MAVLINK_MSG_ID_ESC_RADS_LEN, MAVLINK_MSG_ID_ESC_RADS_CRC);
}

/**
 * @brief Encode a esc_rads struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param esc_rads C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_rads_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_esc_rads_t* esc_rads)
{
    return mavlink_msg_esc_rads_pack(system_id, component_id, msg, esc_rads->motor0_fil, esc_rads->motor1_fil, esc_rads->motor2_fil, esc_rads->motor3_fil, esc_rads->motor0_raw, esc_rads->motor1_raw, esc_rads->motor2_raw, esc_rads->motor3_raw);
}

/**
 * @brief Encode a esc_rads struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param esc_rads C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_rads_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_esc_rads_t* esc_rads)
{
    return mavlink_msg_esc_rads_pack_chan(system_id, component_id, chan, msg, esc_rads->motor0_fil, esc_rads->motor1_fil, esc_rads->motor2_fil, esc_rads->motor3_fil, esc_rads->motor0_raw, esc_rads->motor1_raw, esc_rads->motor2_raw, esc_rads->motor3_raw);
}

/**
 * @brief Send a esc_rads message
 * @param chan MAVLink channel to send the message
 *
 * @param motor0_fil Motor 0 RADS Filtered
 * @param motor1_fil Motor 1 RADS Filtered
 * @param motor2_fil Motor 2 RADS Filtered
 * @param motor3_fil Motor 3 RADS Filtered
 * @param motor0_raw Motor 0 RADS Raw
 * @param motor1_raw Motor 1 RADS Raw
 * @param motor2_raw Motor 2 RADS Raw
 * @param motor3_raw Motor 3 RADS Raw
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_esc_rads_send(mavlink_channel_t chan, float motor0_fil, float motor1_fil, float motor2_fil, float motor3_fil, float motor0_raw, float motor1_raw, float motor2_raw, float motor3_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_RADS_LEN];
    _mav_put_float(buf, 0, motor0_fil);
    _mav_put_float(buf, 4, motor1_fil);
    _mav_put_float(buf, 8, motor2_fil);
    _mav_put_float(buf, 12, motor3_fil);
    _mav_put_float(buf, 16, motor0_raw);
    _mav_put_float(buf, 20, motor1_raw);
    _mav_put_float(buf, 24, motor2_raw);
    _mav_put_float(buf, 28, motor3_raw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_RADS, buf, MAVLINK_MSG_ID_ESC_RADS_MIN_LEN, MAVLINK_MSG_ID_ESC_RADS_LEN, MAVLINK_MSG_ID_ESC_RADS_CRC);
#else
    mavlink_esc_rads_t packet;
    packet.motor0_fil = motor0_fil;
    packet.motor1_fil = motor1_fil;
    packet.motor2_fil = motor2_fil;
    packet.motor3_fil = motor3_fil;
    packet.motor0_raw = motor0_raw;
    packet.motor1_raw = motor1_raw;
    packet.motor2_raw = motor2_raw;
    packet.motor3_raw = motor3_raw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_RADS, (const char *)&packet, MAVLINK_MSG_ID_ESC_RADS_MIN_LEN, MAVLINK_MSG_ID_ESC_RADS_LEN, MAVLINK_MSG_ID_ESC_RADS_CRC);
#endif
}

/**
 * @brief Send a esc_rads message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_esc_rads_send_struct(mavlink_channel_t chan, const mavlink_esc_rads_t* esc_rads)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_esc_rads_send(chan, esc_rads->motor0_fil, esc_rads->motor1_fil, esc_rads->motor2_fil, esc_rads->motor3_fil, esc_rads->motor0_raw, esc_rads->motor1_raw, esc_rads->motor2_raw, esc_rads->motor3_raw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_RADS, (const char *)esc_rads, MAVLINK_MSG_ID_ESC_RADS_MIN_LEN, MAVLINK_MSG_ID_ESC_RADS_LEN, MAVLINK_MSG_ID_ESC_RADS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ESC_RADS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_esc_rads_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float motor0_fil, float motor1_fil, float motor2_fil, float motor3_fil, float motor0_raw, float motor1_raw, float motor2_raw, float motor3_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, motor0_fil);
    _mav_put_float(buf, 4, motor1_fil);
    _mav_put_float(buf, 8, motor2_fil);
    _mav_put_float(buf, 12, motor3_fil);
    _mav_put_float(buf, 16, motor0_raw);
    _mav_put_float(buf, 20, motor1_raw);
    _mav_put_float(buf, 24, motor2_raw);
    _mav_put_float(buf, 28, motor3_raw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_RADS, buf, MAVLINK_MSG_ID_ESC_RADS_MIN_LEN, MAVLINK_MSG_ID_ESC_RADS_LEN, MAVLINK_MSG_ID_ESC_RADS_CRC);
#else
    mavlink_esc_rads_t *packet = (mavlink_esc_rads_t *)msgbuf;
    packet->motor0_fil = motor0_fil;
    packet->motor1_fil = motor1_fil;
    packet->motor2_fil = motor2_fil;
    packet->motor3_fil = motor3_fil;
    packet->motor0_raw = motor0_raw;
    packet->motor1_raw = motor1_raw;
    packet->motor2_raw = motor2_raw;
    packet->motor3_raw = motor3_raw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_RADS, (const char *)packet, MAVLINK_MSG_ID_ESC_RADS_MIN_LEN, MAVLINK_MSG_ID_ESC_RADS_LEN, MAVLINK_MSG_ID_ESC_RADS_CRC);
#endif
}
#endif

#endif

// MESSAGE ESC_RADS UNPACKING


/**
 * @brief Get field motor0_fil from esc_rads message
 *
 * @return Motor 0 RADS Filtered
 */
static inline float mavlink_msg_esc_rads_get_motor0_fil(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field motor1_fil from esc_rads message
 *
 * @return Motor 1 RADS Filtered
 */
static inline float mavlink_msg_esc_rads_get_motor1_fil(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field motor2_fil from esc_rads message
 *
 * @return Motor 2 RADS Filtered
 */
static inline float mavlink_msg_esc_rads_get_motor2_fil(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field motor3_fil from esc_rads message
 *
 * @return Motor 3 RADS Filtered
 */
static inline float mavlink_msg_esc_rads_get_motor3_fil(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field motor0_raw from esc_rads message
 *
 * @return Motor 0 RADS Raw
 */
static inline float mavlink_msg_esc_rads_get_motor0_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field motor1_raw from esc_rads message
 *
 * @return Motor 1 RADS Raw
 */
static inline float mavlink_msg_esc_rads_get_motor1_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field motor2_raw from esc_rads message
 *
 * @return Motor 2 RADS Raw
 */
static inline float mavlink_msg_esc_rads_get_motor2_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field motor3_raw from esc_rads message
 *
 * @return Motor 3 RADS Raw
 */
static inline float mavlink_msg_esc_rads_get_motor3_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a esc_rads message into a struct
 *
 * @param msg The message to decode
 * @param esc_rads C-struct to decode the message contents into
 */
static inline void mavlink_msg_esc_rads_decode(const mavlink_message_t* msg, mavlink_esc_rads_t* esc_rads)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    esc_rads->motor0_fil = mavlink_msg_esc_rads_get_motor0_fil(msg);
    esc_rads->motor1_fil = mavlink_msg_esc_rads_get_motor1_fil(msg);
    esc_rads->motor2_fil = mavlink_msg_esc_rads_get_motor2_fil(msg);
    esc_rads->motor3_fil = mavlink_msg_esc_rads_get_motor3_fil(msg);
    esc_rads->motor0_raw = mavlink_msg_esc_rads_get_motor0_raw(msg);
    esc_rads->motor1_raw = mavlink_msg_esc_rads_get_motor1_raw(msg);
    esc_rads->motor2_raw = mavlink_msg_esc_rads_get_motor2_raw(msg);
    esc_rads->motor3_raw = mavlink_msg_esc_rads_get_motor3_raw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ESC_RADS_LEN? msg->len : MAVLINK_MSG_ID_ESC_RADS_LEN;
        memset(esc_rads, 0, MAVLINK_MSG_ID_ESC_RADS_LEN);
    memcpy(esc_rads, _MAV_PAYLOAD(msg), len);
#endif
}
