/** @file
 *    @brief MAVLink comm protocol testsuite generated from TS_Addon.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef TS_ADDON_TESTSUITE_H
#define TS_ADDON_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_TS_Addon(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_TS_Addon(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_esc_rads(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ESC_RADS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_esc_rads_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0
    };
    mavlink_esc_rads_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.motor0_fil = packet_in.motor0_fil;
        packet1.motor1_fil = packet_in.motor1_fil;
        packet1.motor2_fil = packet_in.motor2_fil;
        packet1.motor3_fil = packet_in.motor3_fil;
        packet1.motor0_raw = packet_in.motor0_raw;
        packet1.motor1_raw = packet_in.motor1_raw;
        packet1.motor2_raw = packet_in.motor2_raw;
        packet1.motor3_raw = packet_in.motor3_raw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ESC_RADS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ESC_RADS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_rads_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_esc_rads_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_rads_pack(system_id, component_id, &msg , packet1.motor0_fil , packet1.motor1_fil , packet1.motor2_fil , packet1.motor3_fil , packet1.motor0_raw , packet1.motor1_raw , packet1.motor2_raw , packet1.motor3_raw );
    mavlink_msg_esc_rads_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_rads_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor0_fil , packet1.motor1_fil , packet1.motor2_fil , packet1.motor3_fil , packet1.motor0_raw , packet1.motor1_raw , packet1.motor2_raw , packet1.motor3_raw );
    mavlink_msg_esc_rads_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_esc_rads_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_rads_send(MAVLINK_COMM_1 , packet1.motor0_fil , packet1.motor1_fil , packet1.motor2_fil , packet1.motor3_fil , packet1.motor0_raw , packet1.motor1_raw , packet1.motor2_raw , packet1.motor3_raw );
    mavlink_msg_esc_rads_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_TS_Addon(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_esc_rads(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // TS_ADDON_TESTSUITE_H
