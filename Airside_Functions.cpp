/**************************************************************************************************/
// Author: Jingting Liu
// April 9th, 2021

// to run the test without cmake:
// gcc -g Airside_Functions.cpp
// a.out

// to run the test with cmake:
// mkdir build, cd build
// cmake ..
// make
// execute generated target


// refer to this page for the stucture of mavlink messages 
// https://mavlink.io/en/guide/serialization.html

// The decoder and encoder only support GPS and gimbal control, other simpler commands will be taken
// care of by the Xbee communication directly
/**************************************************************************************************/
#include <stdlib.h>
#include "Mavlink2_lib/common/mavlink.h"
#include "Airside_Functions.hpp"

PIGO_Message_IDs_t decoded_message_type = MESSAGE_ID_NONE;


mavlink_decoding_status_t Mavlink_airside_decoder(int channel, uint8_t incomingByte, uint8_t *telemetryData)
{
    decoded_message_type = MESSAGE_ID_NONE; //init
    mavlink_decoding_status_t decoding_status = MAVLINK_DECODING_INCOMPLETE;

    mavlink_status_t status;
    memset(&status, 0x00, sizeof(mavlink_status_t));

    mavlink_message_t decoded_msg;
    memset(&decoded_msg, 0, sizeof(mavlink_message_t));

    uint8_t message_received = mavlink_parse_char(channel, incomingByte, &decoded_msg, &status);

    if (message_received)
    {
        if (telemetryData == NULL)
        {
            return MAVLINK_DECODING_FAIL;
        }

        switch(decoded_msg.msgid)
        {
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // ID for GLOBAL_POSITION_INT, 33
                {
                    mavlink_global_position_int_t global_position;
                    memset(&global_position, 0x00, sizeof(mavlink_global_position_int_t));
                    mavlink_msg_global_position_int_decode(&decoded_msg, &global_position);
                    uint32_t warg_ID = global_position.time_boot_ms;

                        if (warg_ID == MESSAGE_ID_GPS_LANDING_SPOT)
                        {
                            PIGO_GPS_LANDING_SPOT_t landing_spot;
                            memset(&landing_spot, 0x00, sizeof(PIGO_GPS_LANDING_SPOT_t));
                            landing_spot.latitude = global_position.lat;
                            landing_spot.longitude = global_position.lon;
                            landing_spot.altitude = global_position.alt;
                            landing_spot.landingDirection = global_position.relative_alt;

                            memcpy((void*) telemetryData, (void*) &landing_spot, sizeof(PIGO_GPS_LANDING_SPOT_t));

                            decoding_status = MAVLINK_DECODING_OKAY;
                        }

                        else if (   warg_ID == MESSAGE_ID_WAYPOINTS ||
                                    warg_ID == MESSAGE_ID_HOMEBASE)
                        {
                            PIGO_WAYPOINTS_t waypoints;
                            memset(&waypoints, 0x00, sizeof(PIGO_WAYPOINTS_t));
                            waypoints.latitude = global_position.lat;
                            waypoints.longitude = global_position.lon;
                            waypoints.altitude = global_position.alt;
                            waypoints.turnRadius = global_position.relative_alt;
                            waypoints.waypointType = global_position.hdg;

                            memcpy((void*) telemetryData, (void*) &waypoints, sizeof(PIGO_WAYPOINTS_t));

                            decoding_status = MAVLINK_DECODING_OKAY;
                        }

                        else if (   warg_ID == MESSAGE_ID_NUM_WAYPOINTS ||
                                    warg_ID == MESSAGE_ID_HOLDING_ALTITUDE ||
                                    warg_ID == MESSAGE_ID_HOLDING_TURN_RADIUS ||
                                    warg_ID == MESSAGE_ID_PATH_MODIFY_NEXT_LD ||
                                    warg_ID == MESSAGE_ID_PATH_MODIFY_PREV_LD ||
                                    warg_ID == MESSAGE_ID_PATH_MODIFY_LD)
                        {
                            four_bytes_int_cmd_t command;
                            memset(&command, 0x00, sizeof(four_bytes_int_cmd_t));
                            command.cmd = global_position.lat;

                            memcpy((void*) telemetryData, (void*) &command, sizeof(four_bytes_int_cmd_t));

                            decoding_status = MAVLINK_DECODING_OKAY;
                        }

                        else if (warg_ID == MESSAGE_ID_GIMBAL_CMD)
                        {                 
                            PIGO_GIMBAL_t command;
                            memset(&command, 0x00, sizeof(PIGO_GIMBAL_t));
                            command.pitch = global_position.lat;
                            command.yaw = global_position.lon;

                            memcpy((void*) telemetryData, (void*) &command, sizeof(PIGO_GIMBAL_t));

                            decoding_status = MAVLINK_DECODING_OKAY;                              
                        }

                        else if (warg_ID == MESSAGE_ID_GROUND_CMD)
                        {                 
                            PIGO_GROUND_COMMAND_t command;
                            memset(&command, 0x00, sizeof(PIGO_GROUND_COMMAND_t));
                            command.heading = global_position.lat;
                            command.latestDistance = global_position.lon;

                            memcpy((void*) telemetryData, (void*) &command, sizeof(PIGO_GROUND_COMMAND_t));

                            decoding_status = MAVLINK_DECODING_OKAY;                               
                        }

                        else if (   warg_ID == MESSAGE_ID_WAYPOINT_MODIFY_PATH_CMD ||
                                    warg_ID == MESSAGE_ID_WAYPOINT_NEXT_DIRECTIONS_CMD ||
                                    warg_ID == MESSAGE_ID_HOLDING_TURN_DIRECTION)
                        {
                            one_byte_uint_cmd_t command;
                            memset(&command, 0x00, sizeof(one_byte_uint_cmd_t));
                            command.cmd = global_position.hdg;

                            memcpy((void*) telemetryData, (void*) &command, sizeof(one_byte_uint_cmd_t));
                            decoding_status = MAVLINK_DECODING_OKAY;
                        }

                        else if (   warg_ID == MESSAGE_ID_BEGIN_LANDING ||
                                    warg_ID == MESSAGE_ID_BEGIN_TAKEOFF ||
                                    warg_ID ==MESSAGE_ID_INITIALIZING_HOMEBASE)
                        {
                            single_bool_cmd_t isLanded;
                            memset(&isLanded, 0x00, sizeof(single_bool_cmd_t));
                            isLanded.cmd = global_position.hdg;

                            memcpy((void*) telemetryData, (void*) &isLanded, sizeof(single_bool_cmd_t));
                            decoding_status = MAVLINK_DECODING_OKAY;
                        }

                        if (decoding_status == MAVLINK_DECODING_OKAY)
                        {
                            decoded_message_type = (PIGO_Message_IDs_t) warg_ID;
                            return MAVLINK_DECODING_OKAY;
                        }

                        return MAVLINK_DECODING_FAIL;
                }
            default:
                break;
        }// end of switch
    }// if message receieved

    return MAVLINK_DECODING_INCOMPLETE;
}


PIGO_Message_IDs_t Mavlink_airside_decoder_get_message_type(void)
{
    return decoded_message_type;
}


mavlink_encoding_status_t Mavlink_airside_encoder(POGI_Message_IDs_t msgID, mavlink_message_t *message, const uint8_t *struct_ptr) 
{
    mavlink_encoding_status_t encoding_status = MAVLINK_ENCODING_INCOMPLETE;
    uint8_t system_id = 1;
    uint8_t component_id = 1;
    mavlink_message_t encoded_msg_original;
    memset(&encoded_msg_original, 0, sizeof(mavlink_message_t));
    uint16_t message_len;
    mavlink_global_position_int_t global_position;

    switch(msgID)
    {
        case MESSAGE_ID_TIMESTAMP:
        {
            POGI_Timestamp_t* timestamp_cmd = (POGI_Timestamp_t*) struct_ptr;

            //mavlink_global_position_int_t global_position;
            //global_position.time_boot_ms = MESSAGE_ID_TIMESTAMP;
            global_position.lat = timestamp_cmd ->timeStamp;

            //message_len = mavlink_msg_global_position_int_encode(system_id, component_id, &encoded_msg_original, &global_position);
        } 
        break;

        case MESSAGE_ID_GPS:
        {
            POGI_GPS_t* warg_GPS_cmd = (POGI_GPS_t*) struct_ptr;

            //mavlink_global_position_int_t global_position;
            //global_position.time_boot_ms = MESSAGE_ID_GPS; //TODO set this to the correct ID
            global_position.lat = warg_GPS_cmd ->latitude;
            global_position.lon = warg_GPS_cmd ->longitude;
            global_position.alt = warg_GPS_cmd ->altitude;

            //message_len = mavlink_msg_global_position_int_encode(system_id, component_id, &encoded_msg_original, &global_position);
        } 
        break;

        case MESSAGE_ID_EULER_ANGLE_PLANE:
        case MESSAGE_ID_EULER_ANGLE_CAM:
        {
            POGI_Euler_Angle_t* gimbal_cmd = (POGI_Euler_Angle_t*) struct_ptr;

            //mavlink_global_position_int_t global_position;
            //global_position.time_boot_ms = MESSAGE_ID_GIMBAL_CMD; //TODO set this to the correct ID
            global_position.lat = gimbal_cmd ->roll;
            global_position.lon = gimbal_cmd ->pitch;
            global_position.alt = gimbal_cmd ->yaw;

            //message_len = mavlink_msg_global_position_int_encode(system_id, component_id, &encoded_msg_original, &global_position);
        }
        break;

        case MESSAGE_ID_IS_LANDED:
        case MESSAGE_ID_HOMEBASE_INITIALIZED:
        {
            single_bool_cmd_t* warg_cmd = (single_bool_cmd_t*) struct_ptr;

            //mavlink_global_position_int_t global_position;
            //global_position.time_boot_ms = MESSAGE_ID_BEGIN_LANDING; //TODO set this to the correct ID
            global_position.hdg = warg_cmd ->cmd;

            //message_len = mavlink_msg_global_position_int_encode(system_id, component_id, &encoded_msg_original, &global_position);
        }
        break;

        case MESSAGE_ID_AIR_SPEED:
        case MESSAGE_ID_CURRENT_WAYPOINT_LD:
        case MESSAGE_ID_CURRENT_WAYPOINT_INDEX:
        {
            four_bytes_int_cmd_t* numWaypoint_cmd = (four_bytes_int_cmd_t*) struct_ptr;

            //mavlink_global_position_int_t global_position;
            //global_position.time_boot_ms = 1; //TODO  use appropriate ID here
            global_position.lat = numWaypoint_cmd ->cmd;

            //message_len = mavlink_msg_global_position_int_encode(system_id, component_id, &encoded_msg_original, &global_position);
        } 
        break;

        case MESSAGE_ID_ERROR_CODE:
        case MESSAGE_ID_EDITING_FLIGHT_PATH_ERROR_CODE:
        case MESSAGE_ID_FLIGHT_PATH_FOLLOWING_ERROR_CODE:
        {
            one_byte_uint_cmd_t* warg_cmd = (one_byte_uint_cmd_t*) struct_ptr;

            //mavlink_global_position_int_t global_position;
            //global_position.time_boot_ms = MESSAGE_ID_HOLDING_TURN_DIRECTION; //TODO  use appropriate ID here
            global_position.hdg = warg_cmd ->cmd;

            //message_len = mavlink_msg_global_position_int_encode(system_id, component_id, &encoded_msg_original, &global_position);
        }
        break;

        default:
            encoding_status = MAVLINK_ENCODING_FAIL;
            break;
    }
    global_position.time_boot_ms = MESSAGE_ID_HOLDING_TURN_DIRECTION;
    message_len = mavlink_msg_global_position_int_encode(system_id, component_id, &encoded_msg_original, &global_position);

    if (message_len == 0)
    {
        return MAVLINK_ENCODING_FAIL;
    }

    // the following loop is supposed to fix an inconsistency the original mavlink encoder, where the first byte is always shifted.
    // this code lets the encoder to send out a message (byte array) that starts exactly with the starting byte
    unsigned char* ptr_in_byte = (unsigned char *) &encoded_msg_original;
    char message_buffer[message_len];

    uint8_t start_index = 0;
    for( int i = 0; i < message_len; i++)
    {

        if (ptr_in_byte[i] == 0xfd) //0xfd, starting byte
        {
            start_index = i;
            for(int r = 0; r < message_len-2; r++)
            {
                message_buffer[r] = ptr_in_byte[r+i];
                //printf("copying byte: %d / %d   |   current byte : %hhx\n", r, message_len, message_buffer[r]);
            }
            break;
        }

        else if (i == message_len-1)
        {
            return MAVLINK_ENCODING_FAIL;
        }
    }

    if (start_index > 1)
    {
        for (int i = 0; i<2;i++)
        {
            message_buffer[message_len-2+i] = ptr_in_byte[start_index-2+i]; // load the last 2 checksum bytes
            //printf("copying byte: %d / %d   |   current byte : %hhx\n", message_len-2+i, message_len, message_buffer[message_len-2+i]);
        }
        //message = (mavlink_message_t*) malloc(message_len);
        memcpy(message, message_buffer, message_len);

        return MAVLINK_ENCODING_OKAY;
    }
    else
    {
        return MAVLINK_ENCODING_FAIL;
    }
}

//---------------------------------------- tests -----------------------------------------------------------------------------
//an example of how to use the encoder and decoder

int test__encode_then_decode(void)
{
    //printf("put printf here affect message, random bytes inserted\n");

   PIGO_GPS_LANDING_SPOT_t landing_spot = {
       1,
       2,
       3,
       4,
   };
   POGI_GPS_t GPS_Test = {
       1,2,3,
   };
    PIGO_WAYPOINTS_t warg_GPS = 
    {
        2, //int32_t latitude;
        3, //int32_t longitude;
        4, //int32_t altitude;
        5, //int32_t turnpoint;
        9,
    };

    PIGO_GIMBAL_t gimbal_cmd = 
    {
        2, //pitch
        1, //yaw
    };
    POGI_Euler_Angle_t angle_cmd = {
        1, //yaw
        2, //pitch
        3, //roll
    };
    uint16_t inte = 8;
    one_byte_uint_cmd_t uint8_cmd = 
    {
        8,
    };

    single_bool_cmd_t bool_cmd = 
    {
        1,
    };


    mavlink_message_t encoded_msg;
    memset(&encoded_msg, 0, sizeof(mavlink_message_t));
    //printf("space holder here\n");
    uint8_t encoderStatus = Mavlink_airside_encoder(MESSAGE_ID_ERROR_CODE, &encoded_msg, (const uint8_t*) &uint8_cmd);
    //uint8_t encoderStatus = Mavlink_airside_encoder(MESSAGE_ID_GPS, &encoded_msg, (const uint8_t*) &landing_spot);
    //uint8_t encoderStatus = Mavlink_airside_encoder(MESSAGE_ID_EULER_ANGLE_CAM, &encoded_msg, (const uint8_t*) &angle_cmd);

    if (encoderStatus == MAVLINK_ENCODING_FAIL)
    {
        //printf("encoding failed");
        return 0;
    }
    //printf("put printf here doesn't affect message\n");
    //---------------------------------- decoding starts ---------------------------------- 

    mavlink_decoding_status_t decoderStatus = MAVLINK_DECODING_INCOMPLETE;
    //mavlink_attitude_t gimbal_command_decoded;

    char decoded_message_buffer[50]; //256 is the max payload length

    // the following few lines imitates how a decoder is used when it gets one byte at a time from a serial port
    unsigned char* ptr_in_byte = (unsigned char *) &encoded_msg;
    
    for( int i = 0; i < 50; i++) // 50 is just a random number larger than message length (for GPS message length is 39)
    {
        if (decoderStatus != MAVLINK_DECODING_OKAY)
        {
            //printf("copying byte: %d  |  current byte : %hhx\n", i, ptr_in_byte[i]);
            decoderStatus = Mavlink_airside_decoder(MAVLINK_COMM_0, ptr_in_byte[i], (uint8_t*) &decoded_message_buffer);
            //printf("copying byte: %d  |  current byte : %hhx\n", i, ptr_in_byte[i]);
            //printf("%d\n", decoderStatus);
        }
    }

    if (decoderStatus == MAVLINK_DECODING_OKAY)
    {
        //printf("initial decoding okay\n");
        PIGO_Message_IDs_t message_type = Mavlink_airside_decoder_get_message_type();
        int result = 1;

        switch (message_type) // those types need to match ground side decoder's type, currently use the airside only for the purpose of testing
        {
            case MESSAGE_ID_GPS_LANDING_SPOT:
                {
                    PIGO_GPS_LANDING_SPOT_t landing_spot_decoded;
                    memcpy(&landing_spot_decoded, &decoded_message_buffer, sizeof(PIGO_GPS_LANDING_SPOT_t));
                    
                    result = memcmp(&landing_spot_decoded, &landing_spot, sizeof(PIGO_GPS_LANDING_SPOT_t) );

                }
                break;

            case MESSAGE_ID_GIMBAL_CMD:
                {
                    PIGO_GIMBAL_t cmd_received;
                    memcpy(&cmd_received, &decoded_message_buffer, sizeof(PIGO_GIMBAL_t));

                    result = memcmp(&cmd_received, &gimbal_cmd, sizeof(PIGO_GIMBAL_t) );
                }
                break;

            case MESSAGE_ID_BEGIN_LANDING: //test for all one byte cmd
                {
                    single_bool_cmd_t islanded;
                    memcpy(&islanded, &decoded_message_buffer, sizeof(single_bool_cmd_t));
                    result = memcmp(&islanded, &bool_cmd, sizeof(single_bool_cmd_t) );
                }
                break;

            case MESSAGE_ID_WAYPOINT_MODIFY_PATH_CMD: //test for all one byte cmd
                {
                    one_byte_uint_cmd_t cmd_decoded;
                    memcpy(&cmd_decoded, &decoded_message_buffer, sizeof(one_byte_uint_cmd_t));

                    result = memcmp(&cmd_decoded, &uint8_cmd, sizeof(one_byte_uint_cmd_t) );
                }
                break;

            case MESSAGE_ID_HOLDING_TURN_DIRECTION: //test for all one byte uint8_t cmd
                {

                    one_byte_uint_cmd_t cmd_decoded;

                    memcpy(&cmd_decoded, &decoded_message_buffer, sizeof(one_byte_uint_cmd_t));

                    result = memcmp(&cmd_decoded, &uint8_cmd, sizeof(one_byte_uint_cmd_t));
                    /*
                    void* check = malloc(sizeof(one_byte_uint_cmd_t));
                    memcpy(check, &decoded_message_buffer, sizeof(one_byte_uint_cmd_t));
                    result = memcmp(check, &uint8_cmd, sizeof(one_byte_uint_cmd_t));
                    free(check);
                    */
                }
                break;

                
            default:
                break;
        }

        if (result == 0)
        {
            printf("test passed!\n");
            return 1;
        }
    }
    else{
        printf("decoding failed\n");
    }
    return 0;
}


int main(void) // TODO: this main needs to be removed once integrated
{
    test__encode_then_decode();

    return 0;
}


