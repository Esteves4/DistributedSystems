/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.9.3 at Tue Oct 01 20:33:43 2019. */

#ifndef PB_SENSOR_PB_H_INCLUDED
#define PB_SENSOR_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _Sensor_SensorType {
    Sensor_SensorType_TEMPERATURE = 0,
    Sensor_SensorType_LIGHT = 1,
    Sensor_SensorType_LUMINOSITY = 2
} Sensor_SensorType;
#define _Sensor_SensorType_MIN Sensor_SensorType_TEMPERATURE
#define _Sensor_SensorType_MAX Sensor_SensorType_LUMINOSITY
#define _Sensor_SensorType_ARRAYSIZE ((Sensor_SensorType)(Sensor_SensorType_LUMINOSITY+1))

typedef enum _CommandMessage_CommandType {
    CommandMessage_CommandType_GET_STATE = 0,
    CommandMessage_CommandType_SET_STATE = 1,
    CommandMessage_CommandType_SENSOR_STATE = 2
} CommandMessage_CommandType;
#define _CommandMessage_CommandType_MIN CommandMessage_CommandType_GET_STATE
#define _CommandMessage_CommandType_MAX CommandMessage_CommandType_SENSOR_STATE
#define _CommandMessage_CommandType_ARRAYSIZE ((CommandMessage_CommandType)(CommandMessage_CommandType_SENSOR_STATE+1))

/* Struct definitions */
typedef struct _Sensor {
    Sensor_SensorType type;
    int32_t id;
    float state;
/* @@protoc_insertion_point(struct:Sensor) */
} Sensor;

typedef struct _CommandMessage {
    CommandMessage_CommandType command;
    bool has_parameter;
    Sensor parameter;
/* @@protoc_insertion_point(struct:CommandMessage) */
} CommandMessage;

/* Default values for struct fields */

/* Initializer values for message structs */
#define Sensor_init_default                      {_Sensor_SensorType_MIN, 0, 0}
#define CommandMessage_init_default              {_CommandMessage_CommandType_MIN, false, Sensor_init_default}
#define Sensor_init_zero                         {_Sensor_SensorType_MIN, 0, 0}
#define CommandMessage_init_zero                 {_CommandMessage_CommandType_MIN, false, Sensor_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define Sensor_type_tag                          1
#define Sensor_id_tag                            2
#define Sensor_state_tag                         3
#define CommandMessage_command_tag               1
#define CommandMessage_parameter_tag             2

/* Struct field encoding specification for nanopb */
extern const pb_field_t Sensor_fields[4];
extern const pb_field_t CommandMessage_fields[3];

/* Maximum encoded size of messages (where known) */
#define Sensor_size                              18
#define CommandMessage_size                      22

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define SENSOR_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif