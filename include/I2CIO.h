// I2C_input_packet is nothing more than a pack of int we shouldn't have alignment issues with sizeof
// try to make this as portable as possible without messing up with struct alignments

#ifndef __I2C_STRUCT__
#define __I2C_STRUCT__


typedef struct I2C_input_packet
{
    int16_t left_motor_speed;
    int16_t right_motor_speed;
    int8_t use_pid;
    uint8_t crc;
} I2C_input_packet;

typedef struct I2C_output_packet
{
    uint16_t left_encoder;
    uint16_t right_encoder;
    uint8_t __dont_use_this_padding_byte;
    uint8_t crc;
} I2C_output_packet;

#endif
