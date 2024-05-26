#ifndef BME280_LIB_H
#define BME280_LIB_H

#include <stdint.h>

#define ACK_EN  0x1
#define ACK_DIS 0x0

typedef struct 
{
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;

    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;

    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;

}BME280_CalibData_t;

extern BME280_CalibData_t oldCalibData;

typedef union{
    uint8_t buffer[33];

    struct 
    {
        uint16_t dig_t1;
        int16_t dig_t2;
        int16_t dig_t3;

        uint8_t unused[18];
        uint8_t unused2[9];
    } temperature;

    struct 
    {
        uint8_t unused[6];

        uint16_t dig_p1;
        int16_t dig_p2;
        int16_t dig_p3;
        int16_t dig_p4;
        int16_t dig_p5;
        int16_t dig_p6;
        int16_t dig_p7;
        int16_t dig_p8;
        int16_t dig_p9;

        uint8_t unused2[9];
    } pressure;

    struct 
    {
        uint8_t unused[6];
        uint8_t unused2[18];

        uint8_t dig_h1;
        int16_t dig_h2;
        uint8_t dig_h3;
        int16_t dig_h4;
        int16_t dig_h5;
        int8_t dig_h6;

    } humidity;
    
}BME280_CalibData_u;

extern BME280_CalibData_u calibData;

typedef struct {
    uint8_t data_msb;
    uint8_t data_lsb;
    uint8_t data_xlsb;
}BME280_DataRecvBytes_t;

extern BME280_DataRecvBytes_t recvBytes;

void BME280_Init(void);
int32_t BME280_GetTemp(void);
int32_t BME280_GetPress(void);
uint32_t BME280_GetHum(void);

#endif //BME280_LIB_H
