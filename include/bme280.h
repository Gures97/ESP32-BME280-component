#ifndef BME280_LIB_H
#define BME280_LIB_H

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

extern BME280_CalibData_t calibData;

esp_err_t BME280_get_calib_data(i2c_port_t i2c_num);

#endif //BME280_LIB_H
