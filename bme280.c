#include "bme280.h"

#include <stdio.h>
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "bme280_conf.h"

/*
All of calibration algorithms are from https://cdn-shop.adafruit.com/product-files/2652/2652.pdf
*/

BME280_CalibData_t calibData;

static esp_err_t askI2cForArray(uint8_t addr, uint8_t * buff, size_t buffLen)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BME280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x88, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(BME280_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    //wait for response
    vTaskDelay(30 / portTICK_PERIOD_MS);
    //read data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BME280_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, buff, buffLen, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(BME280_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

esp_err_t BME280_get_calib_data(i2c_port_t i2c_num)
{
    printf("Calibration data start\n");
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    uint8_t buf[26];
    //calib 0x88 to 0xA1
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BME280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x88, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    //wait for response
    vTaskDelay(30 / portTICK_PERIOD_MS);
    //read data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BME280_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, buf,26 , I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    printf("Calibration 0x88 to 0xA1\n");
    disp_buf(buf, 26);

    calibData.dig_t1 = (buf[1] << 8) + buf[0];
    calibData.dig_t2 = (buf[3] << 8) + buf[2];
    calibData.dig_t3 = (buf[5] << 8) + buf[4];
    calibData.dig_p1 = (buf[7] << 8) + buf[6];
    calibData.dig_p2 = (buf[9] << 8) + buf[8];
    calibData.dig_p3 = (buf[11] << 8) + buf[10];
    calibData.dig_p4 = (buf[13] << 8) + buf[12];
    calibData.dig_p5 = (buf[15] << 8) + buf[14];
    calibData.dig_p6 = (buf[17] << 8) + buf[16];
    calibData.dig_p7 = (buf[19] << 8) + buf[18];
    calibData.dig_p8 = (buf[21] << 8) + buf[20];
    calibData.dig_p9 = (buf[23] << 8) + buf[22];
    calibData.dig_h1 = buf[25];

    //calib 0xE1 to 0xF0
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BME280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xE1, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    //wait for response
    vTaskDelay(30 / portTICK_PERIOD_MS);
    //read data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BME280_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, buf,7 , I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    printf("Calibration 0xE1 to 0xF0\n");
    disp_buf(buf, 7);

    calibData.dig_h2 = (buf[0] << 8) + buf[1];
    calibData.dig_h3 = buf[2];
    calibData.dig_h4 = (buf[3] << 4) + (buf[4] & 0x0F);
    calibData.dig_h5 = ((buf[4] & 0xF0) << 8) + buf[5];
    calibData.dig_h6 = buf[6];

    disp_calib_data();

    return ret;
}

static esp_err_t i2c_master_BME280_read_temp(i2c_port_t i2c_num, uint8_t *data_msb, uint8_t *data_lsb, uint8_t *data_xlsb)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BME280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xFA, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(1);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BME280_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_msb, ACK_VAL);
    i2c_master_read_byte(cmd, data_lsb, ACK_VAL);
    i2c_master_read_byte(cmd, data_xlsb, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t BME280_read_press(i2c_port_t i2c_num, uint8_t *data_msb, uint8_t *data_lsb, uint8_t *data_xlsb)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BME280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xF7, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(1);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BME280_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_msb, ACK_VAL);
    i2c_master_read_byte(cmd, data_lsb, ACK_VAL);
    i2c_master_read_byte(cmd, data_xlsb, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int32_t t_fine;
int32_t BME280_calibrate_temp(int32_t adc)
{
    int32_t a, b, ret;
    a = (((adc>>3) - ((int32_t)calibData.dig_t1<<1)) * ((int32_t)calibData.dig_t2)) >> 11;
    b = ((((adc>>4) - ((int32_t)calibData.dig_t1)) * ((adc>>4) - ((int32_t)calibData.dig_t1))) >> 12 ) * ((int32_t)calibData.dig_t3) >> 14;
    t_fine = a + b;
    ret = (t_fine * 5 + 128) >> 8;
    return ret;
}

uint32_t BME280_calibrate_pressure(int32_t adc)
{
    int32_t a, b;
    uint32_t ret;
    a = (((int32_t)t_fine) >> 1) - 64000;
    b = (((a>>1) * (a>>1)) >> 11) * (int32_t)calibData.dig_p6;
    b = b + ((a * ((int32_t)calibData.dig_p5)) << 1);
    b = (b >> 2) + (((int32_t)calibData.dig_p4) << 16);
    a = (((calibData.dig_p3 * (((a >> 2) * (a >> 2)) >> 13)) >> 3) + ((((int32_t)calibData.dig_p2) * a) >> 1)) >> 18;
    a = ((((32768 + a))*((int32_t)calibData.dig_p1)) >> 15);
    if(0 == a) 
    {
        return 0;
    }
    ret = (((uint32_t)(((int32_t)1048576) - adc ) - (b >> 12))) * 3125;
    if(ret < 0x80000000) 
    {
        ret = (ret << 1) / ((uint32_t)a);
    }
    else 
    {
        ret = (ret / ((uint32_t)a)) * 2;
    }

    a = (((int32_t)calibData.dig_p9) * ((int32_t)(((ret >> 3) * (ret >> 3)) >> 13))) >> 12;
    b = (((int32_t)(ret >> 2)) * ((int32_t)calibData.dig_p8)) >> 13;

    ret = (uint32_t)((int32_t)ret + ((a + b + calibData.dig_p7) >> 4));

    return ret;
}

void BME280_measure(i2c_port_t i2c_num)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BME280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xF2, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xF4, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x20 + 0x04 + 0x01, ACK_CHECK_EN);//first osrs_t[7:5] second osrs_p[4:2] third mode[1:0]
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}