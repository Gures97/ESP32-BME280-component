#include "bme280.h"

#include <stdio.h>
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "bme280_conf.h"

/*
All of calibration algorithms are from https://cdn-shop.adafruit.com/product-files/2652/2652.pdf
*/

BME280_CalibData_t oldCalibData;
BME280_DataRecvBytes_t recvBytes;

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_BME280_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = CONFIG_BME280_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(BME280_I2C_PORT, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(BME280_I2C_PORT, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t sendDataI2C(uint8_t* data, size_t size)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write(cmd, data, size, ACK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(BME280_I2C_PORT, cmd, CONFIG_BME280_TIMEOUT_TICKS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t  askForRegisters(uint8_t address)
{
    esp_err_t ret;
    uint8_t data[] = {
        CONFIG_BME280_SENSOR_ADDR << 1 | I2C_MASTER_WRITE,
        address
    };

    return sendDataI2C(data, sizeof(data));
}

static esp_err_t getReceivedData(uint8_t * buffer, size_t bufSize)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CONFIG_BME280_SENSOR_ADDR << 1 | I2C_MASTER_READ, ACK_EN);
    i2c_master_read(cmd, buffer,bufSize , I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(BME280_I2C_PORT, cmd, CONFIG_BME280_TIMEOUT_TICKS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t getRegisters(uint8_t address, uint8_t * buffer, size_t bufSize)
{
    esp_err_t ret;
    ret = askForRegisters(address);
    if (ESP_OK != ret) {
        return ret;
    }

    ret = getReceivedData(buffer, bufSize);
    return ret;
}

static esp_err_t getCalibData(void)
{
    esp_err_t ret;
    uint8_t buf[26];

    //calib 0x88 to 0x9F
    ret = getRegisters(0x88, buf, 24);
    if (ESP_OK != ret) {
        return ret;
    }

    oldCalibData.dig_t1 = (buf[1] << 8) + buf[0];
    oldCalibData.dig_t2 = (buf[3] << 8) + buf[2];
    oldCalibData.dig_t3 = (buf[5] << 8) + buf[4];
    oldCalibData.dig_p1 = (buf[7] << 8) + buf[6];
    oldCalibData.dig_p2 = (buf[9] << 8) + buf[8];
    oldCalibData.dig_p3 = (buf[11] << 8) + buf[10];
    oldCalibData.dig_p4 = (buf[13] << 8) + buf[12];
    oldCalibData.dig_p5 = (buf[15] << 8) + buf[14];
    oldCalibData.dig_p6 = (buf[17] << 8) + buf[16];
    oldCalibData.dig_p7 = (buf[19] << 8) + buf[18];
    oldCalibData.dig_p8 = (buf[21] << 8) + buf[20];
    oldCalibData.dig_p9 = (buf[23] << 8) + buf[22];

    //calib 0xA1
    ret = getRegisters(0xA1, buf, 1);
    if (ESP_OK != ret) {
        return ret;
    }

    oldCalibData.dig_h1 = buf[0];

    //calib 0xE1 to 0xF0
    ret = getRegisters(0xE1, buf, 7);
    if (ESP_OK != ret) {
        return ret;
    }

    oldCalibData.dig_h2 = (buf[1] << 8) + buf[0];
    oldCalibData.dig_h3 = buf[2];
    oldCalibData.dig_h4 = (buf[3] << 4) + (buf[4] & 0x0F);
    oldCalibData.dig_h5 = (buf[5] << 4) + ((buf[4] & 0xF0));
    oldCalibData.dig_h6 = buf[6];

    return ret;
}

static esp_err_t readTemp(void)
{
    esp_err_t ret;
    uint8_t buf[3];
    
    ret = getRegisters(0xFA, buf, 3);
    if (ESP_OK != ret) {
        return ret;
    }

    recvBytes.data_msb = buf[0];
    recvBytes.data_lsb = buf[1];
    recvBytes.data_xlsb = buf[2];

    return ret;
}

static esp_err_t readPress(void)
{
    esp_err_t ret;
    uint8_t buf[3];
    
    ret = getRegisters(0xF7, buf, 3);
    if (ESP_OK != ret) {
        return ret;
    }

    recvBytes.data_msb = buf[0];
    recvBytes.data_lsb = buf[1];
    recvBytes.data_xlsb = buf[2];

    return ret;
}

static esp_err_t readHum(void)
{
    esp_err_t ret;
    uint8_t buf[2];
    
    ret = getRegisters(0xFD, buf, 2);
    if (ESP_OK != ret) {
        return ret;
    }

    recvBytes.data_msb = buf[0];
    recvBytes.data_lsb = buf[1];

    return ret;
}

int32_t t_fine;
static int32_t calibrateTemp(int32_t adc)
{
    int32_t a, b, ret;
    a = (((adc>>3) - ((int32_t)oldCalibData.dig_t1<<1)) * ((int32_t)oldCalibData.dig_t2)) >> 11;
    b = ((((adc>>4) - ((int32_t)oldCalibData.dig_t1)) * ((adc>>4) - ((int32_t)oldCalibData.dig_t1))) >> 12 ) * ((int32_t)oldCalibData.dig_t3) >> 14;
    t_fine = a + b;
    ret = (t_fine * 5 + 128) >> 8;
    return ret;
}

static uint32_t calibratePress(int32_t adc)
{
    int32_t a, b;
    uint32_t ret;
    a = (((int32_t)t_fine) >> 1) - 64000;
    b = (((a>>1) * (a>>1)) >> 11) * (int32_t)oldCalibData.dig_p6;
    b = b + ((a * ((int32_t)oldCalibData.dig_p5)) << 1);
    b = (b >> 2) + (((int32_t)oldCalibData.dig_p4) << 16);
    a = (((oldCalibData.dig_p3 * (((a >> 2) * (a >> 2)) >> 13)) >> 3) + ((((int32_t)oldCalibData.dig_p2) * a) >> 1)) >> 18;
    a = ((((32768 + a))*((int32_t)oldCalibData.dig_p1)) >> 15);
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

    a = (((int32_t)oldCalibData.dig_p9) * ((int32_t)(((ret >> 3) * (ret >> 3)) >> 13))) >> 12;
    b = (((int32_t)(ret >> 2)) * ((int32_t)oldCalibData.dig_p8)) >> 13;

    ret = (uint32_t)((int32_t)ret + ((a + b + oldCalibData.dig_p7) >> 4));

    return ret;
}

static uint32_t calibrateHum(int32_t adc)
{
    int32_t v_x1;
    v_x1 = (t_fine - ((int32_t)76800));
    v_x1 = (((((adc << 14) - (((int32_t)oldCalibData.dig_h4) << 20) - (((int32_t)oldCalibData.dig_h5) * v_x1)) + ((int32_t)16384)) >> 15) * (((((((v_x1 * ((int32_t)oldCalibData.dig_h6)) >> 10) * (((v_x1 * ((int32_t)oldCalibData.dig_h3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)oldCalibData.dig_h2) + 8192) >> 14));
    v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15 )) >> 7) * ((int32_t)oldCalibData.dig_h1)) >> 4));
    v_x1 = (v_x1 < 0 ? 0 : v_x1);
    v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
    return (uint32_t)(v_x1>>12);
}

static void initMeasure(void)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    uint8_t data[] = {
        CONFIG_BME280_SENSOR_ADDR << 1 | I2C_MASTER_WRITE,
        0xF2,
        0x01, //osrs_h [2:0]
        0xF4,
        0x20 + 0x04 + 0x01 //first osrs_t[7:5] second osrs_p[4:2] third mode[1:0]
    };

    sendDataI2C(data, sizeof(data));
}

void BME280_Init(void)
{
    getCalibData();
}

int32_t BME280_GetTemp(void)
{
    uint32_t readValTemp;
    int32_t tempResult;

    initMeasure();

    readTemp();
    readValTemp = ((recvBytes.data_msb << 16) + (recvBytes.data_lsb << 8) + recvBytes.data_xlsb) >> 4;
    tempResult = calibrateTemp(readValTemp);

    return tempResult;
}

int32_t BME280_GetPress(void)
{
    uint32_t readValPress;
    int32_t pressResult;

    initMeasure();

    readPress();
    readValPress = ((recvBytes.data_msb << 16) + (recvBytes.data_lsb << 8) + recvBytes.data_xlsb) >> 4;
    pressResult = calibratePress(readValPress);
    
    return pressResult;
}

uint32_t BME280_GetHum(void)
{
    uint32_t readValHum;
    uint32_t humResult;

    initMeasure();

    readHum();
    readValHum = (recvBytes.data_msb << 8) + recvBytes.data_lsb;
    humResult = calibrateHum(readValHum);
    
    return humResult;
}