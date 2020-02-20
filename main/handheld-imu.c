#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "soc/gpio_struct.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/spi_common.h"
#include "driver/i2c.h"
#include "esp_event_loop.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <math.h>
#include <driver/adc.h>
#include "sdkconfig.h"

//wifi sockets etc requirements
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

//wifi requirements
#define EXAMPLE_WIFI_SSID "troutstream"
#define EXAMPLE_WIFI_PASS "password"
#define PORT 80
#include "esp_wifi.h"
#include "../components/wifisetup.h"

//i2c and periferal requirements
#include "driver/i2c.h"
#define QMC5883L_I2C_ADDR     0x0d //hmc5883 i2c address
#define i2c_frequency       500000 // max frequency of i2c clk
#define i2c_port                 0 //i2c channel on ESP-WROOM-32 ESP32S
#define i2c_gpio_scl            19 //D19 on ESP-WROOM-32 ESP32S
#define i2c_gpio_sda            18 //D18 on ESP-WROOM-32 ESP32S
#include "../components/i2c.h"
//#include "./i2c-bus/qmc5883l.h"

//tcp_server_task and globals declare
//does server setup and waits for http_request
//    index.html - returns webpage
//    GetData GET - reads http post into rx_buffer
//                  and returns outstr
char outstr[4096];
char rx_buffer[1024];
#include "../components/tcp_server_task.h"

#define BMP280_I2C_ADDR       0x76
int digT1, digT2, digT3;
int digP1, digP2, digP3, digP4, digP5, digP6, digP7, digP8, digP9;
#include "./i2c-bus/bmp280.h"
#include "./i2c-bus/qmc5883l.h"
#include "./i2c-bus/ssd1306.h"

//requirements for vspi attached to mpu9250
#define VSPI_MISO_PIN  17       //ad0
#define VSPI_MOSI_PIN  5        //sda
#define VSPI_SCLK_PIN  23       //scl
#define VSPI_CS_PIN    16       //ncs
#define VSPI_SPI_CLOCK 10000000  // 10 MHz - must be >> 1M for 8ksamp/sec
#include "./include/vspi.h"

//requirements for imu
#include "./include/imu.h"

float height = 3.72; 
float height_cal = 0;
float heading= 90.5;
float heading_cal = 0;
float xdisp = -1.6; float ydisp = -0.1;
int cnt = 0;

void app_main() {
    nvs_flash_init();
    initialise_wifi();  //in wifisetup.h
    wait_for_ip();      //in wifisetup.h
     
    vspi_init();         //spi for imu
    imu_init (vspi); 
    vTaskDelay(50/portTICK_RATE_MS);  

    i2cdetect();
    bmp280_cal();       //air pressure altitude measurements
    qmc5883_init();     //magnetometer heading measurements
    ssd1305_init();

    //start tasks
    xTaskCreatePinnedToCore (imu_read, "imu_read", 8096, NULL, 5, NULL, 1);
    vTaskDelay(1);
    xTaskCreatePinnedToCore (tcp_server_task, "tcp_server", 8096, NULL, 6, NULL, 0);

    char tmp[128];
    char *temp;
    imu.cal_cnt = 0;
    float height_cal=1010.1;
    char disp_str[128];
    while(1){
       //read rx_data and parse commands
       temp = strstr(rx_buffer, "cal=");
       if(temp)sscanf(temp,"cal=%d", &imu.cal_cnt);

       //construct outstr - outstr is gloabal that will be forwarded by tcp_server_task
       snprintf( outstr, sizeof tmp,"%d,", cnt);
       snprintf( tmp, sizeof tmp,"%4.2f,%4.2f,%4.2f,%4.2f",0.00,0.00,imu.pitch,imu.roll);
       strcat (outstr, tmp);  
       strcat ( outstr, "EOF\0");

       if(cnt == 10) height_cal = bmp280_read();
       qmc5883_init();     //magnetometer heading measurements
       printf("%7.3f  heading=%4ddeg  height=%7.2fft  roll=%7.2f   pitch=%7.2f\n", 
            (float)imu.nsamp/1000, qmc5883_read(), 28.4*(height_cal-bmp280_read()), imu.pitch, imu.roll);
       sprintf(disp_str,"4HandheldIMU||1    Heading    Height|4 %4d %5.1f||1    Pitch      Roll|4%5.1f %5.1f", 
              qmc5883_read(), 28.4*(height_cal-bmp280_read()),imu.pitch,imu.roll);
       ssd1305_text(disp_str);

       cnt++;
       vTaskDelay(500/portTICK_RATE_MS);  
    }

    removeDevice(vspi);
}
