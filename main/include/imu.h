#include <math.h>

struct IMU {
       int nsamp;
       int cal_cnt;
       float pitch;
       float roll;
} imu;

static void imu_init (spi_device_handle_t device) 
{
   spi_write_byte( device, 0x19, 0x00); /* Sample Rate Div not used in rates >= 1K */
   spi_write_byte( device, 0x1a, 0x01); /* Config - fifo overflow, no sync*/
   spi_write_byte( device, 0x1b, 0x00); /* Gyro Config - range +/-250dps, bypass dlpf 8Ksamp 250LP*/
   spi_write_byte( device, 0x1c, 0x00); /* Accel Config - range +/-2g */
   spi_write_byte( device, 0x1d, 0x00); /* Accel Config 1 -x00 1khz/1ksamp 0x08=200hz x0a=100 x0e=5 x0f=420 */
   spi_write_byte( device, 0x23, 0x78); /* fifo enable - gyroscope and accel */
   spi_write_byte( device, 0x6b, 0x00); /* Power Management 1 */
   spi_write_byte( device, 0x6c, 0x00); /* Power Management 2 */
   spi_write_byte( device, 0x6a, 0x40); /* User Control - enable fifo*/
   spi_write_byte( device, 0x68, 0x07); /* Signal Path Reset */
}

void imu_read(void * unused)
{
    uint8_t temp_buffer[32];
    uint8_t buffer[1024];
    float calx=0, caly=0, calz=0, gcalx=0, gcaly=0;
    float pitch_last=0, roll_last=0;
    float xAccl_cal=0, yAccl_cal=0, zAccl_cal=0, xGyro_cal=0, yGyro_cal=0;
    float xAccl, yAccl, zAccl, xGyro, yGyro;

    float rate = 0.001;
    float xFusion_T = (0.50/rate)/(1+0.50/rate); 
    float yFusion_T = (0.50/rate)/(1+0.50/rate); 

    imu.nsamp = 0;
    spi_write_byte(vspi, 0x6a, 0x44); /* reset fifo */
    while (1) {
        spi_read_bytes(vspi, 0x72, 2, temp_buffer);
        int samples = 256 * temp_buffer[0] + temp_buffer[1];
        if(samples > 256)samples=256;  //max size of system
        samples = samples / 12;
        if(samples>0){ spi_read_bytes(vspi, 0x74, 12 * samples, buffer); }
        for (int n = 0; n <samples; n++){
            imu.nsamp++;
            //accelerometers raw calibrated and filtered - 1K sample rate (1/8 of gyro)
            xAccl = (float)(256 * buffer[12*n+0] + buffer[12*n+1]);
            if(xAccl>0x8000)xAccl = -1.0*(0xffff - xAccl) ; 
            xAccl = xAccl / 0x4000;
            xAccl = xAccl - xAccl_cal;

            yAccl = (float)(256 * buffer[12*n+2] + buffer[12*n+3]);
            if(yAccl>0x8000)yAccl = -1.0 * (0xffff - yAccl);
            yAccl = yAccl / 0x4000;
            yAccl = yAccl - yAccl_cal;

            zAccl = (float)(256 * buffer[12*n+4] + buffer[12*n+5]);
            if(zAccl>0x8000)zAccl = -1.0 * (0xffff - zAccl );
            zAccl = zAccl / 0x4000;
            zAccl = zAccl - zAccl_cal;

            //Gyros raw calibrated and Integral at full 8K sample rate
            xGyro = (float)(256 * buffer[12*n+6] + buffer[12*n+7]);
            if(xGyro>0x8000)xGyro = -1.0*(0xffff - xGyro) ; 
            xGyro = 0.001 * xGyro * 250.00 / 0x8000;

            yGyro = (float)(256 * buffer[12*n+8] + buffer[12*n+9]);
            if(yGyro>0x8000)yGyro = -1.0 * (0xffff - yGyro);
            yGyro = 0.001 * yGyro * 250.00 / 0x8000;

            //sensor fusion 'complimentary filter' LP filter on Accelerometer and HP filter on gyro
            if(xAccl< -1)xAccl = -1; if(xAccl> 1)xAccl = 1;
            if(yAccl< -1)yAccl = -1; if(yAccl> 1)yAccl = 1;
            imu.pitch = xFusion_T * pitch_last + (1 - xFusion_T) * 57.3 * asin(yAccl) + 1* (xGyro - xGyro_cal) ;
            pitch_last = imu.pitch;  
            imu.roll = yFusion_T * roll_last + (1 - yFusion_T) * -57.3 * asin(xAccl) + 1* (yGyro - yGyro_cal) ;
            roll_last = imu.roll;
            //if(imu.nsamp%1000==0){
            //    printf("%6d  %7.2f %7.2f %7.2f %7.2f %7.2f  ",imu.nsamp,xAccl,yAccl,zAccl,xGyro,yGyro);
            //    printf("    %7.2f %7.2f  ",imu.pitch, imu.roll);
            //    printf("\n");
            //}

            //calibration routine runs for about 1/4sec averages 500 measurements
            if (imu.cal_cnt <= 2000){
                if(imu.cal_cnt == 0 ) {
                   xAccl = 0; yAccl = 0; zAccl = 0; xGyro = 0; yGyro = 0;
                   xAccl_cal=0; yAccl_cal=0; zAccl_cal=0; xGyro_cal=0; yGyro_cal=0; 
                   calx = 0.0; caly = 0.0; calz = 0.0; gcalx = 0.0; gcaly = 0.0; } 
                else if(imu.cal_cnt >= 1500 && imu.cal_cnt < 2000) {
                   calx = calx + xAccl; caly = caly + yAccl; calz = calz + zAccl;
                   gcalx = gcalx + xGyro; gcaly = gcaly + yGyro; 
                }
                else if(imu.cal_cnt ==  2000) {
                   xAccl_cal = calx / 500; yAccl_cal = caly / 500; zAccl_cal = calz / 500;
                   xGyro_cal = gcalx / 500; yGyro_cal = gcaly / 500; 
                   zAccl_cal = zAccl_cal - 1.0;  //gravity orientation
                   printf( "cal   %f %f %f   %f %f\n", calx/500,caly/500,calz/500,gcalx/500,gcaly/500);
                }
                ++imu.cal_cnt;
            } //end of cal
        } //end of samples
    } //end of while 1
} //end of task
