#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;

#define G_TO_ACCEL 9.81
#define MGAUSS_TO_UTESLA 0.1
#define UTESLA_TO_TESLA 0.000001

#define ACCEL_SCALE 1 / 16384 // LSB/g
#define GYRO_SCALE 1 / 131 // LSB/(deg/s)
#define MAG_SCALE 0.3 // uT/LSB

#include <ros.h>
#include <ros/time.h>
#include "std_msgs/Float64MultiArray.h"
ros::NodeHandle  nh;

std_msgs::Float64MultiArray imu_msg;
ros::Publisher imu_pub("/Arduino/raw_imu", &imu_msg);
float imu_data[6] = {0.0};

uint32_t seq;

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(imu_pub);
  
  accelgyro.initialize();
  seq = 0;
}

void loop()

{
    static unsigned long prev_imu_time = 0;
    if ((millis() - prev_imu_time) >= (50))
    {    
        seq++;
         accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        imu_data[0] = ax * (double) ACCEL_SCALE * G_TO_ACCEL;
        imu_data[1] = ay * (double) ACCEL_SCALE * G_TO_ACCEL;
        imu_data[2] = az * (double) ACCEL_SCALE * G_TO_ACCEL;
        imu_data[3] = gx * (double) GYRO_SCALE * DEG_TO_RAD;
        imu_data[4] = gy * (double) GYRO_SCALE * DEG_TO_RAD;
        imu_data[5] = gz * (double) GYRO_SCALE * DEG_TO_RAD;
        imu_msg.data_length = 6;
        imu_msg.data = imu_data;

        imu_pub.publish( &imu_msg );

        prev_imu_time = millis();
         
    }

 nh.spinOnce();
}