// ros2 humble with esp8266 
// with ros2arduino 
// imu = mpu6050 

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "math.h"


// offset values 
// Offset variables
float ax_offset, ay_offset, az_offset;
float gx_offset, gy_offset, gz_offset;

Adafruit_MPU6050 mpu;

float ax = 0.06;
float ay = -0.26;
float az = 10.73;
float gx = -0.04;
float gy = -0.03;
float gz = -0.04 ;

// 0.74
// -0.04
// 10.76
// -0.04
// -0.02
// -0.03

//offset values
// AX: 5.00
// AY: 0.00
// AZ: -1.00
// GX: 4.00
// GY: 0.00
// GZ: -22.00

// after offset calculation
// -4.15
// -0.02
// 11.70
// -4.04
// -0.02
// 21.97
 

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // Serial.print("Accelerometer range set to: ");
  // switch (mpu.getAccelerometerRange()) {
  // case MPU6050_RANGE_2_G:
  //   Serial.println("+-2G");
  //   break;
  // case MPU6050_RANGE_4_G:
  //   Serial.println("+-4G");
  //   break;
  // case MPU6050_RANGE_8_G:
  //   Serial.println("+-8G");
  //   break;
  // case MPU6050_RANGE_16_G:
  //   Serial.println("+-16G");
  //   break;
  // }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // Serial.print("Gyro range set to: ");
  // switch (mpu.getGyroRange()) {
  // case MPU6050_RANGE_250_DEG:
  //   Serial.println("+- 250 deg/s");
  //   break;
  // case MPU6050_RANGE_500_DEG:
  //   Serial.println("+- 500 deg/s");
  //   break;
  // case MPU6050_RANGE_1000_DEG:
  //   Serial.println("+- 1000 deg/s");
  //   break;
  // case MPU6050_RANGE_2000_DEG:
  //   Serial.println("+- 2000 deg/s");
  //   break;
  // }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  // Serial.print("Filter bandwidth set to: ");
  // switch (mpu.getFilterBandwidth()) {
  // case MPU6050_BAND_260_HZ:
  //   Serial.println("260 Hz");
  //   break;
  // case MPU6050_BAND_184_HZ:
  //   Serial.println("184 Hz");
  //   break;
  // case MPU6050_BAND_94_HZ:
  //   Serial.println("94 Hz");
  //   break;
  // case MPU6050_BAND_44_HZ:
  //   Serial.println("44 Hz");
  //   break;
  // case MPU6050_BAND_21_HZ:
  //   Serial.println("21 Hz");
  //   break;
  // case MPU6050_BAND_10_HZ:
  //   Serial.println("10 Hz");
  //   break;
  // case MPU6050_BAND_5_HZ:
  //   Serial.println("5 Hz");
  // //   break;
  // }

  // Serial.println("");
  // delay(100);
  delay(100);

    // Calibration
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  const int samples = 1000; // Number of samples to average

  for (int i = 0; i < samples; i++) {
        float ax, ay, az;
        float gx, gy, gz;

        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        delay(5); // Delay between samples
    }  

    ax_offset = ax_sum / samples;
    ay_offset = ay_sum / samples;
    az_offset = az_sum / samples; // Should be ~16384 (1g in 16g mode)
    gx_offset = gx_sum / samples;
    gy_offset = gy_sum / samples;
    gz_offset = gz_sum / samples;

    // Print offsets
    Serial.println("Offsets:");
    Serial.print("AX: "); Serial.println(ax_offset);
    Serial.print("AY: "); Serial.println(ay_offset);
    Serial.print("AZ: "); Serial.println(az_offset);
    Serial.print("GX: "); Serial.println(gx_offset);
    Serial.print("GY: "); Serial.println(gy_offset);
    Serial.print("GZ: "); Serial.println(gz_offset);

}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  // // accelaration 
  // float X = a.acceleration.x - ax;
  // float Y  = a.acceleration.y - ay;
  // float Z = a.acceleration.z - az;


  // float X = a.acceleration.x ;
  // float Y  = a.acceleration.y ;
  // float Z = a.acceleration.z ;

  
  float X = a.acceleration.x - ax_offset;
  float Y  = a.acceleration.y - ay_offset;
  float Z = a.acceleration.z - az_offset;


  // Serial.print("ax=");Serial.print(X);
  // Serial.print(" , ");
  // Serial.print("ay=");Serial.print(Y);
  // Serial.print(" , ");
  // Serial.print("az=");Serial.print(Z);
  // Serial.print(" , ");

  // Serial.print("ax=");
  Serial.print(X);
  Serial.print(" , ");
  // Serial.print("ay=");
  Serial.print(Y);
  Serial.print(" , ");
  // Serial.print("az=");
  Serial.print(Z);
  Serial.print(" , ");


  float angle_x = X/65.5;
  float angle_y = Y/65.5;
  float angle_z = Z/65.5;

  Serial.print("anglex: ");
  Serial.print(angle_x);
  Serial.print(" ");
  // Serial.print("ay=");
  Serial.print("angle_y");
  Serial.print(angle_y);
  Serial.print(" ");
  // Serial.print("az=");
  Serial.print("angle_z");
  Serial.print(angle_z);
  Serial.print(" ");


  // Rotation
  float rX = g.gyro.x - gx;
  float rY = g.gyro.y - gy;
  float rZ = g.gyro.z - gz;


  // float rX = g.gyro.x ;
  // float rY = g.gyro.y ;
  // float rZ = g.gyro.z ;

  float rX = g.gyro.x - gx_offset;
  float rY = g.gyro.y - gy_offset;
  float rZ = g.gyro.z - gz_offset;

  
  // Serial.print("rx=");
  Serial.print(rX);
  Serial.print(" , ");
  // Serial.print("ry=");
  Serial.print(rY);
  Serial.print(" , ");
  // Serial.print("rz=");
  Serial.print(rZ);

  // Serial.print("rx=");Serial.print(rX);
  // Serial.print(" , ");
  // Serial.print("ry=");Serial.print(rY);
  // Serial.print(" , ");
  // Serial.print("rz=");Serial.print(rZ);

  Serial.println("");
  delay(500);

}