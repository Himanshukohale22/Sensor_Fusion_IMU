#include <Wire.h>
// #include <MPU6050.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include "math.h"

// MPU6050 mpu;
Adafruit_MPU6050 mpu;

Madgwick filter;
unsigned long lastTime = 0;
float deltatime = 0.0f;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // mpu.begin();

  // Check if IMU is working
  if (!mpu.begin()) {
    Serial.println("IMU connection failed.");
    while (1);
  }

  Serial.println("IMU Initialized!");
  filter.begin(50);  // Initialize the filter with a sample rate of 50Hz
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
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  //callback the error function for accurate calibration

  error_check();

}

void loop() {
  
  // Time delta for filter
  // unsigned long currentTime = millis();
  // deltatime = (currentTime - lastTime) / 1000.0f;
  // lastTime = currentTime;

  // Read accelerometer and gyroscope values
  // int16_t ax, ay, az;
  // int16_t gx, gy, gz;
  // imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax, ay, az;
  float gx, gy, gz;

  ax = a.acceleration.x;
  ay  = a.acceleration.y ;
  az= a.acceleration.z ;

  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;

  // Normalize accelerometer data (optional but improves accuracy)
  float accelX = (float)ax / 16384.0;  // assuming default range +/- 2g
  float accelY = (float)ay / 16384.0;
  float accelZ = (float)az / 16384.0;

  // Normalize gyroscope data (convert to radians per second)
  float gyroX = (float)gx / 131.0 * DEG_TO_RAD;  // assuming default range +/- 250deg/s
  float gyroY = (float)gy / 131.0 * DEG_TO_RAD;
  float gyroZ = (float)gz / 131.0 * DEG_TO_RAD;

  // Update the filter with new data
  filter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);

  // Get the orientation angles from the filter
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();

  // Print the angles
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("\tPitch: ");
  Serial.print(pitch);
  Serial.print("\tYaw: ");
  Serial.println(yaw);

  delay(500);  // Update at 50 Hz
  
}

void error_check(){
  // function for error values offset valuse and hardcode offset values
  //
  // sum 
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

        delay(10); // Delay between samples
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

