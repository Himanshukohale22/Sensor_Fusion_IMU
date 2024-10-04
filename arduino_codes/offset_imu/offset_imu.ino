#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Offset variables
float ax_offset, ay_offset, az_offset;
float gx_offset, gy_offset, gz_offset;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.begin();


  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
 
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
 
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  

  Serial.println("");
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
  
  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);

  // float ax, ay, az;
  // float gx, gy, gz;

  // /* Print out the values */

  // ax = a.acceleration.x;
  // ay = a.acceleration.y;
  // az = a.acceleration.z;

  // gx = g.gyro.x;
  // gy = g.gyro.y;
  // gz = g.gyro.z;

  // Serial.println(ax);
  // Serial.println(ay);
  // Serial.println(az);

  // Serial.println(gx);
  // Serial.println(gy);
  // Serial.println(gz);
  
  // delay(2000);
    // Main loop can be used to read and process data
}
