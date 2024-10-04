#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Variables to store raw values
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Offset variables
int16_t ax_offset, ay_offset, az_offset;
int16_t gx_offset, gy_offset, gz_offset;

void setup() {
  Serial.begin(115200);
  
  // Initialize MPU6050
  mpu.initialize();
  
  // Read initial values for offset calculation
  calculateOffsets();
}

void loop() {
  // Read raw values from the sensor
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate angles from raw values
  float angleX = atan2(ay, az) * 180 / PI; // Roll
  float angleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI; // Pitch
  float angleZ = gx / 131.0; // Gyroscope value converted to degrees

  // Print results
  Serial.print("Ax: "); Serial.print(ax - ax_offset);
  Serial.print(" Ay: "); Serial.print(ay - ay_offset);
  Serial.print(" Az: "); Serial.print(az - az_offset);
  Serial.print(" Gx: "); Serial.print(gx - gx_offset);
  Serial.print(" Gy: "); Serial.print(gy - gy_offset);
  Serial.print(" Gz: "); Serial.println(gz - gz_offset);
  
  Serial.print("Angle X: "); Serial.print(angleX);
  Serial.print(" Angle Y: "); Serial.print(angleY);
  Serial.print(" Angle Z: "); Serial.println(angleZ);

  delay(500);
}

void calculateOffsets() {
  // Calculate the average offsets for calibration
  int16_t ax_sum = 0, ay_sum = 0, az_sum = 0;
  int16_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
  const int samples = 100;

  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    delay(10);
  }

  ax_offset = ax_sum / samples;
  ay_offset = ay_sum / samples;
  az_offset = az_sum / samples;
  gx_offset = gx_sum / samples;
  gy_offset = gy_sum / samples;
  gz_offset = gz_sum / samples;
}
