#include <Wire.h>
#include <MPU6050.h>

MPU6050 imu;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int ax_offset, ay_offset, az_offset;
int gx_offset, gy_offset, gz_offset;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  imu.initialize();
  
  // Check if IMU is connected
  if (!imu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  Serial.println("Starting Calibration...");
  calibrateMPU6050();
  Serial.println("Calibration done!");
  
  // Set the calculated offsets
  imu.setXAccelOffset(ax_offset);
  imu.setYAccelOffset(ay_offset);
  imu.setZAccelOffset(az_offset);
  imu.setXGyroOffset(gx_offset);
  imu.setYGyroOffset(gy_offset);
  imu.setZGyroOffset(gz_offset);
}

void loop() {
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Display corrected readings
  Serial.print("Accel: ");
  Serial.print(" X: "); Serial.print(ax); 
  Serial.print(" Y: "); Serial.print(ay); 
  Serial.print(" Z: "); Serial.println(az);

  Serial.print("Gyro: ");
  Serial.print(" X: "); Serial.print(gx);
  Serial.print(" Y: "); Serial.print(gy);
  Serial.print(" Z: "); Serial.println(gz);

  delay(500);
}

void calibrateMPU6050() {
  int numReadings = 1000;
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;

  for (int i = 0; i < numReadings; i++) {
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    axSum += ax;
    aySum += ay;
    azSum += az;
    gxSum += gx;
    gySum += gy;
    gzSum += gz;

    delay(3);  // Small delay between readings
  }

  // Average the sums
  ax_offset = axSum / numReadings;
  ay_offset = aySum / numReadings;
  az_offset = azSum / numReadings - 16384; // Account for gravity
  gx_offset = gxSum / numReadings;
  gy_offset = gySum / numReadings;
  gz_offset = gzSum / numReadings;

  // Print the calculated offsets
  Serial.print("Accel X offset: "); Serial.println(ax_offset);
  Serial.print("Accel Y offset: "); Serial.println(ay_offset);
  Serial.print("Accel Z offset: "); Serial.println(az_offset);
  Serial.print("Gyro X offset: "); Serial.println(gx_offset);
  Serial.print("Gyro Y offset: "); Serial.println(gy_offset);
  Serial.print("Gyro Z offset: "); Serial.println(gz_offset);
}
