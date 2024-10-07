#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>

MPU6050 imu;
Madgwick filter;
unsigned long lastTime = 0;
float deltaTime = 0.0f;

int ax_offset, ay_offset, az_offset;
int gx_offset, gy_offset, gz_offset;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  imu.initialize();
  
  // Check if the MPU6050 is connected properly
  if (!imu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // Calibrate the MPU6050 to remove offsets and biases
  Serial.println("Calibrating MPU6050...");
  calibrateMPU6050();
  Serial.println("Calibration Complete!");

  // Set the calculated offsets
  imu.setXAccelOffset(ax_offset);
  imu.setYAccelOffset(ay_offset);
  imu.setZAccelOffset(az_offset);
  imu.setXGyroOffset(gx_offset);
  imu.setYGyroOffset(gy_offset);
  imu.setZGyroOffset(gz_offset);

  // Initialize Madgwick filter
  filter.begin(50);  // Use 50Hz sampling rate
}

void loop() {
  // Calculate delta time for each loop
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0f;
  lastTime = currentTime;

  // Read raw accelerometer and gyroscope data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw data into physical units
  float accelX = ax / 16384.0;  // +/-2g range
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  
  float gyroX = gx / 131.0 * DEG_TO_RAD;  // +/- 250 degrees/sec
  float gyroY = gy / 131.0 * DEG_TO_RAD;
  float gyroZ = gz / 131.0 * DEG_TO_RAD;

  // Update Madgwick filter with accelerometer and gyroscope data
  filter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);

  // Get the orientation angles from the Madgwick filter
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();

  // Print the angles
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print("\tPitch: "); Serial.print(pitch);
  Serial.print("\tYaw: "); Serial.println(yaw);

  delay(2000);  // Update at 50Hz
}

void calibrateMPU6050() {
  int numReadings = 1000;
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;

  // Collect multiple readings to average out the offset values
  for (int i = 0; i < numReadings; i++) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    axSum += ax;
    aySum += ay;
    azSum += az;
    gxSum += gx;
    gySum += gy;
    gzSum += gz;

    delay(3);  // Small delay between readings
  }

  // Calculate the average offsets

  ax_offset = axSum / numReadings;
  ay_offset = aySum / numReadings;
  az_offset = azSum / numReadings - 16384;  // Subtract 1g for Z-axis gravity compensation
  gx_offset = gxSum / numReadings;
  gy_offset = gySum / numReadings;
  gz_offset = gzSum / numReadings;

  // Print the offsets for debugging
  Serial.print("Accel X offset: "); Serial.println(ax_offset);
  Serial.print("Accel Y offset: "); Serial.println(ay_offset);
  Serial.print("Accel Z offset: "); Serial.println(az_offset);
  Serial.print("Gyro X offset: "); Serial.println(gx_offset);
  Serial.print("Gyro Y offset: "); Serial.println(gy_offset);
  Serial.print("Gyro Z offset: "); Serial.println(gz_offset);

}
