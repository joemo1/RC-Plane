#include <Arduino.h>
#include "Wire.h"

#define I2C_DEV_ADDR 0x68
#define SIZE 14
#define I2C_SDA 21
#define I2C_SCL 22
#define SERVO_PIN 18
#define SERVO_CHANNEL 0
uint32_t i = 0;


void setup() {
  Serial.begin(115200);
  Serial.printf("Serial monitor on\n");

  // Servo set up
  ledcSetup(SERVO_CHANNEL, 50, 14);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);

  Wire.begin(I2C_SDA, I2C_SCL);
  // 0. Wake IMU from sleep
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00); // Write 0x00 into the address 0x6B (PWR_MGMT_1) to wake IMU.
  uint8_t error = Wire.endTransmission(true); // The 'true' bool here tells the device to send a 'STOP' signal rather than a repeated 'START' signal (both are accepted by this I2C device)
  Serial.printf("Error message: %d\n", error);
}

void loop() {
  delay(100);
  i++;
  Serial.printf("Loop %d\n", i);
  // 1. Write the address of first byte which is 0x3B in datasheet (Accel X High)
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(0x3B); // Address of first byte to be read. See datasheet - the first part of one of these I2C messages is to write the address that we begin reading/writing at
  Wire.endTransmission(false); // Ending message with another START signal
  // No 'endTranmission' as we want to continue the message
  // 2. Read stage - read 14 bytes or so of data
  uint8_t buffer[SIZE];
  uint8_t receivedData = Wire.requestFrom(I2C_DEV_ADDR, sizeof(buffer)); // receiving data in bytes from MPU6050 and put it onto the buffer stored in Wire object
  size_t numBytesReturned = Wire.readBytes(buffer, SIZE); // Take 'SIZE' number of bytes and move it from Wire buffer to 'buffer'

  Serial.printf("Number of bytes returned: %zu\n", numBytesReturned);

  // 3. Interpret bytes and put together high and low bytes


  Serial.printf("Buffer: ");
  for (int j=0; j<SIZE; j++) {
    Serial.printf("%d\n", buffer[j]);
  }
  // Put together low and high bytes
  int16_t accelX = buffer[1] + (buffer[0]<<8);
  Serial.printf("accelX: %d\n", accelX);
  int16_t accelY = buffer[3] + (buffer[2]<<8);
  int16_t accelZ = buffer[5] + (buffer[4]<<8);
  // Convert to g
  float accelX_g = (float) accelX/16384;
  float accelY_g = (float) accelY/16384;
  float accelZ_g = (float) accelZ/16384;
  
  Serial.printf("accelX_g: %f\n", accelX_g);

  float accelToServoX = ( ( (accelX_g) / 2 ) + 1.5 ) * 819;
  Serial.printf("accelToServoX: %f\n", accelToServoX);

  ledcWrite(SERVO_CHANNEL, accelToServoX);
}