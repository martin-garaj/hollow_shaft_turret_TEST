#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include "cnc_pfm_driver.h"


// 
#define LED_PIN             13

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"
#include "Arduino.h"
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

bool flip_flop;

void setup()
{
  Serial.begin(115200);
  while (!Serial) 
  { 
    ; // Wait for serial to connect 
  }
  Serial.println("Running.");
  delay(200);

  // IMU
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  setup_pfm();
  // enable cnc
  init_cnc_pins();
  set_cnc_enable();
  // initial direction

  Serial.print("init  \t");
  Serial.print(int32_t(pfm[PFM_X].pfm_delta_steps)); Serial.print("\t");
  Serial.print(int32_t(pfm[PFM_X].pfm_target_freq)); Serial.print("\t");
  Serial.print(int32_t(pfm[PFM_X].pfm_direction)); Serial.print("\t");
  Serial.print(int32_t(pfm[PFM_X].pfm_target_delta)); Serial.print("\t");
  Serial.print("\n");

  control_pfm_target_delta(PFM_X, 2, 3200);
}


void loop()
{

  // if(get_pfm_delta_steps(PFM_X) > 3200)
  // {
  //   control_pfm_target_freq(PFM_X, 2, false);
  // }
  // if(get_pfm_delta_steps(PFM_X) <= 0)
  // {
  //   control_pfm_target_freq(PFM_X, 4, true);
  // }


  // if(get_pfm_delta_steps(PFM_X) == 3200)
  // {
  //   control_pfm_target_delta(PFM_X, 2, 0);
  // }

  // if(get_pfm_delta_steps(PFM_X) == 0)
  // {
  //   control_pfm_target_delta(PFM_X, 2, 3200);
  // }

  // Serial.print("state \t");
  // Serial.print(int32_t(pfm[PFM_X].pfm_delta_steps)); Serial.print("\t");
  // Serial.print(int32_t(pfm[PFM_X].pfm_target_freq)); Serial.print("\t");
  // Serial.print(int32_t(pfm[PFM_X].pfm_direction)); Serial.print("\t");
  // Serial.print(int32_t(pfm[PFM_X].pfm_target_delta)); Serial.print("\t");
  // Serial.print("\n");

  // read raw accel/gyro measurements from device
  int t1 = micros();
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  int t2 = micros();

  Serial.print("a/g/m:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.print(gz); Serial.print("\t");
  Serial.print(mx); Serial.print("\t");
  Serial.print(my); Serial.print("\t");
  Serial.print(mz); Serial.print("\t");
  Serial.print(t2-t1); Serial.print("\t");
  Serial.print(float(get_pfm_delta_steps(PFM_X))/1600.0, 10); Serial.print("\t");
  Serial.print("\n");


  // delay(100);

}
