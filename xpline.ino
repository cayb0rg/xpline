#include <Wire.h>
#include <FastLED.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Globals.h"
#include "Spider.h"
#include "WiFi.h"

bool verbose = true;

// PIR Motion Sensor
#define MOTION_SENSOR_PIN 2

// Photoresistor
#define ANALOG_READ_PIN A0

// LED Strip
#define LED_PIN 13
CRGB leds[NUM_LEDS];
CRGB LED_color = CRGB(0, 0, 255);

/*
   Base MPU6050 Wiring
   VCC = 3.3V
   GND = GND
   SCL = D1
   SDA = D2
   AD0 = D3

   Joystick MPU6050 Wiring
   SCL = D1
   SDA = D2
   AD0 = D4
*/
MPU6050 mpu_base(0x69);
MPU6050 mpu_joystick(0x68);

#define MPU_BASE_AD0 14

// accelerometer values
int16_t ax_base, ay_base, az_base;
int16_t ax_joy, ay_joy, az_joy;
int base_ax, base_ay, base_az;
int joystick_ax, joystick_ay, joystick_az;

// gyroscope values
int16_t gx_base, gy_base, gz_base;
int16_t gx_joy, gy_joy, gz_joy;
int base_gx, base_gy, base_gz;
int joystick_gx, joystick_gy, joystick_gz;

// quaternion deadzone
float deadzone = 0.07;

bool movedJoystick = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q_;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// MPU control/status vars
bool base_dmpReady = false;  // set true if DMP init was successful
uint8_t base_mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t base_devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t base_packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t base_fifoCount;     // count of all bytes currently in FIFO
uint8_t base_fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion base_q_;           // [w, x, y, z]         quaternion container
VectorInt16 base_aa;         // [x, y, z]            accel sensor measurements
VectorInt16 base_aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 base_aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat base_gravity;    // [x, y, z]            gravity vector
float base_euler[3];         // [psi, theta, phi]    Euler angle container
float base_ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

enum Output {
  QUATERNION_VALS,
  EULER_VALS,
  ACCELERATION_VALS,
  LINEAR_ACCELERATION_VALS,
  GRAVITY_VALS,
  WORLD_ACCELERATION_VALS,
  NONE
} output_type;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // LEDs
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(20);

  // Spiders
  initSpiderData();

  pinMode(MOTION_SENSOR_PIN, INPUT);
  pinMode(MPU_BASE_AD0, OUTPUT);

  // Set the IC2 address of the joystick MPU to 0x68
  // This is the default value so we won't touch it
  //digitalWrite(MPU_JOYSTICK_AD0, LOW);
  // Set the IC2 address of the base MPU to 0x69
  digitalWrite(MPU_BASE_AD0, HIGH);

  // Initialize MPU6050's
  mpu_base.initialize();
  mpu_joystick.initialize();
  Serial.println(mpu_base.testConnection() ? "MPU6050 BASE CONNECTED."
                 : "... Are we supposed to be connecting to something?");
  Serial.println(mpu_joystick.testConnection() ? "MPU6050 JOYSTICK CONNECTED."
                 : "... Are we supposed to be connecting to something?");

  // Set gyro offsets
  mpu_base.setXGyroOffset(0);
  mpu_base.setYGyroOffset(0);
  mpu_base.setZGyroOffset(0);
  mpu_base.setXAccelOffset(0);
  mpu_base.setYAccelOffset(0);
  mpu_base.setZAccelOffset(0);
  mpu_joystick.setXGyroOffset(0);
  mpu_joystick.setYGyroOffset(0);
  mpu_joystick.setZGyroOffset(0);
  mpu_joystick.setZAccelOffset(0);

  // Load the DMP
  loadDMPBase(&mpu_base);
  loadDMPJoystick(&mpu_joystick);

  // Setup Wifi Server
  setupServer();
}

void getMotionSensorInput()
{
  // Motion Sensor
  if (digitalRead(MOTION_SENSOR_PIN) == HIGH) {
    Serial.println("MOVEMENT! UNLEASH THE SPIDERS!");
    // Wake the ESP8266 from deep sleep
  }
  else {
    Serial.println("NO MOVEMENT DETECTED.");
  }
}

void loadDMPJoystick(MPU6050 *mpu) {
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu->dmpInitialize();

  // make sure dmpInitialize worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu->CalibrateAccel(6);
    mpu->CalibrateGyro(6);
    mpu->PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu->setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready!"));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu->dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loadDMPBase(MPU6050 *mpu) {
  Serial.println(F("Initializing DMP..."));
  base_devStatus = mpu->dmpInitialize();

  // make sure dmpInitialize worked (returns 0 if so)
  if (base_devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu->CalibrateAccel(6);
    mpu->CalibrateGyro(6);
    mpu->PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu->setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready!"));
    base_dmpReady = true;

    // get expected DMP packet size for later comparison
    base_packetSize = mpu->dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(base_devStatus);
    Serial.println(F(")"));
  }
}

void getGyroInput(MPU6050 *mpu1, MPU6050 *mpu2)
{
  // Get first MPU6050 input
  mpu1->getMotion6(&gx_base, &gy_base, &gz_base, &ax_base, &ay_base, &az_base);
  base_ax = map(ax_base, -17000, 17000, 0, NUM_LEDS);
  base_ay = map(ay_base, -17000, 17000, 0, NUM_LEDS);
  base_az = map(az_base, -17000, 17000, 0, NUM_LEDS);
  base_gx = map(gx_base, -17000, 17000, 0, NUM_LEDS);
  base_gy = map(gy_base, -17000, 17000, 0, NUM_LEDS);
  base_gz = map(gz_base, -17000, 17000, 0, NUM_LEDS);

  // Get second MPU6050 input
  mpu2->getMotion6(&gx_joy, &gy_joy, &gz_joy, &ax_joy, &ay_joy, &az_joy);
  joystick_ax = map(ax_joy, -17000, 17000, 0, NUM_LEDS);
  joystick_ay = map(ay_joy, -17000, 17000, 0, NUM_LEDS);
  joystick_az = map(az_joy, -17000, 17000, 0, NUM_LEDS);
  joystick_gx = map(gx_joy, -17000, 17000, 0, NUM_LEDS);
  joystick_gy = map(gy_joy, -17000, 17000, 0, NUM_LEDS);
  joystick_gz = map(gz_joy, -17000, 17000, 0, NUM_LEDS);


  Serial.print("ax1: ");
  Serial.println(ax_base);
  Serial.print("ax2: ");
  Serial.println(ax_joy);
  Serial.print("ay1: ");
  Serial.println(ay_base);
  Serial.print("ay2: ");
  Serial.println(ay_joy);
  Serial.print("az1: ");
  Serial.println(az_base);
  Serial.print("az2: ");
  Serial.println(az_joy);
  Serial.print("gx1: ");
  Serial.println(gx_base);
  Serial.print("gx2: ");
  Serial.println(gx_joy);
  Serial.print("gy1: ");
  Serial.println(gy_base);
  Serial.print("gy2: ");
  Serial.println(gy_joy);
  Serial.print("gz1: ");
  Serial.println(gz_base);
  Serial.print("gz2: ");
  Serial.println(gz_joy);
}

void readFifoBuffer_(MPU6050 *mpu) {
  // Clear the buffer so as we can get fresh values
  // The sensor is running a lot faster than our sample period
  mpu->resetFIFO();

  // get current FIFO count
  fifoCount = mpu->getFIFOCount();

  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu->getFIFOCount();

  // read a packet from FIFO
  mpu->getFIFOBytes(fifoBuffer, packetSize);

  switch (output_type)
  {
    case QUATERNION_VALS:
      // get quarternion
      mpu->dmpGetQuaternion(&q_, fifoBuffer);
      if (verbose) {
        Serial.print("quat\t");
        Serial.print(q_.w);
        Serial.print("\t");
        Serial.print(q_.x);
        Serial.print("\t");
        Serial.print(q_.y);
        Serial.print("\t");
        Serial.println(q_.z);
      }
      break;
    case EULER_VALS:
      // get euler angles
      mpu->dmpGetEuler(euler, &q_);
      if (verbose) {
        Serial.print("euler\t");
        Serial.print(euler[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(euler[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(euler[2] * 180 / M_PI);
      }
      break;
    case ACCELERATION_VALS:
      // get acceleration
      mpu->dmpGetAccel(&aa, fifoBuffer);
      if (verbose) {
        Serial.print("aa\t");
        Serial.print(aa.x);
        Serial.print("\t");
        Serial.print(aa.y);
        Serial.print("\t");
        Serial.println(aa.z);
      }
      break;
    case LINEAR_ACCELERATION_VALS:
      // get acceleration
      mpu->dmpGetAccel(&aa, fifoBuffer);
      mpu->dmpGetGravity(&gravity, &q_);
      mpu->dmpGetLinearAccel(&aaReal, &aa, &gravity);
      if (verbose) {
        Serial.print("areal\t");
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.print(aaReal.y);
        Serial.print("\t");
        Serial.println(aaReal.z);
      }
      break;
    case WORLD_ACCELERATION_VALS:
      // get world-frame acceleration
      mpu->dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q_);
      if (verbose) {
        Serial.print("aworld\t");
        Serial.print(aaWorld.x);
        Serial.print("\t");
        Serial.print(aaWorld.y);
        Serial.print("\t");
        Serial.println(aaWorld.z);
      }
      break;
    default:
      break;
  }
}

void readFifoBufferBase(MPU6050 *mpu) {
  // Clear the buffer so as we can get fresh values
  // The sensor is running a lot faster than our sample period
  mpu->resetFIFO();

  // get current FIFO count
  base_fifoCount = mpu->getFIFOCount();

  // wait for correct available data length, should be a VERY short wait
  while (base_fifoCount < base_packetSize) base_fifoCount = mpu->getFIFOCount();

  // read a packet from FIFO
  mpu->getFIFOBytes(base_fifoBuffer, base_packetSize);

  switch (output_type)
  {
    case QUATERNION_VALS:
      // get quarternion
      mpu->dmpGetQuaternion(&base_q_, base_fifoBuffer);
      if (verbose) {
        Serial.print("quat\t");
        Serial.print(base_q_.w);
        Serial.print("\t");
        Serial.print(base_q_.x);
        Serial.print("\t");
        Serial.print(base_q_.y);
        Serial.print("\t");
        Serial.println(base_q_.z);
      }
      break;
    case EULER_VALS:
      // get euler angles
      mpu->dmpGetEuler(base_euler, &base_q_);
      if (verbose) {
        Serial.print("euler\t");
        Serial.print(base_euler[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(base_euler[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(base_euler[2] * 180 / M_PI);
      }
      break;
    case ACCELERATION_VALS:
      // get acceleration
      mpu->dmpGetAccel(&base_aa, base_fifoBuffer);
      if (verbose) {
        Serial.print("aa\t");
        Serial.print(base_aa.x);
        Serial.print("\t");
        Serial.print(base_aa.y);
        Serial.print("\t");
        Serial.println(base_aa.z);
      }
      break;
    case LINEAR_ACCELERATION_VALS:
      // get acceleration
      mpu->dmpGetAccel(&base_aa, base_fifoBuffer);
      mpu->dmpGetGravity(&base_gravity, &base_q_);
      mpu->dmpGetLinearAccel(&base_aaReal, &base_aa, &base_gravity);
      if (verbose) {
        Serial.print("areal\t");
        Serial.print(base_aaReal.x);
        Serial.print("\t");
        Serial.print(base_aaReal.y);
        Serial.print("\t");
        Serial.println(base_aaReal.z);
      }
      break;
    case WORLD_ACCELERATION_VALS:
      // get world-frame acceleration
      mpu->dmpGetLinearAccelInWorld(&base_aaWorld, &base_aaReal, &base_q_);
      if (verbose) {
        Serial.print("aworld\t");
        Serial.print(base_aaWorld.x);
        Serial.print("\t");
        Serial.print(base_aaWorld.y);
        Serial.print("\t");
        Serial.println(base_aaWorld.z);
      }
      break;
    default:
      break;
  }

}

void getAnalogInput()
{
  int val = analogRead(ANALOG_READ_PIN);
  //  Serial.println(val);
  // Values from [0, 1024]
  // 0 means no light (light level 0)
  // 1024 means all the light (light level 15)
  if (val < 200)
  {
    // technically monster mobs spawn at a light level of 0,
    // but given that's nearly impossible to recreate....
    spider_spawn = true;
  }
  else {
    spider_spawn = false;
  }

  // At light level 11 and below, spiders become hostile
  if (val <= (1024 / 15) * 11) {
    spider_hostile = true;
    spider_speed = 5;
    // Serial.println(">:E");
  } else
  {
    spider_hostile = false;
    spider_speed = 2;
    // Serial.println(":)");
  }

  //  float voltage = val * (3.3 / 1023.0);
  //
  //  Serial.println(voltage);
}

void updateXP() {
  LED_XP = numSpidersKilled * spider_XP;

  int diff = 0;
  if (LED_level < 16) {
    diff = 2;
  } else if (LED_level < 32) {
    diff = 5;
  } else {
    diff = 9;
  }
  int diffOfXPbetweenLevels = (7 + (LED_level) * diff);
  if (LED_XP - LED_lastLevelXP > diffOfXPbetweenLevels) {
    LED_lastLevelXP = LED_lastLevelXP + diffOfXPbetweenLevels;
    LED_level += 1;
  }
}


void loop() {
  // Get light sensor data

  // getGyroInput(&mpu_base, &mpu_joystick);

  // Get light data
  getAnalogInput();

  // Get gyroscope data
  output_type = QUATERNION_VALS;
  verbose = false;
  if (dmpReady)
  {
    readFifoBuffer_(&mpu_joystick);
    readFifoBufferBase(&mpu_base);
  }

  getMotionSensorInput();

  if (abs(q_.y - base_q_.y) > deadzone) {
    movedJoystick = true;
    Serial.println("Player is moving");
  }
  else
  {
    Serial.println("Player is still");
    movedJoystick = false;
  }

  // Calculate player direction
  if (movedJoystick) {
    if (q_.y - base_q_.y < 0) {
      LED_dir = -1;
    }
    else if (q_.y - base_q_.y > 0) {
      LED_dir = 1;
    }
  }
  else {
    LED_dir = 0;
  }

  // Calculate player position
  if (LED_dir < 0 && LED_pos > 0
      || LED_dir > 0 && LED_pos < NUM_LEDS)
  {
    LED_pos += LED_dir;
  }

  // Reset LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(0, 0, 0);
  }

  /* Monsters */
  spawnSpider();
  wander();

  Serial.print("Spawned: ");
  Serial.println(numSpawned);

  // Update mob positions
  for (int i = 0; i < NUM_LEDS; i++) {
    if (spider_pool[i] != -1)
    {
      if (spider_pool[i] == LED_pos) {
        killSpider(i);
        leds[spider_pool[i]] = LED_color;
      }
      else {
        switch (mob_type_at_location[spider_pool[i]]) {
          // spider
          case 1:
            leds[spider_pool[i]] = CRGB(255, 0, 0);
            break;
          // zombie
          case 2:
            leds[spider_pool[i]] = CRGB(0, 255, 255);
            break;
          // creeper
          case 3:
            leds[spider_pool[i]] = CRGB(0, 255, 0);
            break;
          // skeleton
          case 4:
            leds[spider_pool[i]] = CRGB(255, 255, 255);
            break;
          // enderman
          case 5:
            leds[spider_pool[i]] = CRGB(255, 0, 255);
            break;
          default:
            break;
        }
      }
    }
  }

  // Update player position
  leds[LED_pos] = LED_color;

  // Render LED strip
  FastLED.show();

  // Update player XP
  updateXP();
  
  // Listen for HTTP Post/Get Requests
  runServer();
}
