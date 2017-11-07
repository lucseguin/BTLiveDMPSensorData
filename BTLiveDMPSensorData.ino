
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include <SPI.h>
#include <SD.h>

#include <HMC58X3.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <helper_3dmath.h>

////////////////////////////////////////////////////////////////////////////////
//// Parameters
////////////////////////////////////////////////////////////////////////////////
#define OFFSETS_FILE "offsets.txt"
//#define READINGS_FILE "data.txt"
//#define MAX_TIME_RECORDING 80.00
#define DEBUGGING 1
#define DMP_STABILIZATION_TIME 30.00
#define CUTOFF_DMP_LSB_VALUE 25.0 
#define EMA_ALPHA 0.9
#define SEND_LIVE_DATA_OVER_BT 1

#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13

MPU6050 mpu;
HMC58X3 mag;
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 gyro;       // [x, y, z]          gyro
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorInt16 linearAccel;    // [x, y, z]            
VectorFloat gravity;    // [x, y, z]            gravity vector
float mx, my, mz;
int16_t ax, ay, az, gx, gy, gz;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
int referenceHeadingDegrees = -1;
bool deviceConnected = false;

#define ble Serial3
bool only_displacement = true; 
bool post_dmp_stab_complete = false; 
#define CODE_OPTION1 1
//#define CODE_OPTION2 2
//#define CODE_OPTION3 3

float axFilt = 0.0;
float ayFilt = 0.0;
float dirFilt = 0.0;
float dirAlpha = 0.05;
float flagX = 0.0;
float flagY = 0.0;

float directionX = 0;
float directionY = 0;
float sumAccelX = 0.0;
float sumAccelY = 0.0;

File offset_file;
#ifdef READINGS_FILE
File readings_file;
#endif

int secondCnt = 0, minuteCnt = 0;
double z[1];
float sec10rst = 0.0;
float intervalTemps = 0.0;
int intervalCnt = 0;

volatile uint32_t t_now = millis();
volatile uint32_t t_last = 0;
uint32_t t_start = 0;
uint32_t t_print = 0;

float dt = 0.0;
float totalTime = 0.0;
uint32_t sampleCnt = 0;

VectorFloat current_acceleration;
VectorFloat previous_acceleration;
VectorFloat previous_velocity;
VectorFloat current_velocity;
VectorFloat previous_position;
VectorFloat current_position;

char convert_buffer[16];
////////////////////////////////////////////////////////////////////////////////
////////// Bluetooth LE
////////////////////////////////////////////////////////////////////////////////

#ifdef SEND_LIVE_DATA_OVER_BT
#define BLE_BUFFER_LENGTH 100
char BLE_received[BLE_BUFFER_LENGTH]; 
int BLE_received_cnt = 0;
char BLE_cmd_buffer[BLE_BUFFER_LENGTH];        // Buffer to store response
char BLE_buffer[BLE_BUFFER_LENGTH];        // Buffer to store response
int BLE_timeout = 800;

boolean BLECmd(long timeout, char* command, char* temp);
boolean BLEIsReady();
long BLEAutoBaud();
#endif 
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  display.begin(SSD1306_SWITCHCAPVCC);
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  delay(2000);
  display.clearDisplay();

  Serial.print("Initializing SD card...");
  if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  display.clearDisplay();
  display.setCursor(0, 9);
  display.println("Calibrating Mag.");
  display.display();
  Serial.println(F("Calibrating Mag."));
  // no delay needed as we have already a delay(5) in HMC5843::init()
  mag.init(false); // Dont set mode yet, we'll do that later on.
  // Calibrate HMC using self test, not recommended to change the gain after calibration.
  mag.calibrate(1, 128); // Use gain 1=default, valid 0-7, 7 not recommended.
  // Single mode conversion was used in calibration, now set continuous mode
  mag.setMode(0);
  

  display.clearDisplay();
  display.setCursor(0, 9);
  display.println("Init MPU6050");
  display.display();
  
  // initialize device
  Serial.println(F("Initializing MPU"));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing MPU DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  if (SD.exists(OFFSETS_FILE)) {
    Serial.println("OFFSETS_FILE found, loading offsets.");
    offset_file = SD.open(OFFSETS_FILE);
    String line = offset_file.readStringUntil('\n');
    sscanf (line.c_str(), "MPU6050-Offsets:%d,%d,%d,%d,%d,%d", &ax_offset, &ay_offset, &az_offset, &gx_offset, &gy_offset, &gz_offset);
    offset_file.close();

    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
  } else {
    // if the file didn't open, print an error:
    Serial.println("OFFSETS_FILE not found!");
    while (1) {}
  }

#ifdef READINGS_FILE
  //write readings to file for test in Matlab
  if (SD.exists(READINGS_FILE)) {
    SD.remove(READINGS_FILE);
  }
  readings_file = SD.open(READINGS_FILE, FILE_WRITE);
#endif

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  #ifdef SEND_LIVE_DATA_OVER_BT
  display.clearDisplay();
  display.setCursor(0, 9);
  display.println("BLE Auto Baud.");
  display.display();
  
  long baudrate = BLEAutoBaud();
  if (baudrate > 0) {
    Serial.print("Found BLE baud rate ");
    Serial.println(baudrate);
    if (baudrate != 115200) {
      BLECmd(BLE_timeout, "AT+BAUD4", BLE_buffer);
    }

  } else {
    Serial.println("No BLE detected.");
    while (1) {};         // No BLE found, just going to stop here
  }

  memset(BLE_received, 0, BLE_BUFFER_LENGTH);
  BLE_received_cnt = 0;
  BLECmd(BLE_timeout, "AT+NAMEUQOLuc", BLE_buffer); //set identifiable name

  BLECmd(BLE_timeout, "AT+NAME?", BLE_buffer);
  BLECmd(BLE_timeout, "AT+BAUD?", BLE_buffer);
  BLECmd(BLE_timeout, "AT+MODE?", BLE_buffer);
  BLECmd(BLE_timeout, "AT+PASS?", BLE_buffer);
  BLECmd(BLE_timeout, "AT+VERS?", BLE_buffer);
  BLECmd(BLE_timeout, "AT+RADD?", BLE_buffer);
  BLECmd(BLE_timeout, "AT+ADDR?", BLE_buffer);
  BLECmd(BLE_timeout, "AT+TYPE?", BLE_buffer);
  BLECmd(BLE_timeout, "AT+POWE?", BLE_buffer);
  BLECmd(BLE_timeout, "AT+NOTI1", BLE_buffer);
  Serial.println("----------------------");
  Serial.println("Waiting for remote connection...");
#endif

  previous_velocity.x = 0.0;
  previous_velocity.y = 0.0;
  previous_position.x = 0.0;
  previous_position.y = 0.0;

#ifdef CODE_OPTION1
  Serial.println(F("Using Code Option1"));
#elif CODE_OPTION2
  Serial.println(F("Using Code Option 2"));
#elif CODE_OPTION3
  Serial.println(F("Using Code Option 3"));  
#endif

  t_last = t_print = t_start = millis();
  Serial.print("Start time ");
  Serial.println(t_start);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

#ifdef SEND_LIVE_DATA_OVER_BT  
  if (ble.available()) {
    char c = (char)ble.read();
    BLE_received[BLE_received_cnt++] = c;
    BLE_received[BLE_received_cnt] = '\0';
    
    if(strcmp(BLE_received, "OK+CONN") == 0){
      BLE_received[0] = '\0';
      BLE_received_cnt = 0;
      
      //device just connected, send info
      deviceConnected = true;
      Serial.println(F("BLE Device Connected!"));

      if(post_dmp_stab_complete) {
          Serial.println(F("Sending Device Connected Init String!"));
          sprintf(BLE_cmd_buffer, "init:%d;", (int)referenceHeadingDegrees);
          BLECmd(BLE_timeout, BLE_cmd_buffer, BLE_buffer);
      }
    } else if (strcmp(BLE_received, "OK+LOST") == 0 ){
      BLE_received[0] = '\0';
      BLE_received_cnt = 0;
      
      deviceConnected = false;
      Serial.println(F("BLE Device Disconnected!"));
    }
    //Serial.print(c);
  }
#endif

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    ////////////////////////////////////////////////////////////////////////////////
    // calcul dt & total run time in seconds
    ////////////////////////////////////////////////////////////////////////////////
    t_now = millis();
    dt = (t_now - t_last) / 1000.0f;
    totalTime += dt;

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    ////////////////////////////////////////////////////////////////////////////////
    //get module direction from magnetometer
    ////////////////////////////////////////////////////////////////////////////////
    mag.getValues(&mx, &my, &mz);
    float heading = atan2(my, mx);
    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    float declinationAngle = 0.22;
    heading += declinationAngle;
    // Correct for when signs are reversed.
    if (heading < 0)
      heading += 2 * PI;
    // Check for wrap due to addition of declination.
    if (heading > 2 * PI)
      heading -= 2 * PI;
    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180 / M_PI;

    //Exponential Moving Average
    dirFilt = (1 - dirAlpha)*dirFilt + dirAlpha*headingDegrees;
    headingDegrees = dirFilt;

    ////////////////////////////////////////////////////////////////////////////////
    // get linear acceleration, adjusted to remove gravity
    ////////////////////////////////////////////////////////////////////////////////
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGyro(&gyro, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    linearAccel = aaReal;
    
    //if value is in interval [-CUTOFF_DMP_LSB_VALUE, CUTOFF_DMP_LSB_VALUE] set to zero
    if (linearAccel.x >= -CUTOFF_DMP_LSB_VALUE && linearAccel.x <= CUTOFF_DMP_LSB_VALUE) {
      linearAccel.x = 0;
    }
    if (linearAccel.y >= -CUTOFF_DMP_LSB_VALUE && linearAccel.y <= CUTOFF_DMP_LSB_VALUE) {
      linearAccel.y = 0;
    }
//
//#ifdef DEBUGGING   
//    //print readings to serial console for debugging
//   Serial.print("dt:"); Serial.println(dt, 4);;
//    Serial.print("accel:"); Serial.print(aa.x); Serial.print(","); Serial.print(aa.y); Serial.print(","); Serial.println(aa.z);
//    Serial.print("laccel:"); Serial.print(aaReal.x); Serial.print(","); Serial.print(aaReal.y); Serial.print(","); Serial.println(aaReal.z);
//    Serial.print("laccelw:"); Serial.print(aaWorld.x); Serial.print(","); Serial.print(aaWorld.y); Serial.print(","); Serial.println(aaWorld.z);
//    Serial.print("g:"); Serial.print(gravity.x); Serial.print(","); Serial.print(gravity.y); Serial.print(","); Serial.println(gravity.z);
//    Serial.print("gyro:"); Serial.print(gyro.x); Serial.print(","); Serial.print(gyro.y); Serial.print(","); Serial.println(gyro.z);
//    Serial.print("mag:"); Serial.print(mx); Serial.print(","); Serial.print(my); Serial.print(","); Serial.println(mz);
//    Serial.print("q:"); Serial.print(q.w); Serial.print(","); Serial.print(q.x); Serial.print(","); Serial.print(q.y);Serial.print(","); Serial.println(q.z);
//    Serial.print("heading:");  Serial.print(headingDegrees_raw); Serial.print(",");  Serial.println(headingDegrees);
//    Serial.print("TotalTime:"); Serial.println(totalTime, 4);
//#endif

    if(totalTime < DMP_STABILIZATION_TIME) {
          display.clearDisplay();
          display.setCursor(0, 9);
          display.println("DMP Stabilization");
          display.display();

          #ifdef READINGS_FILE
              //write readings for 
              readings_file.print("dt:"); readings_file.println(dt, 4);;
              readings_file.print("accel:"); readings_file.print(aa.x); readings_file.print(","); readings_file.print(aa.y); readings_file.print(","); readings_file.println(aa.z);
              readings_file.print("laccel:"); readings_file.print(aaReal.x); readings_file.print(","); readings_file.print(aaReal.y); readings_file.print(","); readings_file.println(aaReal.z);
              readings_file.print("laccelw:"); readings_file.print(aaWorld.x); readings_file.print(","); readings_file.print(aaWorld.y); readings_file.print(","); readings_file.println(aaWorld.z);
              readings_file.print("g:"); readings_file.print(gravity.x); readings_file.print(","); readings_file.print(gravity.y); readings_file.print(","); readings_file.println(gravity.z);
              readings_file.print("gyro:"); readings_file.print(gyro.x); readings_file.print(","); readings_file.print(gyro.y); readings_file.print(","); readings_file.println(gyro.z);
              readings_file.print("mag:"); readings_file.print(mx); readings_file.print(","); readings_file.print(my); readings_file.print(","); readings_file.println(mz);
              readings_file.print("q:"); readings_file.print(q.w); readings_file.print(","); readings_file.print(q.x); readings_file.print(","); readings_file.print(q.y);readings_file.print(","); readings_file.println(q.z);
              readings_file.print("heading:"); readings_file.println(headingDegrees); 
              readings_file.println("speed:0.0000,0.0000");
              readings_file.println("position:0.0000,0.0000"); 
              readings_file.print("TotalTime:"); readings_file.println(totalTime, 4);
          #endif
       
    } else {
      if (post_dmp_stab_complete == false) {
        post_dmp_stab_complete = true;
        Serial.println(F("DMP Stabilized!"));
                
        referenceHeadingDegrees = (int)headingDegrees;
        if(deviceConnected) {
          Serial.println(F("Sending Post Stabilization Init String!"));
          #ifdef SEND_LIVE_DATA_OVER_BT
          sprintf(BLE_cmd_buffer, "init:%d;", referenceHeadingDegrees);
          BLECmd(BLE_timeout, BLE_cmd_buffer, BLE_buffer);
          #endif
        }

      }

      //convert from LSB/g to m/s2
      current_acceleration.x = linearAccel.x / 8192.0 * 9.81;
      current_acceleration.y = linearAccel.y / 8192.0 * 9.81;
      
      //Exponential Moving Average
      axFilt = (1 - EMA_ALPHA)*axFilt + EMA_ALPHA*current_acceleration.x;
      ayFilt = (1 - EMA_ALPHA)*ayFilt + EMA_ALPHA*current_acceleration.y;
      
      current_acceleration.x = axFilt;
      current_acceleration.y  = ayFilt;

      if (current_acceleration.x < 0.0001 && current_acceleration.x > -0.0001) {
        current_acceleration.x = 0.0;
        current_velocity.x = 0.0;
      } else {
        current_velocity.x = previous_velocity.x + dt * current_acceleration.x;
      }

      if(only_displacement)
        current_position.x = dt * current_velocity.x;
      else
        current_position.x = previous_position.x+ dt * current_velocity.x;

        
      if (current_acceleration.y < 0.0001 && current_acceleration.y > -0.0001) {
        current_acceleration.y = 0.0;
        current_velocity.y = 0.0;
      } else {
        current_velocity.y = previous_velocity.y + dt * current_acceleration.y;
      }

      if(only_displacement)
        current_position.y = dt * current_velocity.y;
      else
        current_position.y = previous_position.y + dt * current_velocity.y;

      #ifdef READINGS_FILE
          //write readings for 
          readings_file.print("dt:"); readings_file.println(dt, 4);;
          readings_file.print("accel:"); readings_file.print(aa.x); readings_file.print(","); readings_file.print(aa.y); readings_file.print(","); readings_file.println(aa.z);
          readings_file.print("laccel:"); readings_file.print(aaReal.x); readings_file.print(","); readings_file.print(aaReal.y); readings_file.print(","); readings_file.println(aaReal.z);
          readings_file.print("laccelw:"); readings_file.print(aaWorld.x); readings_file.print(","); readings_file.print(aaWorld.y); readings_file.print(","); readings_file.println(aaWorld.z);
          readings_file.print("g:"); readings_file.print(gravity.x); readings_file.print(","); readings_file.print(gravity.y); readings_file.print(","); readings_file.println(gravity.z);
          readings_file.print("gyro:"); readings_file.print(gyro.x); readings_file.print(","); readings_file.print(gyro.y); readings_file.print(","); readings_file.println(gyro.z);
          readings_file.print("mag:"); readings_file.print(mx); readings_file.print(","); readings_file.print(my); readings_file.print(","); readings_file.println(mz);
          readings_file.print("q:"); readings_file.print(q.w); readings_file.print(","); readings_file.print(q.x); readings_file.print(","); readings_file.print(q.y);readings_file.print(","); readings_file.println(q.z);
          readings_file.print("heading:"); readings_file.println(headingDegrees); 
          readings_file.print("speed:"); readings_file.print(current_velocity.x); readings_file.print(","); readings_file.println(current_velocity.y); 
          readings_file.print("position:"); readings_file.print(current_position.x); readings_file.print(","); readings_file.println(current_position.y); 
          readings_file.print("TotalTime:"); readings_file.println(totalTime, 4);
      #endif

     if(deviceConnected) {
        #ifdef SEND_LIVE_DATA_OVER_BT
        sprintf(BLE_cmd_buffer, "upd:%d,", (int)headingDegrees);
        strcat(BLE_cmd_buffer, dtostrf(current_acceleration.x, 7, 4, convert_buffer));
        strcat(BLE_cmd_buffer, ",");
        strcat(BLE_cmd_buffer, dtostrf(current_acceleration.y, 7, 4, convert_buffer));
        strcat(BLE_cmd_buffer, ",");
        strcat(BLE_cmd_buffer, dtostrf(current_velocity.x, 7, 4, convert_buffer));
        strcat(BLE_cmd_buffer, ",");
        strcat(BLE_cmd_buffer, dtostrf(current_velocity.y, 7, 4, convert_buffer));
        strcat(BLE_cmd_buffer, ",");
        strcat(BLE_cmd_buffer, dtostrf(current_position.x, 9, 6, convert_buffer));
        strcat(BLE_cmd_buffer, ",");
        strcat(BLE_cmd_buffer, dtostrf(current_position.y, 9, 6, convert_buffer));
        strcat(BLE_cmd_buffer, ";");
        BLECmd(0, BLE_cmd_buffer, BLE_buffer);
        #endif
      }
      
      display.clearDisplay();
      display.setCursor(0, 9);
      display.print("vX:");display.print(current_velocity.x);
      display.setCursor(0, 18);
      display.print("vY:");display.println(current_velocity.y); 
      display.display();

      previous_velocity = current_velocity;
      previous_position = current_position;
      previous_acceleration = current_acceleration;
    }
    
    #ifdef READINGS_FILE
      #ifdef MAX_TIME_RECORDING
        if ( totalTime >= MAX_TIME_RECORDING) {
          readings_file.close();
          Serial.println("Done!");
          display.clearDisplay();
          display.setCursor(0, 9);
          display.println("Done!");
          display.display();
          while (1) {}
        }
      #endif
    #endif    
    
    t_last = t_now;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Bluetooth functions
////////////////////////////////////////////////////////////////////////////////
#ifdef SEND_LIVE_DATA_OVER_BT
boolean BLEIsReady() {
  BLECmd(BLE_timeout, "AT" , BLE_buffer);   // Send AT and store response to buffer
  if (strcmp(BLE_buffer, "OK") == 0) {
    return true;
  } else {
    return false;
  }
}
boolean BLECmd(long timeout, char* command, char* temp) {
  long endtime;
  boolean found = false;
  endtime = millis() + timeout; //
  memset(temp, 0, BLE_BUFFER_LENGTH);       // clear buffer
  found = true;
  
// #ifdef DEBUGGING
//  Serial.print("BLE send = ");
//  Serial.println(command);
// #endif 
 
  ble.print(command);

  // The loop below wait till either a response is received or timeout
  // The problem with this BLE Shield is the HM-10 module does not response with CR LF at the end of the response,
  // so a timeout is required to detect end of response and also prevent the loop locking up.
    if(timeout > 0) {
      while (!ble.available()) {
        if (millis() > endtime) {   // timeout, break
          found = false;
          break;
        }
      }
    
      if (found) {            // response is available
        int i = 0;
        while (ble.available()) {   // loop and read the data
          char a = ble.read();
     
           #ifdef DEBUGGING     
          Serial.print((char)a); // Uncomment this to see raw data from BLE
          #endif
     
          temp[i] = a;        // save data to buffer
          i++;
          if (i >= BLE_BUFFER_LENGTH) break; // prevent buffer overflow, need to break
          delay(2);           // give it a 2ms delay before reading next character
        }
     
         #ifdef DEBUGGING    
        Serial.print("BLE reply    = ");
        Serial.println(temp);
        #endif  
      }
    return true;
  } else {
 
 #ifdef DEBUGGING    
    //Serial.println("BLE timeout!");
 #endif   

    return false;
  }
}
long bauds[] = {115200, 9600, 57600, 38400, 2400, 4800, 19200};
long BLEAutoBaud() {
  int baudcount = sizeof(bauds) / sizeof(long);
  for (int i = 0; i < baudcount; i++) {
    for (int x = 0; x < 3; x++) { // test at least 3 times for each baud
      Serial.print("Testing baud ");
      Serial.println(bauds[i]);
      ble.begin(bauds[i]);
      if (BLEIsReady()) {
        return bauds[i];
      }
    }
  }
  return -1;
}
#endif


