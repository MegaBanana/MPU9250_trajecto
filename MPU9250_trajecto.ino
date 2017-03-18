#include <MPU9250.h>
#include <quaternionFilters.h>


/* MPU9250 Basic Example Code
  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
  Modified by Brent Wilkins July 19, 2016

  Demonstrate basic MPU-9250 functionality including parameterizing the register
  addresses, initializing the sensor, getting properly scaled accelerometer,
  gyroscope, and magnetometer data out. Added display functions to allow display
  to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
  Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
  and the Teensy 3.1.

  SDA and SCL should have external pull-up resistors (to 3.3V).
  10k resistors are on the EMSENSR-9250 breakout board.

  MPU9250 Breakout --------- Arduino
  VDD ---------------------- 3.3V
  VDDI --------------------- 3.3V
  SDA ----------------------- A4
  SCL ----------------------- A5
  GND ---------------------- GND
*/
#include <TinyGPS++.h>
#include <SoftwareSerial.h>




#define SerialDebug false  // Set to true to get formated Serial output for debugging or false to get raw data

// Pin definitions

// I2C
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins

//Tiny GPS
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

MPU9250 myIMU;
// The TinyGPS++ object
TinyGPSPlus gps;
float lati;
float longi;
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

double gpsTimer =0;

void setup()
{
    //////// GPS///////////
  ss.begin(GPSBaud);
  
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(115200);
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  
  // Calibrate gyro and accelerometers, load biases in bias registers
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

  myIMU.initMPU9250();

  // Get magnetometer calibration from AK8963 ROM
  myIMU.initAK8963(myIMU.factoryMagCalibration);

  //myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);

  //////// GPS///////////
  ss.begin(GPSBaud);
  longi = 0;
  lati  = 0;
  


} // void setup()

void loop()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();



    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    // Following data from manual calibration were extracted
    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();


  myIMU.magBias[0]  = 173.275;
  myIMU.magBias[1]  = 155.415;
  myIMU.magBias[2] = -886,805;
 

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] -
               myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] -
               myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.factoryMagCalibration[2] -
               myIMU.magBias[2];


    //************** manual calibration ****************
    //myIMU.mx *= myIMU.magScale[0];
    //myIMU.my *= myIMU.magScale[1];
    //myIMU.mz *= myIMU.magScale[2];


  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);

  MadgwickQuaternionUpdate(myIMU.ax,                  // void MadgwickQuaternionUpdate(float ax,
                           myIMU.ay,                  //                               float ay,
                           myIMU.az,                  //                               float az,
                           myIMU.gx * DEG_TO_RAD,     //                               float gx,
                           myIMU.gy * DEG_TO_RAD,     //                               float gy,
                           myIMU.gz * DEG_TO_RAD,     //                               float gz,
                           myIMU.my,                  //                               float mx,
                           myIMU.mx,                  //                               float my,
                           -myIMU.mz,                  //                               float mz,
                           myIMU.deltat               //                               float deltat
                          );                         //                               );

  // Serial print and/or display at 0.5 s rate independent of data rates
  myIMU.delt_t = millis() - myIMU.count;




////////// GPS
smartDelay(10);
if (gps.location.isValid())
{
  lati = gps.location.lat();
  longi = gps.location.lng();
}


//  if ( millis() - gpsTimer > 500)
//  {  
//
//    Serial.print(lati, 6);
//    Serial.print(F(","));
//    Serial.println(longi, 6);
//    gpsTimer = millis();
//  }
  
  // update LCD once per half-second independent of read rate
  if (myIMU.delt_t > 100)
  {
        

    // Define output variables from updated quaternion---these are Tait-Bryan
    // angles, commonly used in aircraft orientation. In this coordinate system,
    // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
    // x-axis and Earth magnetic North (or true North if corrected for local
    // declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
    // arise from the definition of the homogeneous rotation matrix constructed
    // from quaternions. Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order which for this configuration is yaw,
    // pitch, and then roll.
    // For more see
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.
    myIMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                                *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1) * *(getQ() + 1)
                        - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
    // 2*q1*q3 + q0*q4, q0*q2 - q4*q4
    myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                                *(getQ() + 2)));
    myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) *
                                *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1) * *(getQ() + 1)
                        - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));

    // 0° 41' E  ± 0° 35' (or 8.5°) on 2017-01-05
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    myIMU.yaw   -= 0.68; // 48°53'37.3"N 2°11'38.9"E

    // ***** Data transmitted according to the following pattern ****
    //      [ax]\t\t[ay]\t\t[az]\t\t[pitch]\t\t[yaw]\t\t[roll]\n
    // **************************************************************
    Serial.write((const uint8_t *)&myIMU.ax, sizeof(float));
    Serial.write("\t\t");
    Serial.write((const uint8_t *)&myIMU.ay, sizeof(float));
    Serial.print("\t\t");
    Serial.write((const uint8_t *)&myIMU.az, sizeof(float));
    Serial.print("\t\t");

    Serial.write((const uint8_t *)&myIMU.pitch, sizeof(float));
    Serial.print("\t\t");
    Serial.write((const uint8_t *)&myIMU.yaw, sizeof(float));
    Serial.print("\t\t");
    Serial.write((const uint8_t *)&myIMU.roll, sizeof(float));
    Serial.print("\t\t");

    // GPS
    Serial.write((byte*)&lati,4);
    Serial.print("\t\t");
    Serial.write((byte*)&longi,4);

    Serial.println();
    

    myIMU.count = millis();
    myIMU.sumCount = 0;
    myIMU.sum = 0;
  } // if (myIMU.delt_t > 100)
} // void loop()


// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}



