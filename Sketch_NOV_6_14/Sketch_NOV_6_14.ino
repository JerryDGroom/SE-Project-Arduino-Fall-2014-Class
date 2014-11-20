//This program combines SunPosition_Automated_9_13_13 w/ Calculate_Serial_Write_Values
//It should take YMDHMS & determine 0-4095 motor dope for one of two linear actuators
//The actuators will move the panel into the most optimal position within its reach to 
//recieve the most yield from the sun.

//sunpos Variables, also used to manually input date and time for testing.
int yearSP = 2013;
int monthSP = 11;
int daySP = 14;
double hoursSP = 7;
double minutesSP =00;
double secondsSP =00;

//solution variables
double dZenithAngle;
double dAzimuth;

//decimal degree Oskaloosa, IA
const double dLatitude = 41.28333;
const double dLongitude = -92.63333 ;

//define constants
const double pi = M_PI;
const double twopi (2*M_PI);
const double rad   (pi/180);
const double dEarthMeanRadius    = 6371.01;	// In km
const double dAstronomicalUnit   = 149597890;

//Vars for Linear actuators
const double EXTENSION_LENGTH_OPEN = 11.81; //inches.  
const double ACT_CLOSED_LENGTH = 17.72; //inches.  Closed actuator
const int RESOLUTION = 4095; //8-bit Analog write using Serial

//Vars for Actuator A
const double mastLength = 21.5; //inches
const double boomLength = 15.7; //inches

//Vars for Actuator B
const double mastLengthB = 16.25; //inches
const double boomLengthB = 30.875; //inches

//Software for JRK21V3 controls
#include <SoftwareSerial.h>
SoftwareSerial mySerial1(19,18); //Controls Actuator B
SoftwareSerial mySerial2(17,16); //Controls Actuator A
int myTarget = 0; // target position, 0-4095 is the range of the JRK21V3 control

//Software for RTC (Real Time Clock)
#include <Wire.h>
int clockAddress = 0x68;  // This is the I2C address
int command = 0;  // This is the command char, in ascii form, sent from the serial port     
long previousMillis = 0;  // will store last time Temp was updated
byte seconds, minutes, hours, dayOfWeek, dayOfMonth, month, year;
byte test; 

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}
// RTClib-master must be an available library when you select =>Sketch=>Import Library...Then at the bottom you have to see this library available for the RTC.* commands to work.
// you can download this library from: 
// https://github.com/adafruit/RTClib/archive/master.zip
#include "RTClib.h"
RTC_DS1307 RTC;
//SETUP
void setup()
{
  Serial.begin(9600);
  mySerial1.begin(9600);
  mySerial2.begin(9600);
  Wire.begin();
  Serial.begin(9600);
  RTC.begin();
  
  int myTarget = 4095; //the health level at any point in time
   
  if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
}

void loop()
{
 getDateDs1307();//Gets Current Date and Time

 //Date and Time variables from RTC replace sun position date and time variables for real-time data
 int yearSP = year;
 int monthSP = month;
 int daySP = dayOfMonth;
 double hoursSP = hours;
 double minutesSP = minutes;
 double secondsSP = seconds;

  sunpos(yearSP, monthSP, daySP, hoursSP, minutesSP, secondsSP);//Calculates current sun position
  
  Serial.print("\t");
  Serial.print("Altitude:  ");
  Serial.print(90 - dZenithAngle);
  Serial.print("\t");
  Serial.print("Azimuth:  ");
  Serial.print(dAzimuth);
  Serial.print("\t");
  delay(50);

  findDope(dAzimuth - 90); //Moves Actuator A into proper position
  findDope2(90- dZenithAngle); //Moves Actuator B into proper position
}

//Calculates necessary variables for the given Date and Time
void sunpos(int iYear, int iMonth, int iDay, double dHours, double dMinutes, double dSeconds)
//void sunpos()
{
  // Main variables
  double dElapsedJulianDays;
  double dDecimalHours;
  double dEclipticLongitude;
  double dEclipticObliquity;
  double dRightAscension;
  double dDeclination;

  // Auxiliary variables
  double dY;
  double dX;

  // Calculate difference in days between the current Julian Day 
  // and JD 2451545.0, which is noon 1 January 2000 Universal Time
  double dJulianDate;
  long int liAux1;
  long int liAux2;
  
  // Calculate time of the day in UT decimal hours
  dDecimalHours = dHours + (dMinutes + dSeconds / 60.0 ) / 60.0;
  
  // Calculate current Julian Day
  liAux1 =( iMonth-14)/12;
  liAux2=(1461*( iYear + 4800 + liAux1))/4 + (367*( iMonth - 2-12*liAux1))/12- (3*(( iYear + 4900 + liAux1)/100))/4+ iDay-32075;
  dJulianDate=(double)(liAux2)-0.5+dDecimalHours/24.0;
  
  // Calculate difference between current Julian Day and JD 2451545.0 
  dElapsedJulianDays = dJulianDate-2451545.0;

  // Calculate ecliptic coordinates (ecliptic longitude and obliquity of the 
  // ecliptic in radians but without limiting the angle to be less than 2*Pi 
  // (i.e., the result may be greater than 2*Pi)
  double dMeanLongitude;
  double dMeanAnomaly;
  double dOmega;
  dOmega=2.1429-0.0010394594*dElapsedJulianDays;
  dMeanLongitude = 4.8950630+ 0.017202791698*dElapsedJulianDays; // Radians
  dMeanAnomaly = 6.2400600+ 0.0172019699*dElapsedJulianDays;
  dEclipticLongitude = dMeanLongitude + 0.03341607*sin( dMeanAnomaly ) + 0.00034894*sin( 2*dMeanAnomaly )-0.0001134 -0.0000203*sin(dOmega);
  dEclipticObliquity = 0.4090928 - 6.2140e-9*dElapsedJulianDays +0.0000396*cos(dOmega);

  // Calculate celestial coordinates ( right ascension and declination ) in radians 
  // but without limiting the angle to be less than 2*Pi (i.e., the result may be 
  // greater than 2*Pi)
  double dSin_EclipticLongitude;
  dSin_EclipticLongitude= sin( dEclipticLongitude );
  dY = cos( dEclipticObliquity ) * dSin_EclipticLongitude;
  dX = cos( dEclipticLongitude );
  dRightAscension = atan2( dY,dX );
  if( dRightAscension < 0.0 ) 
  {
    dRightAscension = dRightAscension + twopi;
  }
  dDeclination = asin( sin( dEclipticObliquity )*dSin_EclipticLongitude );

  // Calculate local coordinates ( azimuth and zenith angle ) in degrees
  double dGreenwichMeanSiderealTime;
  double dLocalMeanSiderealTime;
  double dLatitudeInRadians;
  double dHourAngle;
  double dCos_Latitude;
  double dSin_Latitude;
  double dCos_HourAngle;
  double dParallax;
  dGreenwichMeanSiderealTime = 6.6974243242 +  0.0657098283*dElapsedJulianDays + dDecimalHours;
  dLocalMeanSiderealTime = (dGreenwichMeanSiderealTime*15 + dLongitude)*rad;
  dHourAngle = dLocalMeanSiderealTime - dRightAscension;
  dLatitudeInRadians = dLatitude*rad;
  dCos_Latitude = cos( dLatitudeInRadians );
  dSin_Latitude = sin( dLatitudeInRadians );
  dCos_HourAngle= cos( dHourAngle );
  dZenithAngle = (acos( dCos_Latitude*dCos_HourAngle *cos(dDeclination) + sin( dDeclination )*dSin_Latitude));
  dY = -sin( dHourAngle );
  dX = tan( dDeclination )*dCos_Latitude - dSin_Latitude*dCos_HourAngle;
  dAzimuth = atan2( dY, dX );
  if ( dAzimuth < 0.0 ) 
  { 
    dAzimuth =   dAzimuth + twopi;
  }
  dAzimuth =   dAzimuth/rad;
  
  // Parallax Correction
  dParallax=(dEarthMeanRadius/dAstronomicalUnit) *sin(  dZenithAngle);
  dZenithAngle=(  dZenithAngle + dParallax)/rad;
}

//Uses the variables found in sunpos to calculate needed measurements to move Actuator A
int findDope(double sunDegrees) 
{
  double targetLength = 0; //used for calculations
  double extensionLength = 0;//how much of the 11.81" actuator should be out
  double pwmInstruction =0;
  int writePWM = 0; //this value will be written to PWM pin

  //find what the extended length of the actuator should be dependant upon sun's angle.
  //convert extended length to 8bit representation of the required voltage to move the actuator to position
  targetLength = abs(sqrt(pow(mastLength,2)+pow(boomLength,2)-2*mastLength*boomLength*cos(sunDegrees*(M_PI/180))));
  extensionLength = (targetLength - ACT_CLOSED_LENGTH);
  
  //Writes Actuator A's extension length, target length.
   Serial.println();
   Serial.println();
   Serial.println("Actuator A:: ");
   Serial.println();
   Serial.print("Extension Length: ");
   Serial.print(extensionLength);
   Serial.println();
   Serial.print("Target Length: ");
   Serial.print(targetLength);
   
   
   
  //protect against out of bound requests
  if (extensionLength > EXTENSION_LENGTH_OPEN)
  { 
  // taken out for excel analysis 
  // Serial.print("ERR: the linear actuator is too short, the total desired length is: ");
  // Serial.println(targetLength);
    extensionLength = EXTENSION_LENGTH_OPEN;
    targetLength = ACT_CLOSED_LENGTH + EXTENSION_LENGTH_OPEN;
  } 
  else if (extensionLength < 0)
  {
    //taken out for excel analysis
    //Serial.print("ERR: the linear actuator is too long, the total desired length is:");
    //Serial.println(targetLength);
    extensionLength = 0;
    targetLength = ACT_CLOSED_LENGTH;
  }
  
  Serial.println();
  pwmInstruction = extensionLength/(EXTENSION_LENGTH_OPEN/RESOLUTION);
  writePWM = (int)round(pwmInstruction);

  //arbitrary
  delay(1000);
  
  //Writes actuator adjusments necessary
  Serial.print("writePWM: ");
  Serial.print(writePWM);
  Serial.println();
  Serial.print("PWM Instruction: ");
  Serial.print(extensionLength);
  Serial.println();
  Serial.print("Length of actuator: ");
  Serial.print(targetLength );
  Serial.println();
  Serial.print("total length of actuator");
  Serial.println();
  
  mySerial2.write(0xAA); //tells the controller we're starting to send it commands
  mySerial2.write(0xB);   //This is the pololu device # you're connected too that is found in the config utility(converted to hex). 
  mySerial2.write(0x40 + (writePWM & 0x1F)); //first half of the target, see the pololu jrk manual for more specifics
  mySerial2.write((writePWM >> 5) & 0x7F);   //second half of the target, " " "*/

  Serial.print("PWM: ");
  Serial.print(writePWM);
  Serial.println();
  return writePWM;
  
  //arbitrary
  delay(200);
}

//Uses the variables found in sunpos to calculate needed measurements to move Actuator B
int findDope2(double sunDegrees) 
{
  double targetLength = 0; //used for calculations
  double extensionLength = 0;//how much of the 11.81" actuator should be out
  double pwmInstruction =0;
  int writePWM = 0; //this value will be written to PWM pin

  //find what the extended length of the actuator should be dependant upon sun's angle.
  //convert extended length to 8bit representation of the required voltage to move the actuator to position
  targetLength = abs(sqrt(pow(mastLengthB,2)+pow(boomLengthB,2)-2*mastLengthB*boomLengthB*cos(sunDegrees*(M_PI/180))));
  extensionLength = (targetLength - ACT_CLOSED_LENGTH);
  
  //Writes Actuator B's extension length, target length.
  Serial.println();
  Serial.println("Actuator B:: ");
  Serial.println();
  Serial.print("Extension Length: ");
  Serial.print(extensionLength);
  Serial.println();
  Serial.print("Target Length: ");
  Serial.print(targetLength);
    
  //protect against out of bound requests
  if (extensionLength > EXTENSION_LENGTH_OPEN)
  { 
  // taken out for excel analysis 
  // Serial.print("ERR: the linear actuator is too short, the total desired length is: ");
  // Serial.println(targetLength);
    extensionLength = EXTENSION_LENGTH_OPEN;
    targetLength = ACT_CLOSED_LENGTH + EXTENSION_LENGTH_OPEN;
  } 
  else if (extensionLength < 0)
  {
  //taken out for excel analysis
  //Serial.print("ERR: the linear actuator is too long, the total desired length is:");
  //Serial.println(targetLength);
    extensionLength = 0;
    targetLength = ACT_CLOSED_LENGTH;
  }
 
  Serial.println();
  pwmInstruction = extensionLength/(EXTENSION_LENGTH_OPEN/RESOLUTION);
  writePWM = (int)round(pwmInstruction);
  
  //arbitrary
  delay(1000);
 
   //Writes actuator adjusments necessary
  Serial.print("writePWM: ");
  Serial.print(writePWM);
  Serial.println();
  Serial.print("PWM Instruction: ");
  Serial.print(extensionLength);
  Serial.println();
  Serial.print("Length of actuator: ");
  Serial.print(targetLength );
  Serial.println();
  Serial.print("total length of actuator");
  Serial.println();
  
  mySerial1.write(0xAA); //tells the controller we're starting to send it commands
  mySerial1.write(0xB);   //This is the pololu device # you're connected too that is found in the config utility(converted to hex). 
  mySerial1.write(0x40 + (writePWM & 0x1F)); //first half of the target, see the pololu jrk manual for more specifics
  mySerial1.write((writePWM >> 5) & 0x7F);   //second half of the target
  Serial.print("PWM: ");
  Serial.print(writePWM);
  Serial.println();
  Serial.println();
  Serial.println();
  return writePWM;
  
  
  //arbitrary
  delay(200);
  
  
}

//RTC code to get current Date and Time
void getDateDs1307()
{
  // Reset the register pointer
  Wire.beginTransmission(clockAddress);
  Wire.write(byte(0x00));
  Wire.endTransmission();
  Wire.requestFrom(clockAddress, 7);

  // A few of these need masks because certain bits are control bits
  seconds     = bcdToDec(Wire.read() & 0x7f);
  minutes     = bcdToDec(Wire.read());
  
  // Need to change this if 12 hour am/pm
  hours      = bcdToDec(Wire.read() & 0x3f);  
  dayOfWeek  = bcdToDec(Wire.read());
  dayOfMonth = bcdToDec(Wire.read());
  month      = bcdToDec(Wire.read());
  year       = bcdToDec(Wire.read());

  //if Statements to incoporate Daylight Savings
  //if (month > 10)
  //{
    //hours = hours + 5;
  //}
  //else if (month < 3)
  //{
    //hours = hours + 5;
  //}
  //else
  //{
    //hours = hours + 4;
  //}
  //if (hours > 23)//If we go past 24 we set it for the morning.
  //{
      //hours = 11;      
  //}

//Writes Current Date and Time  
  Serial.print(hours, DEC);
  Serial.print(":");
  Serial.print(minutes, DEC);
  Serial.print(":");
  Serial.print(seconds, DEC);
  Serial.print("  ");
  Serial.print(month, DEC);
  Serial.print("/");
  Serial.print(dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(year, DEC);
}



