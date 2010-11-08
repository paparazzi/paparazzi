/* ******************************************************* */
/* I2C HMC5843 magnetometer                                */
/* ******************************************************* */

// Local magnetic declination
// I use this web : http://www.ngdc.noaa.gov/geomagmodels/Declination.jsp
#define MAGNETIC_DECLINATION -6.0    // not used now -> magnetic bearing

int CompassAddress = 0x1E;  //0x3C //0x3D;  //(0x42>>1);

void I2C_Init()
{
  Wire.begin();
}


void Compass_Init()
{
  Wire.beginTransmission(CompassAddress);
  Wire.send(0x02); 
  Wire.send(0x00);   // Set continouos mode (default to 10Hz)
  Wire.endTransmission(); //end transmission
}

void Read_Compass()
{
  int i = 0;
  byte buff[6];
 
  Wire.beginTransmission(CompassAddress); 
  Wire.send(0x03);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  //Wire.beginTransmission(CompassAddress); 
  Wire.requestFrom(CompassAddress, 6);    // request 6 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  
  if (i==6)  // All bytes received?
    {
    // MSB byte first, then LSB, X,Y,Z
    magnetom_x = SENSOR_SIGN[6]*((((int)buff[2]) << 8) | buff[3]);    // X axis (internal sensor y axis)
    magnetom_y = SENSOR_SIGN[7]*((((int)buff[0]) << 8) | buff[1]);    // Y axis (internal sensor x axis)
    magnetom_z = SENSOR_SIGN[8]*((((int)buff[4]) << 8) | buff[5]);    // Z axis
    }
  else
    Serial.println("!ERR: Mag data");
}


void Compass_Heading()
{
  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  // Tilt compensated Magnetic filed X:
  MAG_X = magnetom_x*cos_pitch+magnetom_y*sin_roll*sin_pitch+magnetom_z*cos_roll*sin_pitch;
  // Tilt compensated Magnetic filed Y:
  MAG_Y = magnetom_y*cos_roll-magnetom_z*sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-MAG_Y,MAG_X);
}
