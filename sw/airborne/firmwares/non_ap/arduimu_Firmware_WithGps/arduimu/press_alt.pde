/* VTI SCP1000-D11 I2C pressure sensor */
/* Based on code by Chris Barnes February 2010 */
/* Based on code by Jose Julio */

/* ATTENTION: SCP1000 is a 3v3 device */

#if USE_BAROMETER == 1
void setup_scp()
{
  unsigned char temp_byte = 0;
  delay(6);  
//  Init_Diagnostics();              // Initialise diagnostics output

  // If we know the starting altitude we could initialise it
  // else just treat this as the altitude relative to the start
  // init_alt(START_ALTITUDE);     // altitude in m     ****************
  

  Wire.begin();
  TWIinit();
  SCP1000_StopAcquisition();    // Stop acquisition first - in case the sensor had already been put into a non-standby mode (e.g. Arduino s/w reset but sensor not)
  delay(100);
  SCP1000_StartAcquisition();   // SCP1000 only accepts new mode from standby - otherwise device will be left in previous mode
/*  
  Serial.print("Status: ");
  temp_byte = SCP1000_GetStatus2();  
  Serial.println((int)temp_byte);
  DecodeStatus(temp_byte);
  
  Serial.print("Operation Mode: ");
  Serial.println((int)SCP1000_GetOperation2());
*/
  delay(100); 
}

/********************************************************************
	Altitude Calculation
*********************************************************************/

void alti(void)
{
  
	double x;
	double p = (double)press_gnd/(double)press;
	double temp = (float)temperature/20.f + 273.15f;
	x = log(p) * temp * 29271.267f;
	//x = log(p) * temp * 29.271267 * 1000;
	press_alt = x + ground_alt;
	//  Need to add comments for theory.....
}

/********************************************************************
	Data Retrieval
*********************************************************************/
 void SCP1000_StartAcquisition()
{
  Wire.beginTransmission((uint8_t)PRESSURE_ADDR);
  Wire.send((uint8_t)SNS_ADDR_POPERATION);  // Start acquisition
  Wire.send((uint8_t)SCP_MODE); 
  Wire.endTransmission();
}

void SCP1000_StopAcquisition()
{
  Wire.beginTransmission((uint8_t)PRESSURE_ADDR);
  Wire.send((uint8_t)SNS_ADDR_POPERATION);  
  Wire.send(0x00);   // Stop acquisition
  Wire.endTransmission();
}
   
unsigned char SCP1000_GetStatus2()
{
  unsigned char ready[] = {
    2, WRITE_PRESSURE_ADDR, SNS_ADDR_PSTATUS,
    2, READ_PRESSURE_ADDR, 0,
    0
  };  
  TWIdocmd(ready);
 
  return(ready[5]);
}

unsigned char SCP1000_GetOperation2()
{
  unsigned char oper[] = {
    2, WRITE_PRESSURE_ADDR, SNS_ADDR_POPERATION,
    2, READ_PRESSURE_ADDR, 0,
    0
  };  
  TWIdocmd(oper);
 
  return(oper[5]);
}

// Return the raw temperature value provided by the SCP1000
// This has units of degrees C x 20
int SCP1000_GetTemp2()
{
  int temp;
 
  unsigned char getdatat[] = {
    2, WRITE_PRESSURE_ADDR, SNS_ADDR_PTEMP,
    3, READ_PRESSURE_ADDR, 0, 0,
    0
  };

  TWIdocmd(getdatat);

  // check sign bit of temperature
  if (0x20U & getdatat[5])
  {
    // negative temperature - extend sign
    getdatat[5] |= 0xC0;
  }
  temp = getdatat[5] << 8;    // MSByte
  temp |= getdatat[6];        // LSByte

  return(temp);
}

// Return the raw pressure value provided by the SCP1000
unsigned long SCP1000_GetPressure2()
{
  unsigned long press;
  
  unsigned char getdatap1[] = {
    2, WRITE_PRESSURE_ADDR, SNS_ADDR_DATARD8,
    2, READ_PRESSURE_ADDR, 0, 
    0
  };
  unsigned char getdatap2[] = {
    2, WRITE_PRESSURE_ADDR, SNS_ADDR_PPRESSURE,
    3, READ_PRESSURE_ADDR, 0, 0,
    0
  };
  
  TWIdocmd(getdatap1);

  press = 0x07 & getdatap1[5];  // MSByte (either 5 or 6 for the sort of altitude we are interested in)
  
  TWIdocmd(getdatap2);
  
  press <<= 8;
  press |= getdatap2[5];
  press <<= 8; 
  press |= getdatap2[6];
 
  return(press);
}

// Decode bits from status byte and return TRUE is there is a measurement ready to be read
byte DecodeStatus(byte u8Status)
{
  static byte u8Starting = 0U;
  
  if (0U != (0x01 & u8Status))
  {
    //SCP1000 Startup procedure is still running
    if (u8Starting)
    { 
      Serial.println("Starting");
      u8Starting = 1;  // remember that we have already output this diagnostic
    }
  }
  else
  {
    u8Starting = 0;    // remember that we ahve seen the device out of starting mode
    if (0U != (0x10 & u8Status))
    {
      // Real Time Error - data was not read in time - clear by reading it
      // Note - this "error" is to be expected on startup when the SCP1000 is already running 
      // as we will not have been reading out all of the measurements that have been made...
      Serial.println("RTErr");						
    }
    if (0U != (0x20 & u8Status))
    {
      // DRDY - new data ready
      return (TRUE);
    }
  }
  return (FALSE);
}
  
void ReadSCP1000(void)
{
  temp_unfilt = SCP1000_GetTemp2();
  press = SCP1000_GetPressure2();

}



/********************************************************************
	This Library is needed for the SCP1000-D11
	because I2C commands needs a RESTART between commands and not and STOP-START (like Wire library do)
	This code is from http://harleyhacking.blogspot.com/
*********************************************************************/


#include <avr/io.h>

// TWI operations
#define ENABTW ((1<<TWINT)|(1<<TWEN)|(0<<TWIE)) // 0x80 4 1
#define START TWCR = (ENABTW|(1<<TWSTA))        // 0x20
#define STOP TWCR = (ENABTW|(1<<TWSTO)) // 0x10
#define SEND(x)  TWDR = x;  TWCR = ENABTW;
#define RECV(ack) TWCR = ENABTW | (ack? (1<<TWEA) : 0 );
unsigned char twista;
unsigned twitmo;
#define WAIT twitmo=0; while (!((twista = TWCR) & (1 << TWINT)) && ++twitmo);
 
/////===================================////////////////////
void TWIinit(void)
{
    DDRC &= ~0x30;              // pullup
    PORTC |= 0x30;              // pullup
    TWBR = (((F_CPU/400000)-16)/2); // 400 khz
    TWCR |= (1 << TWEN);
}
 
void TWIdocmd(unsigned char *msg)
{
    unsigned int mlen, rdwrf;
 
    while ((mlen = *msg++)) {
        rdwrf = *msg & 1;
        START;
        WAIT;
        do {
            SEND(*msg++);
            WAIT;
            // should check for ACK - twista == SAWA or SDWA
        } while (--mlen && !rdwrf);
        // read
        while (mlen--) {
            RECV(mlen);
            WAIT;
            *msg++ = TWDR;
        }
    }
    STOP;
}

#endif
