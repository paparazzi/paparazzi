#if GPS_PROTOCOL == 1

//*********************************************************************************************
//  You may need to insert parameters appropriate for your gps from this list into init_gps()
//GPS SIRF configuration strings... 
#define SIRF_BAUD_RATE_4800    "$PSRF100,1,4800,8,1,0*0E\r\n"
#define SIRF_BAUD_RATE_9600    "$PSRF100,1,9600,8,1,0*0D\r\n"
#define SIRF_BAUD_RATE_19200    "$PSRF100,1,19200,8,1,0*38\r\n"
#define SIRF_BAUD_RATE_38400    "$PSRF100,1,38400,8,1,0*3D\r\n"  
#define SIRF_BAUD_RATE_57600    "$PSRF100,1,57600,8,1,0*36\r\n"
#define GSA_ON   "$PSRF103,2,0,1,1*27\r\n"   // enable GSA
#define GSA_OFF  "$PSRF103,2,0,0,1*26\r\n"   // disable GSA
#define GSV_ON   "$PSRF103,3,0,1,1*26\r\n"  // enable GSV
#define GSV_OFF  "$PSRF103,3,0,0,1*27\r\n"  // disable GSV
#define USE_WAAS   1     //1 = Enable, 0 = Disable, good in USA, slower FIX... 
#define WAAS_ON    "$PSRF151,1*3F\r\n"       // enable WAAS
#define WAAS_OFF   "$PSRF151,0*3E\r\n"       // disable WAAS

//GPS Locosys configuration strings...
#define USE_SBAS 0
#define SBAS_ON "$PMTK313,1*2E\r\n"
#define SBAS_OFF "$PMTK313,0*2F\r\n"

#define NMEA_OUTPUT_5HZ "$PMTK314,0,5,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 5HZ  
#define NMEA_OUTPUT_4HZ "$PMTK314,0,4,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 4HZ 
#define NMEA_OUTPUT_3HZ "$PMTK314,0,3,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 3HZ 
#define NMEA_OUTPUT_2HZ "$PMTK314,0,2,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 2HZ 
#define NMEA_OUTPUT_1HZ "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 1HZ

#define LOCOSYS_REFRESH_RATE_200 "$PMTK220,200*2C" //200 milliseconds 
#define LOCOSYS_REFRESH_RATE_250 "$PMTK220,250*29\r\n" //250 milliseconds

#define LOCOSYS_BAUD_RATE_4800 "$PMTK251,4800*14\r\n"
#define LOCOSYS_BAUD_RATE_9600 "$PMTK251,9600*17\r\n"
#define LOCOSYS_BAUD_RATE_19200 "$PMTK251,19200*22\r\n"
#define LOCOSYS_BAUD_RATE_38400 "$PMTK251,38400*27\r\n"
//**************************************************************************************************************


/****************************************************************
 Parsing stuff for NMEA
 ****************************************************************/
#define BUF_LEN 200

// GPS Pointers
char *token;
char *search = ",";
char *brkb, *pEnd;
char gps_buffer[BUF_LEN]; //The traditional buffer.

void init_gps(void)
{
	pinMode(2,OUTPUT); //Serial Mux
	digitalWrite(2,HIGH); //Serial Mux
	Serial.begin(9600);
	delay(1000);
	Serial.print(LOCOSYS_BAUD_RATE_38400);
	Serial.begin(38400);
	delay(500);
	Serial.print(LOCOSYS_REFRESH_RATE_250);
	delay(500);
	Serial.print(NMEA_OUTPUT_4HZ);
	delay(500);
	Serial.print(SBAS_OFF);


/* EM406 example init
	Serial.begin(4800); //Universal Sincronus Asyncronus Receiveing Transmiting 
	delay(1000);
	Serial.print(SIRF_BAUD_RATE_9600);
 
	Serial.begin(9600);
	delay(1000);
	
	Serial.print(GSV_OFF);
	Serial.print(GSA_OFF);
	
	#if USE_WAAS == 1
	Serial.print(WAAS_ON);
	#else
	Serial.print(WAAS_OFF);
	#endif
*/
	
}

void decode_gps(void)
{
	const char head_rmc[]="GPRMC"; //GPS NMEA header to look for
	const char head_gga[]="GPGGA"; //GPS NMEA header to look for
	
	static unsigned long GPS_timer = 0; //used to turn off the LED if no data is received. 
	
	static byte unlock = 1; //some kind of event flag
	static byte checksum = 0; //the checksum generated
	static byte checksum_received = 0; //Checksum received
	static byte counter = 0; //general counter

	//Temporary variables for some tasks, specially used in the GPS parsing part (Look at the NMEA_Parser tab)
	unsigned long temp = 0;
	unsigned long temp2 = 0;
	unsigned long temp3 = 0;


	while(Serial.available() > 0)
	{
		if(unlock == 0)
		{
			gps_buffer[0] = Serial.read();//puts a byte in the buffer

			if(gps_buffer[0]=='$')//Verify if is the preamble $
			{
				counter 	= 0;
				checksum 	= 0;
				unlock		= 1;
			}
		} else {
			gps_buffer[counter] = Serial.read();

			if(gps_buffer[counter] == 0x0A)//Looks for \F
			{
				unlock = 0;

				if (strncmp (gps_buffer, head_rmc, 5) == 0)//looking for rmc head....
				{

					/*Generating and parsing received checksum, */
					for(int x=0; x<100; x++)
					{
						if(gps_buffer[x]=='*')
						{ 
							checksum_received = strtol(&gps_buffer[x + 1], NULL, 16);//Parsing received checksum...
							break; 
						}
						else
						{
							checksum ^= gps_buffer[x]; //XOR the received data... 
						}
					}

					if(checksum_received == checksum)//Checking checksum
					{
						/* Token will point to the data between comma "'", returns the data in the order received */
						/*THE GPRMC order is: UTC, UTC status , Lat, N/S indicator, Lon, E/W indicator, speed, course, date, mode, checksum*/
						token = strtok_r(gps_buffer, search, &brkb); //Contains the header GPRMC, not used

						token = strtok_r(NULL, search, &brkb); //UTC Time, not used
						//time=	atol (token);
						token = strtok_r(NULL, search, &brkb); //Valid UTC data? maybe not used... 


						//Longitude in degrees, decimal minutes. (ej. 4750.1234 degrees decimal minutes = 47.835390 decimal degrees)
						//Where 47 are degrees and 50 the minutes and .1234 the decimals of the minutes.
						//To convert to decimal degrees, devide the minutes by 60 (including decimals), 
						//Example: "50.1234/60=.835390", then add the degrees, ex: "47+.835390 = 47.835390" decimal degrees
						token = strtok_r(NULL, search, &brkb); //Contains Latitude in degrees decimal minutes... 

						//taking only degrees, and minutes without decimals, 
						//strtol stop parsing till reach the decimal point "."	result example 4750, eliminates .1234
						temp = strtol (token, &pEnd, 10);

						//takes only the decimals of the minutes
						//result example 1234. 
						temp2 = strtol (pEnd + 1, NULL, 10);

						//joining degrees, minutes, and the decimals of minute, now without the point...
						//Before was 4750.1234, now the result example is 47501234...
						temp3 = (temp * 10000) + (temp2);


						//modulo to leave only the decimal minutes, eliminating only the degrees.. 
						//Before was 47501234, the result example is 501234.
						temp3 = temp3 % 1000000;


						//Dividing to obtain only the de degrees, before was 4750 
						//The result example is 47 (4750/100 = 47)
						temp /= 100;

						//Joining everything and converting to * 10,000,000 ... 
						//First i convert the decimal minutes to degrees decimals stored in "temp3", example: 501234/600000 =.835390
						//Then i add the degrees stored in "temp" and add the result from the first step, example 47+.835390 = 47.835390 
						//The result is stored in "lat" variable... 
						//**This is all changed in this case to be a long integer which is decimal degrees * 10**7

						lat	= (temp * 10000000) + ((temp3 *100) / 6);

						token = strtok_r(NULL, search, &brkb); //lat, north or south?
						//If the char is equal to S (south), multiply the result by -1.. 
						if(*token == 'S'){
							lat *= -1;
						}

						//This the same procedure use in lat, but now for Lon....
						token = strtok_r(NULL, search, &brkb);
						temp = strtol (token,&pEnd, 10); 
						temp2 = strtol (pEnd + 1, NULL, 10); 
						temp3 = (temp * 10000) + (temp2);
						temp3 = temp3%1000000; 
						temp/= 100;
						lon	= (temp * 10000000) + ((temp3 * 100) / 6);

						token = strtok_r(NULL, search, &brkb); //lon, east or west?
						if(*token == 'W'){
							lon *= -1;
						}

						token = strtok_r(NULL, search, &brkb); 	//Speed over ground
						speed_3d = (float)atoi(token);					// We only get ground speed but store it as speed_3d for use in DCM

						token = strtok_r(NULL, search, &brkb); //Course
						ground_course = (float)atoi(token);
						
						gpsFixnew=1;  			//new information available flag for binary message

					}
					checksum = 0;
				}//End of the GPRMC parsing

				if (strncmp (gps_buffer, head_gga, 5) == 0)//now looking for GPGGA head....
				{
					/*Generating and parsing received checksum, */
					for(int x = 0; x<100; x++)
					{
						if(gps_buffer[x]=='*')
						{ 
							checksum_received = strtol(&gps_buffer[x + 1], NULL, 16);//Parsing received checksum...
							break; 
						}
						else
						{
							checksum^= gps_buffer[x]; //XOR the received data... 
						}
					}

					if(checksum_received== checksum)//Checking checksum
					{
						//strcpy(gps_GGA,gps_buffer);

						token = strtok_r(gps_buffer, search, &brkb);//GPGGA header, not used anymore
						token = strtok_r(NULL, search, &brkb);//UTC, not used!!
						token = strtok_r(NULL, search, &brkb);//lat, not used!!
						token = strtok_r(NULL, search, &brkb);//north/south, nope...
						token = strtok_r(NULL, search, &brkb);//lon, not used!!
						token = strtok_r(NULL, search, &brkb);//wets/east, nope
						token = strtok_r(NULL, search, &brkb);//Position fix, used!!
						
						if(atoi(token) >= 1){
							gpsFix = 0x00;			// gpsFix = 0 means valid fix
						}else{
							gpsFix = 0x01;
						}
						token = strtok_r(NULL, search, &brkb); //sats in use!! Nein...
						token = strtok_r(NULL, search, &brkb);//HDOP, not needed
						token = strtok_r(NULL, search, &brkb);//ALTITUDE, is the only meaning of this string.. in meters of course. 
						alt_MSL = abs(atoi(token)) * 1000;
						
						if(gpsFix == 0x00) digitalWrite(6, HIGH); //Status LED...
						else digitalWrite(6, LOW);
					}
					checksum = 0; //Restarting the checksum
				}

				for(int a = 0; a<= counter; a++)//restarting the buffer
				{
					gps_buffer[a]= 0;
				} 
				counter = 0; //Restarting the counter
				GPS_timer = millis(); //Restarting timer...
			}
			else
			{
				counter++; //Incrementing counter
				if (counter >= 200)
				{
					//Serial.flush();
					counter = 0;
					checksum = 0;
					unlock = 0;
				}
			}
		}
	}
	
	if(millis() - GPS_timer > 2000){
		digitalWrite(6, LOW);	//If we don't receive any byte in two seconds turn off gps fix LED... 
		gpsFix = 0x01;
		gpsFixnew = 0;
	}
}
#endif

