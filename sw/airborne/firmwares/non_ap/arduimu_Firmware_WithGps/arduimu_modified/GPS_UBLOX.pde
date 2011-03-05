#if GPS_PROTOCOL == 3

/****************************************************************
 * Here you have all the parsing stuff for uBlox
 ****************************************************************/
 // Code from Jordi, modified by Jose
 
 //You have to disable all the other string, only leave this ones:
 
 //NAV-POSLLH Geodetic Position Solution, PAGE 66 of datasheet
 //NAV-VELNED Velocity Solution in NED, PAGE 71 of datasheet
 //NAV-STATUS Receiver Navigation Status, PAGE 67 of datasheet
 //NAV-SOL Navigation Solution Information, PAGE 72 of datasheet
 
 
 // Baud Rate:38400

/* 
 GPSfix Type 
 - 0x00 = no fix
 - 0x01 = dead reckonin
 - 0x02 = 2D-fix
 - 0x03 = 3D-fix
 - 0x04 = GPS + dead re
 - 0x05 = Time only fix
 - 0x06..0xff = reserved*/
 
 //Luckly uBlox has internal EEPROM so all the settings you change will remain forever.  Not like the SIRF modules!
 
void init_gps(void)
{
  //Serial.begin(38400); 
  pinMode(2,OUTPUT); //Serial Mux
  digitalWrite(2,HIGH); //Serial Mux
} 

/****************************************************************
 * 
 ****************************************************************/

void decode_gps(void){ 

  if(DIYmillis() - GPS_timer > 2000){
    digitalWrite(6, LOW);  //If we don't receive any byte in two seconds turn off gps fix LED...
    debug_print("Yeah, your GPS is disconnected"); 
    gpsFix=1;
  } 
}

/****************************************************************
 * 
 ****************************************************************/
void parse_ubx_gps(){

#if PERFORMANCE_REPORTING == 1
  gps_pos_fix_count++;
#endif

  speed_3d = (float)join_4_bytes(&Paparazzi_GPS_buffer[0])/100.0;    // m/s  0,1,2,3
  ground_speed = (float)join_4_bytes(&Paparazzi_GPS_buffer[4])/100.0; // Ground speed 2D  4,5,6,7
  ground_course = (float)join_4_bytes(&Paparazzi_GPS_buffer[8])/100000.0; // Heading 2D  8,9,10,11
  alt = join_4_bytes(&Paparazzi_GPS_buffer[12]);                      // 12,13,14,15
  alt_MSL = join_4_bytes(&Paparazzi_GPS_buffer[16]);                  // 16,17,18,19
  stGpsFix = Paparazzi_GPS_buffer[20];
  stFlags = Paparazzi_GPS_buffer[21];

  if((stGpsFix >= 0x03)&&(stFlags&0x01)){
    gpsFix=0; //valid position
    digitalWrite(6,HIGH);  //Turn LED when gps is fixed. 
    GPS_timer=DIYmillis(); //Restarting timer...
  }
  else{
    gpsFix=1; //invalid position
    digitalWrite(6,LOW);
  }

  if (ground_speed > SPEEDFILT && gpsFix==0) gc_offset = ground_course - ToDeg(yaw);

#if 0           
  // Serial.print("Time von Arduino ;");
  // Serial.print(millis());
  Serial.print("alt ;");
  Serial.print(alt);
  Serial.print("; alt_MSL: ;");
  Serial.print(alt_MSL);
  Serial.print("; speed_3d ;");
  Serial.print(speed_3d);
  Serial.print("; ground_speed ;");
  Serial.print(ground_speed);
  Serial.print("; ground_course ;");
  Serial.print(ground_course);
#endif

}


/****************************************************************
 * 
 ****************************************************************/
 // Join 4 bytes into a long
int32_t join_4_bytes(byte Buffer[])
{
  longUnion.byte[0] = *Buffer;
  longUnion.byte[1] = *(Buffer+1);
  longUnion.byte[2] = *(Buffer+2);
  longUnion.byte[3] = *(Buffer+3);
  return(longUnion.dword);
}

/****************************************************************
 * 
 ****************************************************************/
void checksum(byte ubx_data)
{
  ck_a+=ubx_data;
  ck_b+=ck_a; 
}


#endif
