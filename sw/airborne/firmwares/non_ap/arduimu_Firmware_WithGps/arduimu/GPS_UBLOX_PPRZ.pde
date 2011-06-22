#if GPS_PROTOCOL == 5

/*
      gps_mode = UBX_NAV_STATUS_GPSfix(ubx_msg_buf);
      gps_flags = UBX_NAV_STATUS_Flags(ubx_msg_buf);
      
      gps_mode = UBX_NAV_SOL_GPSfix(ubx_msg_buf)
      gps_flags = UBX_NAV_SOL_Flags(ubx_msg_buf);
      
      gps_hmsl POS_LLH?
      
      SCP1000_I2C_ADDR 0x22 ?
*/

/****************************************************************
 * Here you have all the parsing stuff for uBlox via I2C
 ****************************************************************/
 // Code from Jordi, modified by Jose
 
 //You have to disable all the other string, only leave this ones:
 
 //NAV-POSLLH Geodetic Position Solution, PAGE 66 of datasheet
 //NAV-VELNED Velocity Solution in NED, PAGE 71 of datasheet
 //NAV-STATUS Receiver Navigation Status, PAGE 67 of datasheet
 //NAV-SOL Navigation Solution Information, PAGE 72 of datasheet
 
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
} 

/****************************************************************
 * 
 ****************************************************************/

void decode_gps(void){ 
  
  if(DIYmillis() - GPS_timer > 2000){
    /* let the main loop set the blue LED */
    gpsFix=1;
  }
}

/****************************************************************
 * 
 ****************************************************************/
void parse_i2c_gps(){

  GPS_timer=DIYmillis();         /* Restarting timer... */

  #if PERFORMANCE_REPORTING == 1
    gps_pos_fix_count++;
  #endif

  ground_speed = (float)join_4_bytes(&Paparazzi_GPS_buffer[0])/100.;      // Ground speed 2D
  ground_course = (float)join_4_bytes(&Paparazzi_GPS_buffer[4])/100000.;  // Heading 2D
  ubGpsFix=Paparazzi_GPS_buffer[8];                                       // Status fix
  ubGpsFlags=Paparazzi_GPS_buffer[9];                                     // Status flags
  speed_3d = (float)join_4_bytes(&Paparazzi_GPS_buffer[10])/100.0;        // Speed 3D

  /* let the main loop set the blue LED */
  if((ubGpsFix >= 0x03) && (ubGpsFlags & 0x01)){
    gpsFix=0; //valid position
  }
  else{
    gpsFix=1; //invalid position
  }
  
  if (ground_speed > SPEEDFILT && gpsFix==0) gc_offset = ground_course - ToDeg(yaw);

#if PRINT_DEBUG != 0 
  /* for ground debug only, do not call Serial.print() from irq */
  #warning PRINT_DEBUG is for ground debug only, no correct timer values!
  Serial.print("ground_speed ;");
  Serial.print(ground_speed);
  Serial.print(";  ground_course ;");
  Serial.print(ground_course);
  Serial.print(";  fix ;");
  Serial.print(ubGpsFix);
  Serial.print(";  flags ;");
  Serial.print(ubGpsFlags);
  Serial.print(";  speed_3d ;");
  Serial.print(speed_3d);
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
 * GPS Data from Paparazzi u-blox
 ****************************************************************/
void receiveEvent(int howMany){

#if PRINT_DEBUG != 0 
  /* for ground debug only, do not call Serial.print() from irq */
  #warning PRINT_DEBUG is for ground debug only, no correct timer values!
  Serial.print(" How Many Bytes GPS : "); 
  Serial.println(howMany);
#endif

  for(int i=0; i < howMany; i++){
    Paparazzi_GPS_buffer[i]=Wire.receive();
  }

  parse_i2c_gps();               // Parse new GPS packet...
}

#endif /* GPS_PROTOCOL == 5 */
