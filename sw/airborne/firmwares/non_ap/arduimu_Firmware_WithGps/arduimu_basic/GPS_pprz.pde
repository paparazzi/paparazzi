/****************************************************************
 * Parse GPS data from a Paparazzi autopilot
 ****************************************************************/
 // Code from Jordi, modified by Jose, modified by Gautier


void init_gps(void)
{
  //Serial.begin(38400); 
  pinMode(2,OUTPUT); //Serial Mux
  digitalWrite(2,HIGH); //Serial Mux
} 

/****************************************************************
 * 
 ****************************************************************/
void parse_pprz_gps() {

#if PERFORMANCE_REPORTING == 1
  gps_pos_fix_count++;
#endif

  speed_3d = (float)join_4_bytes(&Paparazzi_GPS_buffer[0])/100.0;    // m/s  0,1,2,3
  ground_speed = (float)join_4_bytes(&Paparazzi_GPS_buffer[4])/100.0; // Ground speed 2D  4,5,6,7
  ground_course = (float)join_4_bytes(&Paparazzi_GPS_buffer[8])/100000.0; // Heading 2D  8,9,10,11
  stGpsFix = Paparazzi_GPS_buffer[12];
  stFlags = Paparazzi_GPS_buffer[13];
  high_accel_flag = Paparazzi_GPS_buffer[14];

  if((stGpsFix >= 0x03) && (stFlags&0x01)) {
    gpsFix = 0; //valid position
    digitalWrite(6,HIGH);  //Turn LED when gps is fixed. 
    GPS_timer = DIYmillis(); //Restarting timer...
  }
  else {
    gpsFix = 1; //invalid position
    digitalWrite(6,LOW);
  }

  if (ground_speed > SPEEDFILT && gpsFix==0) gc_offset = ground_course - ToDeg(yaw);

}


/****************************************************************
 * Join 4 bytes into a long
 ****************************************************************/
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
void checksum(byte ubx_data)
{
  ck_a+=ubx_data;
  ck_b+=ck_a; 
}

 ****************************************************************/
