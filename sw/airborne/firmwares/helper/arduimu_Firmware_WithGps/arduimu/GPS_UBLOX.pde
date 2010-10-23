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

      messageNr = Paparazzi_GPS_buffer[0];
      
      if(messageNr == 0x00){
        //Nachricht 0                                                               Bytes:                                                         
              iTOW2 = join_4_bytes(&Paparazzi_GPS_buffer[1]);                      //1,2,3,4
              lon2 = join_4_bytes(&Paparazzi_GPS_buffer[5]);                       //5,6,7,8
              lat2 = join_4_bytes(&Paparazzi_GPS_buffer[9]);                       //9,10,11,12
              alt2 = join_4_bytes(&Paparazzi_GPS_buffer[13]);                      //13,14,15,16
              alt_MSL2 = join_4_bytes(&Paparazzi_GPS_buffer[17]);                  // 17,18,19,20
              speed_3d2 = (float)join_4_bytes(&Paparazzi_GPS_buffer[21])/100.0;    // m/s  21,22,23,24
              ground_speed2 = (float)join_4_bytes(&Paparazzi_GPS_buffer[25])/100.0; // Ground speed 2D    25,26,27,28 29,30 31
              recPakOne=0x01;
      }


      if(messageNr == 0x01 && recPakOne==0x01){
              // Nachricht 1   
              ground_course = (float)join_4_bytes(&Paparazzi_GPS_buffer[1])/100000.0; // Heading 2D  1,2,3,4
              ecefVZ=(float)join_4_bytes(&Paparazzi_GPS_buffer[5])/100; //Vertical Speed             5,6,7,8
              numSV=Paparazzi_GPS_buffer[9]; //Number of sats...                                     9
              stGpsFix=Paparazzi_GPS_buffer[10];
              stFlags=Paparazzi_GPS_buffer[11];
              solGpsFix=Paparazzi_GPS_buffer[12];
              solFlags=Paparazzi_GPS_buffer[13];
              
              iTOW = iTOW2;
              lon = lon2;
              lat = lat2;
              alt = alt2;
              alt_MSL = alt_MSL2;
              speed_3d = speed_3d2;
              ground_speed = ground_speed2;
              
              messageNr=messageNr+1; //2
      }
        
        
        if(messageNr == 0x02){
           if((stGpsFix >= 0x03)&&(stFlags&0x01)){
              gpsFix=0; //valid position
              digitalWrite(6,HIGH);  //Turn LED when gps is fixed. 
              GPS_timer=DIYmillis(); //Restarting timer...
            }
            else{
              gpsFix=1; //invalid position
              digitalWrite(6,LOW);
            }
          
           if((solGpsFix >= 0x03)&&(solFlags&0x01)){
              gpsFix=0; //valid position
              digitalWrite(6,HIGH);  //Turn LED when gps is fixed. 
              GPS_timer=DIYmillis(); //Restarting timer...
            }
            else{
              gpsFix=1; //invalid position
              digitalWrite(6,LOW);
            }
            
            if (ground_speed > SPEEDFILT && gpsFix==0) gc_offset = ground_course - ToDeg(yaw);
            recPakOne=0x00;
            //messageNr= 0x05;  // kommt so nicht mehr in die Abfage !!!   
#if 0           
            // Serial.print("Time von Arduino ;");
               // Serial.print(millis());
                Serial.print("MesageNr: ");
                Serial.print((int)(messageNr));
                Serial.print(";    itow ;");
                Serial.print(iTOW);
                Serial.print(";  lon ;");
                Serial.print(lon);
                Serial.print(";  lat ;");
                Serial.print(lat);
                Serial.print("; alt ;");
                Serial.print(alt);
                Serial.print(";  alt_MSL: ;");
                Serial.print(alt_MSL);
                Serial.print(";  speed_3d ;");
                Serial.print(speed_3d);
                Serial.print(";    ground_speed ;");
                Serial.print(ground_speed);
                Serial.print(";  ground_course ;");
                Serial.print(ground_course);
                Serial.print(";  ecefVZ ;");
                Serial.print(ecefVZ);
                Serial.print("; numSV ;");
                Serial.println((int)(numSV));
#endif         
        }
        
               
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
