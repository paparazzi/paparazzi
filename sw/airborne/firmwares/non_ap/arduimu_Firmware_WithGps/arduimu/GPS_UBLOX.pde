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
  Serial.begin(38400); 
  pinMode(2,OUTPUT); //Serial Mux
  digitalWrite(2,HIGH); //Serial Mux
} 

/****************************************************************
 * 
 ****************************************************************/
// optimization : This code doesn`t wait for data, only proccess the data available in the Serial buffer
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_ubx_gps() to parse and update the GPS info.
void decode_gps(void)
{ 
  byte data;
  int numc;
  
  numc = Serial.available();
  if (numc > 0) {
    for (int i=0;i<numc;i++)  // Process bytes received
    {
      data = Serial.read();
      switch(UBX_step)     //Normally we start from zero. This is a state machine
      {
      case 0:  
        if(data==0xB5)  // UBX sync char 1
          UBX_step++;   //first data packet is correct, so jump to the next step
        break; 
      case 1:  
        if(data==0x62)  // UBX sync char 2
          UBX_step++;   //ooh! The second data packet is correct, jump to the step 2
        else 
          UBX_step=0;   //Nop, is not correct so restart to step zero and try again.     
        break;
      case 2:
        UBX_class=data;
        checksum(UBX_class);
        UBX_step++;
        break;
      case 3:
        UBX_id=data;
        checksum(UBX_id);
        UBX_step++;
        break;
      case 4:
        UBX_payload_length_hi=data;
        checksum(UBX_payload_length_hi);
        UBX_step++;
        if (UBX_payload_length_hi>UBX_MAXPAYLOAD)
        {
#if PRINT_DEBUG != 0
          Serial.print("???GPS:1,BAD Payload length:");   
          Serial.print(UBX_payload_length_hi,DEC);
          Serial.println("***");    
#endif
          UBX_step=0;   //Bad data, so restart to step zero and try again.     
          UBX_payload_counter=0;
          ck_a=0;
          ck_b=0;
#if PERFORMANCE_REPORTING == 1
          gps_payload_error_count++;
#endif
        }
        break;
      case 5:
        UBX_payload_length_lo=data;
        checksum(UBX_payload_length_lo);
        UBX_step++;
        break;
      case 6:         // Payload data read...
					// We need to process the data byte before we check the number of bytes so far
					/*UBX_buffer[UBX_payload_counter] = data;
					checksum(data);
					UBX_payload_counter++;
                                         
					if (UBX_payload_counter < UBX_payload_length_hi) {
						// We stay in this state until we reach the payload_length
					} else {
						UBX_step++; // payload_length reached - next byte is checksum
					}*/
                                        
                                        if (UBX_payload_counter < UBX_payload_length_hi) 
                                        {
                                          UBX_buffer[UBX_payload_counter] = data;
                                          checksum(data);
                                          UBX_payload_counter++;
                                          
                                        if (UBX_payload_counter==UBX_payload_length_hi)
                                        UBX_step++;
                                        
                                        }
                                        
                                        
					break;
      case 7:
        UBX_ck_a=data;   // First checksum byte
        UBX_step++;
        break;
      case 8:
        UBX_ck_b=data;   // Second checksum byte
       
	  // We end the GPS read...
        if((ck_a==UBX_ck_a)&&(ck_b==UBX_ck_b))   // Verify the received checksum with the generated checksum.. 
	  	parse_ubx_gps();               // Parse new GPS packet...

        else
        {
#if PERFORMANCE_REPORTING == 1          
            gps_checksum_error_count++;
#endif
            debug_handler(10);    
            Serial.print("???");
            Serial.print((int)UBX_ck_a);
            Serial.print("-");
            Serial.print((int)ck_a);
            Serial.println("***");
            
            Serial.print("???");
            Serial.print((int)UBX_ck_b);
            Serial.print("-");
            Serial.print((int)ck_b);
            Serial.println("***");
            
        }

        // Variable initialization
        UBX_step=0;
        UBX_payload_counter=0;
        ck_a=0;
        ck_b=0;
        GPS_timer=DIYmillis(); //Restarting timer...
        break;
	}
    }    // End for...
  }  
  if(DIYmillis() - GPS_timer > 2000){
      digitalWrite(6, LOW);  //If we don't receive any byte in two seconds turn off gps fix LED...
      debug_print("Yeah, your GPS is disconnected"); 
      gpsFix=1; 
  }
  
}

/****************************************************************
 * 
 ****************************************************************/
void parse_ubx_gps()
{
  int j;
  
//Verifing if we are in class 1, you can change this "IF" for a "Switch" in case you want to use other UBX classes.. 
//In this case all the message im using are in class 1, to know more about classes check PAGE 60 of DataSheet.
  if(UBX_class==0x01) 
  {
    switch(UBX_id)//Checking the UBX ID
    {
      case 0x02: //ID NAV-POSLLH 
#if PERFORMANCE_REPORTING == 1
         gps_pos_fix_count++;
#endif
        j=0;
        iTOW = join_4_bytes(&UBX_buffer[j]);
        j+=4;
        lon = join_4_bytes(&UBX_buffer[j]);
        j+=4;
        lat = join_4_bytes(&UBX_buffer[j]);
        j+=4;
        alt = join_4_bytes(&UBX_buffer[j]);
        j+=4;
        alt_MSL = join_4_bytes(&UBX_buffer[j]);
        j+=4;
        /*
        hacc = (float)join_4_bytes(&UBX_buffer[j])/1000.0;
        j+=4;
        vacc = (float)join_4_bytes(&UBX_buffer[j])/1000.0;
        j+=4;
        */
          gpsFixnew=1;  //new information available flag for binary message
      break;
    case 0x03://ID NAV-STATUS 
       if((UBX_buffer[4] >= 0x03)&&(UBX_buffer[5]&0x01))
        {
          gpsFix=0; //valid position
          digitalWrite(6,HIGH);  //Turn LED when gps is fixed. 
          GPS_timer=DIYmillis(); //Restarting timer...
        }
        else
        {
          gpsFix=1; //invalid position
          digitalWrite(6,LOW);
        }
      break;
      
    case 0x06://ID NAV-SOL
       if((UBX_buffer[10] >= 0x03)&&(UBX_buffer[11]&0x01))
        {
          gpsFix=0; //valid position
          digitalWrite(6,HIGH);  //Turn LED when gps is fixed. 
          GPS_timer=DIYmillis(); //Restarting timer...
        }
        else
        {
          gpsFix=1; //invalid position
          digitalWrite(6,LOW);
        }
        
        ecefVZ=(float)join_4_bytes(&UBX_buffer[36])/100; //Vertical Speed
        
        numSV=UBX_buffer[47]; //Number of sats...     
    break;

    case 0x12:// ID NAV-VELNED  
#if PERFORMANCE_REPORTING == 1
         gps_nav_fix_count++;
#endif
      j=16;
      speed_3d = (float)join_4_bytes(&UBX_buffer[j])/100.0; // m/s
      j+=4;
      ground_speed = (float)join_4_bytes(&UBX_buffer[j])/100.0; // Ground speed 2D
      j+=4;
      ground_course = (float)join_4_bytes(&UBX_buffer[j])/100000.0; // Heading 2D
      j+=4;
      /*
      sacc = join_4_bytes(&UBX_buffer[j]) // Speed accuracy
      j+=4;
      headacc = join_4_bytes(&UBX_buffer[j]) // Heading accuracy
      j+=4;
      */
	  if (ground_speed > SPEEDFILT && gpsFix==0) gc_offset = ground_course - ToDeg(yaw);
      break; 
      }
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
