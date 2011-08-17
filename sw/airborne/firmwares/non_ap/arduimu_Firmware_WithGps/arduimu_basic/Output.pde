
//*****I2C Output************************************************************
void fill_I2C_message() {
  // Message Array : Roll; Pitch; YaW; GyroX; GyroY; GyroZ; ACCX; ACCY; ACCZ
  // Float Number is multipited with 10000 and converted to an Integer, for sending via I2C.
  // Resolution for angles: 12, rates: 12, accel:10 (from pprza BFP math lib)
  I2C_Message_ar[0] = int(roll*(1<<12));
  I2C_Message_ar[1] = int(pitch*(1<<12));
  I2C_Message_ar[2] = int(yaw*(1<<12));
  I2C_Message_ar[3] = int(Omega_Vector[0]*(1<<12));
  I2C_Message_ar[4] = int(Omega_Vector[1]*(1<<12));
  I2C_Message_ar[5] = int(Omega_Vector[2]*(1<<12));
  I2C_Message_ar[6] = int((9.81*Accel_Vector[0]/GRAVITY)*(1<<10));
  I2C_Message_ar[7] = int((9.81*Accel_Vector[1]/GRAVITY)*(1<<10));
  I2C_Message_ar[8] = int((9.81*Accel_Vector[2]/GRAVITY)*(1<<10));

}

void requestEvent(){
#if PRINT_DEBUG != 0
  Serial.println("Sending IMU Data");
#endif

  fill_I2C_message();

  byte* pointer;
  pointer = (byte*) &I2C_Message_ar;
  Wire.send(pointer, 18);

}

//******** GPS Data from Paparazzi AP ******************************************

void receiveEvent(int howMany){
#if PRINT_DEBUG != 0
  Serial.print("Receiving GPS Bytes : "); 
  Serial.println(howMany);
#endif

  for(int i=0; i < howMany; i++){
    Paparazzi_GPS_buffer[i]=Wire.receive();
  }

  parse_pprz_gps();               // Parse new GPS packet...
  GPS_timer=DIYmillis();         //Restarting timer...

}


//*************************************************************************************************************


void printdata(void){    

#if PRINT_I2C_MSG == 1
  fill_I2C_message();
  Serial.print("Time ");
  Serial.print(millis());
  Serial.print(";  Roll ");
  Serial.print(I2C_Message_ar[0]);
  Serial.print(";  Pitch ");
  Serial.print(I2C_Message_ar[1]);
  Serial.print(";  YaW ");
  Serial.print(I2C_Message_ar[2]);
  Serial.print(";  GyroX ");
  Serial.print(I2C_Message_ar[3]);
  Serial.print(";  GyroY ");
  Serial.print(I2C_Message_ar[4]);
  Serial.print(";  GyroZ ");
  Serial.print(I2C_Message_ar[5]);
  Serial.print(";  ACCX ");
  Serial.print(I2C_Message_ar[6]);
  Serial.print(";  ACCY ");
  Serial.print(I2C_Message_ar[7]);
  Serial.print(";  ACCZ ");
  Serial.println(I2C_Message_ar[8]);
#endif

  //Serial.print("!!!VER:");
  //Serial.print(SOFTWARE_VER);  //output the software version
  //Serial.print(",");

#if PRINT_ANALOGS == 1
  Serial.print("AN0:");
  Serial.print(read_adc(0)); //Reversing the sign. 
  Serial.print(",AN1:");
  Serial.print(read_adc(1));
  Serial.print(",AN2:");
  Serial.print(read_adc(2));  
  Serial.print(",AN3:");
  Serial.print(read_adc(3));
  Serial.print (",AN4:");
  Serial.print(read_adc(4));
  Serial.print (",AN5:");
  Serial.println(read_adc(5));
#endif

#if PRINT_DCM == 1
  Serial.print ("EX0:");
  Serial.print(convert_to_dec(DCM_Matrix[0][0]));
  Serial.print (",EX1:");
  Serial.print(convert_to_dec(DCM_Matrix[0][1]));
  Serial.print (",EX2:");
  Serial.print(convert_to_dec(DCM_Matrix[0][2]));
  Serial.print (",EX3:");
  Serial.print(convert_to_dec(DCM_Matrix[1][0]));
  Serial.print (",EX4:");
  Serial.print(convert_to_dec(DCM_Matrix[1][1]));
  Serial.print (",EX5:");
  Serial.print(convert_to_dec(DCM_Matrix[1][2]));
  Serial.print (",EX6:");
  Serial.print(convert_to_dec(DCM_Matrix[2][0]));
  Serial.print (",EX7:");
  Serial.print(convert_to_dec(DCM_Matrix[2][1]));
  Serial.print (",EX8:");
  Serial.println(convert_to_dec(DCM_Matrix[2][2]));
#endif

#if PRINT_EULER == 1
  Serial.print("RLL:");
  Serial.print(ToDeg(roll));
  Serial.print(",PCH:");
  Serial.print(ToDeg(pitch));
  Serial.print(",YAW:");
  Serial.print(ToDeg(yaw));
  Serial.print(",IMUH:");
  Serial.println((imu_health>>8)&0xff);
#endif

#if PRINT_GPS == 1
  Serial.print("COG:");
  Serial.print((ground_course));
  Serial.print(",SOG:");
  Serial.print(ground_speed);
  Serial.print(",FIX:");
  Serial.print((int)gpsFix);
  Serial.print (",");
#if PERFORMANCE_REPORTING == 1
  gps_messages_sent++;
#endif
  Serial.println("");
#endif

}

void printPerfData(long time)
{
  Serial.print("PPP");
  Serial.print("pTm:");
  Serial.print(time,DEC);
  Serial.print(",mLc:");
  Serial.print(mainLoop_count,DEC);
  Serial.print(",DtM:");
  Serial.print(G_Dt_max,DEC);
  Serial.print(",gsc:");
  Serial.print(gyro_sat_count,DEC);
  Serial.print(",adc:");
  Serial.print(adc_constraints,DEC);
  Serial.print(",rsc:");
  Serial.print(renorm_sqrt_count,DEC);
  Serial.print(",rbc:");
  Serial.print(renorm_blowup_count,DEC);
  Serial.print(",gpe:");
  Serial.print(gps_payload_error_count,DEC);
  Serial.print(",gce:");
  Serial.print(gps_checksum_error_count,DEC);
  Serial.print(",gpf:");
  Serial.print(gps_pos_fix_count,DEC);
  Serial.print(",gnf:");
  Serial.print(gps_nav_fix_count,DEC);
  Serial.print(",gms:");
  Serial.print(gps_messages_sent,DEC);
  Serial.print(",imu:");
  Serial.print((imu_health>>8),DEC);
  Serial.println(",***");

  // Reset counters
  mainLoop_count = 0;
  G_Dt_max  = 0;
  gyro_sat_count = 0;
  adc_constraints = 0;
  renorm_sqrt_count = 0;
  renorm_blowup_count = 0;
  gps_payload_error_count = 0;
  gps_checksum_error_count = 0;
  gps_pos_fix_count = 0;
  gps_nav_fix_count = 0;
  gps_messages_sent = 0;


}


long convert_to_dec(float x)
{
  return x*10000000;
}

