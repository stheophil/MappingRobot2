char report[80];
static bool bFirstRun = true;

void loop() {
    if(bFirstRun) {
        Serial.println("AHRS Test");
        
        OnConnection(); // calibrate with running motors to test with interference
        for(int i=0; i<countof(g_amotors); ++i) {
            analogWrite(g_amotors[i].POWER, 128);
        }
        bFirstRun = false;
    } else {
       /* Get the four calibration values (0..3) */
      /* Any sensor data reporting 0 should be ignored, */
      /* 3 means 'fully calibrated" */
      uint8_t system, gyro, accel, mag;
      system = gyro = accel = mag = 0;
      g_bno.getCalibration(&system, &gyro, &accel, &mag);
     
      /* Display the individual values */
      Serial.print("Sys:");
      Serial.print(system, DEC);
      Serial.print(" G:");
      Serial.print(gyro, DEC);
      Serial.print(" A:");
      Serial.print(accel, DEC);
      Serial.print(" M:");
      Serial.print(mag, DEC);
      Serial.print("\t\t\t");

      /* Get a new sensor event */ 
      sensors_event_t event; 
      g_bno.getEvent(&event);
      
      /* Display the floating point data */
      Serial.print("X: ");
      Serial.print(event.orientation.x, 4);
      Serial.print("\tY: ");
      Serial.print(event.orientation.y, 4);
      Serial.print("\tZ: ");
      Serial.print(event.orientation.z, 4);
      Serial.println("");
      delay(100);
  }
}