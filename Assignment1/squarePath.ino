  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
}

void loop() {
  uint8_t i;
  uint8_t k = 0, l = 0;
  Serial.print("Executing: SQUARE!!!\n\n");

  //frontRightM->setSpeed(255);
  //frontLeftM->setSpeed(100);
  //backLeftM->setSpeed(100);
  //backRightM->setSpeed(255);

  // Run Forward -----------------------------------
  backLeftM->run(FORWARD);
  frontRightM->run(FORWARD);
  frontLeftM->run(FORWARD);
  backRightM->run(FORWARD);

  for(i=0; i<255; i++) {
    frontRightM->setSpeed(i);
    frontLeftM->setSpeed(i);
    backLeftM->setSpeed(i);
    backRightM->setSpeed(i);
    delay(10);
  }

  delay(2000);

  for(i=255; i!=0; i--) {
    frontRightM->setSpeed(i);
    frontLeftM->setSpeed(i);
    backLeftM->setSpeed(i);
    backRightM->setSpeed(i);
    delay(10);
  }

  // Turn -----------------------------------
  frontLeftM->run(BACKWARD);
  backLeftM->run(BACKWARD);
  frontLeftM->setSpeed(255);
  for (i=0; i<255; i++) {
      frontRightM->setSpeed(i);
      backRightM->setSpeed(i);
      //frontLeftM->setSpeed(i);
      //backLeftM->setSpeed(i);
      delay(10);
  }
  
  delay(4500); 
  
  frontRightM->run(RELEASE);
  frontLeftM->run(RELEASE);
  backRightM->run(RELEASE);
  backLeftM->run(RELEASE);
}
