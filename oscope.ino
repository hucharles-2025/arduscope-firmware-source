#define BUFFER_SIZE 700
#define SAMPLES_AFTER_TRIGGER 690

#define ARMED_LIGHT 4
#define TRIG_LIGHT 6
#define ONEVOLTREF_LIGHT A1
#define AVCCREF_LIGHT A3
#define EXTREF_LIGHT A5
#define SINGLE_SHOT_LIGHT 9
#define COMMAND_LIGHT 11

volatile uint8_t readingL, readingH, currentReadingTime, lowData[BUFFER_SIZE], highData[BUFFER_SIZE];
volatile uint16_t writeHead = 0, triggerPosition = 0, stopRecordPosition = 0, triggeringData = 0;
volatile bool transmitting = false, triggered = false, hasCycledAtLeastOnePastThrough = false, commandMode = false, awaitingManualPress = false, thatWasInvalid = false;;

// variables used to store the state of the scope & the input cmd string
bool singleShotMode = false;
uint8_t prescaler_bitmask = 0b00000101; // defaults to 32^-1
uint8_t vref_bitmask = 0b01000000; // defaults to 01 (AVCC / 5V)
uint16_t triggerLevel = 100;
float highRefVoltage = 5.0; // defaults to 5.0

void disableADCInterrupt() {
  // It is better to shut down the ADC entirely than to 
  // simply shut down interrupts, since the ADC is sometimes
  // shut down to make changes to ADMUX and ADCSRA
  // which can't safely happen if conversions are still ongoing
  digitalWrite(ARMED_LIGHT, LOW);
  digitalWrite(TRIG_LIGHT, LOW);
  ADCSRA &= ~(1 << ADEN);
}

void enableADCInterrupt() {
  digitalWrite(ARMED_LIGHT, HIGH);
  ADCSRA |= (1 << ADEN);
  ADCSRA |= (1 << ADSC);
}

void resetDataAcqCycle() {
  triggered = false; 
  digitalWrite(TRIG_LIGHT, LOW);
  hasCycledAtLeastOnePastThrough = false;
  writeHead = 0; 
  triggerPosition = 0; 
  stopRecordPosition = 0; 
  triggeringData = 0;
  transmitting = false;
  enableADCInterrupt();
}

void setup() {
  // 1. Set up the onboard ADC
  // ADC MUX AND REF SELECT
  // * High reference   AVCC       01       MSB 
  // * ADC data align   R align    0
  // * Unused bit                  0
  // * Pin select       A0         0000     LSB
  ADMUX = 0b01000000;

  // ADC CTRL AND STATUS A
  // * ADC enable       YES        1        MSB
  // * Initiate acq     YES        1
  // * Auto trig        YES        1
  // * Output ready?    NOT YET    0
  // * Fire interrupt   YES        1
  // * Clock prescale   32^-1      101      LSB
  ADCSRA = 0b11101101;

  // ADC CTRL AND STATUS B
  // * Unused bit                  0        MSB
  // * ACME             NO         0
  // * Unused bits                 000
  // * Trigger source   FREE RUN   000      LSB
  ADCSRB = 0b00000000;

  // 2. Initiate high speed UART communications with 
  // the controlling PC 
  Serial.begin(1000000);

  // 3. Make timer1 operate at 0.5us/tick with prescaler 8
  TCCR1B &= ~0x07;   // clear CS12:0
  TCCR1B |= (1 << CS11);  // CS11=1 makes prescaler 8

  // 4. Gain control of the onboard LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // 5. Handle the disarm arm button being pressed
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), handleArmDisarmButtonPressed, RISING);

  // 6. Configure the manual trigger / rearm button
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), handleManualButtonPressed, RISING);

  // 7. Set up the default values for the status light
  pinMode(ARMED_LIGHT, OUTPUT);
  pinMode(TRIG_LIGHT, OUTPUT);
  pinMode(ONEVOLTREF_LIGHT, OUTPUT);
  pinMode(AVCCREF_LIGHT, OUTPUT);
  pinMode(EXTREF_LIGHT, OUTPUT);
  pinMode(SINGLE_SHOT_LIGHT, OUTPUT);
  pinMode(COMMAND_LIGHT, OUTPUT);

  digitalWrite(ARMED_LIGHT, HIGH); // Armed light starts out HI because scope is always armed on startup
  digitalWrite(TRIG_LIGHT, LOW); // Trigger light starts LO because scope is always not trig'd on startup
  digitalWrite(ONEVOLTREF_LIGHT, LOW); // 1.1V + EXT REF  light start out as LO bc scope starts out on AVCC, AVCC starts HI
  digitalWrite(AVCCREF_LIGHT, HIGH);
  digitalWrite(EXTREF_LIGHT, LOW);
  digitalWrite(SINGLE_SHOT_LIGHT, LOW); // Single shot light start out LO bc free run is the default
  digitalWrite(COMMAND_LIGHT, LOW); // Cmd light starts out LO bc we're not in cmd mode on startup
}

ISR(ADC_vect) {
  // Defines what happens when the ADC reports that a reading is ready

  // Read ADCLo, ADCHi and timer 1 (which tells you how many microseconds
  // it's been since the last reading)
  readingL = ADCL;
  readingH = ADCH;
  currentReadingTime = TCNT1; // this time is usually less than 255, so we can afford to demote here

  // Reset timer1 so you know how much time it's been
  TCNT1 = 0;

  // Write to a buffer of items if we've been triggered and we haven't filled the entire buffer yet
  lowData[writeHead] = readingL;
  highData[writeHead] = (currentReadingTime & 0b11111100) | readingH;

  // Check if we've activated the trigger and if we've recorded any new history 
  // (this is to prevent old history from being shown if the scope is triggered immediately after a tx of data)
  if (!triggered && ((((uint16_t)readingH) << 8) | readingL) > triggerLevel && hasCycledAtLeastOnePastThrough) {
    // Mark the trigger location and set the trigger flag
    triggered = true;
    triggerPosition = writeHead;

    // Determine how much post-trigger we want to record and how much pre-trigger we want to keep.
    stopRecordPosition = (triggerPosition + SAMPLES_AFTER_TRIGGER) % BUFFER_SIZE;
    
    // Record the amplitude of the signal at trigger
    triggeringData = ((((uint16_t)readingH) << 8) | readingL);
  }

  // Adv the buffer idx; if it rolls over, bring it back to 0 (since we hvae a ring buffer)
  writeHead = (writeHead + 1) % BUFFER_SIZE;

  if (!triggered && writeHead > SAMPLES_AFTER_TRIGGER) {
    hasCycledAtLeastOnePastThrough = true;
  }

  // If we are at the place to stop recording, and the trigger was already active,
  // we have filled the entire ring buffer, so get ready to transmit
  if (triggered && writeHead == stopRecordPosition) {
    transmitting = true; 
    disableADCInterrupt();
  }

}

// When the arm/disarm button is pressed, stop data acquisition
// and enter cmd accept mode
void handleArmDisarmButtonPressed() {
  if (commandMode == false) {
    commandMode = true;
  }
}

// When the manual button is pressed:
void handleManualButtonPressed() {
  // If in single shot mode and transmit sequence is done, 
  // restart data acq. Can NOT be used to exit command mode tho
  if (awaitingManualPress) {
    resetDataAcqCycle();
    awaitingManualPress = false;
  }

  // If operating at any time but the user wanted to force a triggering, 
  // emulate the change of variables that would happen with an organic triggering.
  else if (triggered == false && transmitting == false && commandMode == false) {
    // Set this to be true, since we expect it to be true during an organic trig
    hasCycledAtLeastOnePastThrough = true;

    // Mark the trigger location and set the trigger flag
    triggered = true;
    triggerPosition = writeHead;

    // Determine how much post-trigger we want to record and how much pre-trigger we want to keep.
    stopRecordPosition = (triggerPosition + SAMPLES_AFTER_TRIGGER) % BUFFER_SIZE;
    
    // Record the amplitude of the signal at trigger
    triggeringData = ((((uint16_t)readingH) << 8) | readingL);
  }
}

void loop() {
  if (commandMode) {
    // In the event that the arm disarm button was pressed during transmission, 
    // this will re-disable the data acquisition cycle following the resetDataAcq call
    // at the end of the if (transmitting) {} block.
    disableADCInterrupt();
    delay(50);
    String currentCmd;

    digitalWrite(COMMAND_LIGHT, HIGH);
    digitalWrite(TRIG_LIGHT, LOW); // If the cmd button is pressed during single shot mode and the scope isn't armed, TRIG will still be on so shut it off

    // Start listening for commands after sending header (retransmit header until we get a response)
    digitalWrite(ARMED_LIGHT, HIGH);
    while (Serial.available() < 1) {
      uint8_t frameComplete[3] = {0xff, 0xff, 0b11110100};
      Serial.write(frameComplete, 3);
      uint8_t frameStartCmdMode[3] = {0xff, 0xff, 0b11110101};
      Serial.write(frameStartCmdMode, 3);
    }
    while (Serial.available() > 0) {
      Serial.read();
    }
    Serial.println("<!>DISPLAY Two-way ack OK, ready to accept cmds");
    Serial.setTimeout(4294967295);
    digitalWrite(ARMED_LIGHT, LOW);
    while (true) {
      thatWasInvalid = false;
      currentCmd = Serial.readStringUntil('\n');
      if (currentCmd.substring(0,8) == F("PRESCALE")) {
        uint8_t prescale_division = (uint8_t)currentCmd.substring(9).toInt();
        switch (prescale_division) {
          case 2:
            prescaler_bitmask = 0b00000001;
            break;
          case 4:
            prescaler_bitmask = 0b00000010;
            break;
          case 8:
            prescaler_bitmask = 0b00000011;
            break;
          case 16:
            prescaler_bitmask = 0b00000100;
            break;
          case 32:
            prescaler_bitmask = 0b00000101;
            break;
          case 64:
            prescaler_bitmask = 0b00000110;
            break;
          case 128:
            prescaler_bitmask = 0b00000111;
            break;
          default:
            Serial.print(F("<!>DISPLAY "));
            Serial.print(prescale_division);
            Serial.println(F(" is not a valid division factor for the ADC's prescaler. Valid options are 2, 4, 8, 16, 32, 64, 128"));
            thatWasInvalid = true;
            break;
        }
        ADCSRA = (ADCSRA & 0b11111000) | prescaler_bitmask;
        if (!thatWasInvalid) {
          Serial.print(F("<!>PRESCALER "));
          Serial.println(prescale_division);
          Serial.print(F("<!>DISPLAY Prescaler division factor has been set to "));
          Serial.print(prescale_division);
          Serial.print(F(" [prescaler bitmask = "));
          Serial.print(prescaler_bitmask);
          Serial.print(F(", ADCSRA = "));
          Serial.print(ADCSRA);
          Serial.println(F("]"));
        }
      }
      else if (currentCmd.substring(0,7) == F("TRIGLVL")) {
        triggerLevel = (uint16_t)currentCmd.substring(8).toInt();
        Serial.print(F("<!>TRIGGERLINE "));
        Serial.println(triggerLevel);
        Serial.print(F("<!>DISPLAY Trigger set at "));
        Serial.print(triggerLevel);
        Serial.print(F(" [+"));
        Serial.print( (triggerLevel / 1023.0) * highRefVoltage );
        Serial.print(F("V @ VREF = +"));
        Serial.print(highRefVoltage);
        Serial.println(F("V]"));
      }
      else if (currentCmd.substring(0,10) == F("SINGLESHOT")) {
        singleShotMode = true;
        Serial.println(F("<!>SINGLESMODE"));
        Serial.println(F("<!>DISPLAY Single trigger mode enabled"));
        digitalWrite(SINGLE_SHOT_LIGHT, HIGH);
      }
      else if (currentCmd.substring(0,11) == F("REPEATSHOTS")) {
        singleShotMode = false;
        Serial.println(F("<!>FREEMODE"));
        Serial.println(F("<!>DISPLAY Repeated trigger mode enabled"));
        digitalWrite(SINGLE_SHOT_LIGHT, LOW);
      }
      else if (currentCmd.substring(0,10) == F("VREFSOURCE")) {
        if (currentCmd.substring(11) == F("EXTERNAL")) {
          vref_bitmask = 0b00000000;
          digitalWrite(ONEVOLTREF_LIGHT, LOW);
          digitalWrite(AVCCREF_LIGHT, LOW);
          digitalWrite(EXTREF_LIGHT, HIGH);
        }
        else if (currentCmd.substring(11) == F("1.1")) {
          vref_bitmask = 0b11000000;
          digitalWrite(ONEVOLTREF_LIGHT, HIGH);
          digitalWrite(AVCCREF_LIGHT, LOW);
          digitalWrite(EXTREF_LIGHT, LOW);
          Serial.println(F("<!>HIGHREFVOLTAGE 1.1"));
          highRefVoltage = 1.1;
        }
        else if (currentCmd.substring(11) == F("5.0")) {
          vref_bitmask = 0b01000000;
          digitalWrite(ONEVOLTREF_LIGHT, LOW);
          digitalWrite(AVCCREF_LIGHT, HIGH);
          digitalWrite(EXTREF_LIGHT, LOW);
          Serial.println(F("<!>HIGHREFVOLTAGE 5.0"));
          highRefVoltage = 5.0;
        }
        else {
          Serial.println(F("<!>DISPLAY Invalid option for voltage reference source. Valid options are EXTERNAL, 1.1, and 5.0"));
          thatWasInvalid = true;
        }
        ADMUX = (ADMUX & 0b00111111) | vref_bitmask;
        if (!thatWasInvalid) {
          Serial.print(F("<!>DISPLAY ADC high reference voltage is now "));
          Serial.print(currentCmd.substring(11));
          Serial.print(F(" : VREF bitmask = "));
          Serial.print(vref_bitmask);
          Serial.print(F(", ADMUX = "));
          Serial.print(ADMUX);
          Serial.println(F("]"));
        }
      }
      else if (currentCmd.substring(0,5) == F("REARM")) {
        Serial.println(F("<!>DISPLAY Rearming..."));
        resetDataAcqCycle();

        digitalWrite(COMMAND_LIGHT, LOW);

        // resetDataAcqCycle doesn't automatically change
        // commandMode back to false, so we have to do it manually
        commandMode = false; 
        break;
      }
      else {
        Serial.println(F("<!>DISPLAY Command not recognized (or faulty UART transmission)"));
      }
    }
  }

  // Once a batch of data is ready to transmit, send it off and reset
  // the flags so data acq can resume after we're done
  if (transmitting) {
    // Turn on the transmit/trig light
    digitalWrite(TRIG_LIGHT, HIGH);

    // Send the command to clear the screen
    uint8_t frameClear[3] = {0b11111111, 0b11111111, 0b11110000};
    Serial.write(frameClear, 3);

    // Figure out how many microseconds BEFORE the trigger were recorded
    uint16_t microsecondsBeforeTrigger = 0;
    uint16_t countHead = stopRecordPosition;
    while (countHead != triggerPosition) {
      microsecondsBeforeTrigger += (uint16_t)((highData[countHead] & 0b11111100 >> 2) / 2);
      countHead = (countHead + 1) % BUFFER_SIZE;
      //Serial.print((uint16_t)((highData[countHead] & 0b11111100 >> 2) / 2));
      //Serial.print(F("+"));
    }
    //Serial.print(F("\n="));
    //Serial.println(microsecondsBeforeTrigger);
    uint8_t frameStartUs[3] = {
      (uint8_t)(microsecondsBeforeTrigger >> 8), 
      (uint8_t)(microsecondsBeforeTrigger & 0b0000000011111111), 
      0b11110010
    };
    Serial.write(frameStartUs, 3);

    // Dump the entire buffer out
    uint16_t transmitHead = stopRecordPosition;
    // Serial.print(F("Triggering data: "));
    // Serial.println(triggeringData);
    // Serial.print(F("Triggered at idx: "));
    // Serial.println(triggerPosition);
    // Serial.print(F("Recorded until: "));
    // Serial.println(stopRecordPosition);
    do {
      uint8_t frameData[3] = {highData[transmitHead], lowData[transmitHead], 0b11110001};
      Serial.write(frameData, 3);
      // Serial.print(F("bufidx="));
      // Serial.print(transmitHead);
      // Serial.print(F(" d_us="));
      // Serial.print((highData[transmitHead] & 0b11111100) >> 2);
      // Serial.print(F(" high="));
      // Serial.print(highData[transmitHead]);
      // Serial.print(F(" low="));
      // Serial.print(lowData[transmitHead]);
      // Serial.print(F(" adc="));
      // Serial.print(((uint16_t)(highData[transmitHead] & 0b00000011)) << 8 | lowData[transmitHead]);
      // Serial.println(F(";"));

      if (transmitHead == triggerPosition) {
        uint8_t frameZeroPointReached[3] = {0xff, 0xff, 0b11110011};
        Serial.write(frameZeroPointReached, 3);
      }

      transmitHead = (transmitHead + 1) % BUFFER_SIZE;
    }
    while (transmitHead != stopRecordPosition);

    // Transmit the end of batch signal
    uint8_t frameComplete[3] = {0xff, 0xff, 0b11110100};
    Serial.write(frameComplete, 3);

    // If we're in free run mode, start the next acq cycle. 
    // Otherwise, don't do anything and await the rearm button being pressed.
    if (singleShotMode == false) {
      // Reset everything so its ready for the next run
      resetDataAcqCycle();
    }
    else {
      // If we await the button, stop transmission of data
      // so we're not replaying the same information
      transmitting = false;
      awaitingManualPress = true;
    }

    
  }
}