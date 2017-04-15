
// Final Project v 0.1
// Microprocessors for Robotics 525.410
// Doyle 4.11.17
#include <Arduino_FreeRTOS.h>
#include <MeAuriga.h>
#include <Arduino.h>
#define BUZZER_PORT 45


MeEncoderOnBoard Encoder_1(SLOT1); // 1 is right, needs negative value to move forward
MeEncoderOnBoard Encoder_2(SLOT2); // 2 is left, needs positive value to move forward
volatile int soundVal = 0;
volatile int soundHeading = 0;
long int clapTime = 0;

double const INCHES_PER_TURN = 4.91;
int const DEGREES_NEEDED_FOR_ROTATION = 2000; // tested empirically
enum MotionStates { WAITING_FOR_HEADING, IN_PURSUIT_TURNING, IN_PURSUIT_CHASING };
volatile int marcoMotionState = WAITING_FOR_HEADING;

//Variables for ADC and Bearing calculations
int count = 0;
int channel_A_sound, channel_B_sound, channel_C_sound; //these will store the values of the adc registers
int max_sound = 110;
int debounceDelay = 20000;
int loud_noise_flag_A, loud_noise_flag_B, loud_noise_flag_C;  //these are the flags when the sound is higher than the max sound
int noise_state_A, noise_state_B, noise_state_C; //these are the flags for a debounced loud noise on each channel.
int lastDebounceTime_A, lastDebounceTime_B, lastDebounceTime_C; //these are variables to use for the debounce comparison
int last_reading_A, last_reading_B, last_reading_C;   //these store the last values for the loud_noise_flags
int temp = 0;
char ADMUX_A = 0b11100011;
char ADMUX_B = 0b11100100;
char ADMUX_C = 0b11100101;
bool mux = 1;
int bearing = 0;
bool no_direction = 1;  //set to one by default just so I can start to detect bearing immediately.  This should be set to 0 in our program.
int degree;

// define tasks
void TaskAnalogRead( void *pvParameters );
void TaskMotion( void *pvParameters );
void TaskController( void *pvParameters );
// void TaskParseControls( void *pvParameters);

void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();
  }
}
void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }  
}


ISR(ADC_vect)  //This is the function that is called whenever the ADC completes a conversion
{
  if(count == 0) {
    temp = ADCH;

    //Check to see if the analog input is higher than the threshold
    if(temp >= max_sound) {
      loud_noise_flag_A = 1;
    }
    else {
      loud_noise_flag_A = 0;
    }
  
    if (loud_noise_flag_A != last_reading_A) {  //if our flag does not match the last time we had a reading, then we set the time that we got this new loud noise flag to be used to debounce
      lastDebounceTime_A = micros();
    }
    
    if ((micros() - lastDebounceTime_A ) > debounceDelay) {  //If we have had the high loud noise for a certain amount of time we check to see if it's bee high, then we act as if we have a loud noise (debounced)
      if(loud_noise_flag_A != noise_state_A) {
        noise_state_A = loud_noise_flag_A;
        if(no_direction && noise_state_A == HIGH) {//if we have a debounced loud noise (debounce time has surpassed and the noise state is high), then we want to check to see if there have been any other
                                                     //legit loud noise on microphones.  If there have, depending on which came first, we determin the direction of the source. Then we clear the no_direction
                                                     //flag to let other programs know that we have a bearing.  If we haven't had another microphone then the current mic is the first to recieve a signal.
          if(channel_B_sound) {
            bearing = 1;
            no_direction = 0;
            Serial.println(bearing);  
          }
          else if (channel_C_sound) {
            bearing = 4;
            no_direction = 0;
            Serial.println(bearing);  
          }
        }
      }
      if (noise_state_A == HIGH) {  //set the noise state of the current mic to be used elsewhere if needed
        channel_A_sound = 1;
      }
      else {
        channel_A_sound = 0;
      }
    }

    last_reading_A = loud_noise_flag_A;
    if(mux == 1) {ADMUX = ADMUX_B;}   //We want to move to the next plug channel, channel 6
  }
  else if (count == 1) {
    temp = ADCH;
    
     //Check to see if the analog input is higher than the threshold
    if(temp >= max_sound) {
      loud_noise_flag_B = 1;
    }
    else {
      loud_noise_flag_B = 0;
    }
  
    if (loud_noise_flag_B != last_reading_B) {  //if our flag does not match the last time we had a reading, then we set the time that we got this new loud noise flag to be used to debounce
      lastDebounceTime_B = micros();
    }
    
    if ((micros() - lastDebounceTime_B ) > debounceDelay) {  //If we have had the high loud noise for a certain amount of time we check to see if it's bee high, then we act as if we have a loud noise (debounced)
      if(loud_noise_flag_B != noise_state_B) {
        noise_state_B = loud_noise_flag_B;
        if(no_direction && noise_state_B == HIGH) {//if we have a debounced loud noise (debounce time has surpassed and the noise state is high), then we want to check to see if there have been any other
                                                     //legit loud noise on microphones.  If there have, depending on which came first, we determin the direction of the source. Then we clear the no_direction
                                                     //flag to let other programs know that we have a bearing.  If we haven't had another microphone then the current mic is the first to recieve a signal.
          if(channel_A_sound) {
            bearing = 2;
            no_direction = 0;
            Serial.println(bearing);
          }
          else if (channel_C_sound) {
            bearing = 5;
            no_direction = 0;
            Serial.println(bearing);
          }
        }
      }
      if (noise_state_B == HIGH) {//set the noise state of the current mic to be used elsewhere if needed
        channel_B_sound = 1;
      }
      else {
        channel_B_sound = 0;
      }
    }

    last_reading_B = loud_noise_flag_B;
    if (mux == 1) {ADMUX = ADMUX_C;}   //We want to move to the next plug channel, channel 6
  }
  else if (count == 2) {
    temp = ADCH;
    
     //Check to see if the analog input is higher than the threshold
    if(temp >= max_sound) {
      loud_noise_flag_C = 1;
    }
    else {
      loud_noise_flag_C = 0;
    }
  
    if (loud_noise_flag_C != last_reading_C) {  //if our flag does not match the last time we had a reading, then we set the time that we got this new loud noise flag to be used to debounce
      lastDebounceTime_C = micros();
    }
    
    if ((micros() - lastDebounceTime_C ) > debounceDelay) {  //If we have had the high loud noise for a certain amount of time we check to see if it's high, then we act as if we have a loud noise (debounced)
      if(loud_noise_flag_C != noise_state_C) {
        noise_state_C = loud_noise_flag_C;
        if(no_direction && noise_state_C == HIGH) {  //if we have a debounced loud noise (debounce time has surpassed and the noise state is high), then we want to check to see if there have been any other
                                                     //legit loud noise on microphones.  If there have, depending on which came first, we determin the direction of the source. Then we clear the no_direction
                                                     //flag to let other programs know that we have a bearing.  If we haven't had another microphone then the current mic is the first to recieve a signal.
          if(channel_A_sound) {
            bearing = 3;
            no_direction = 0;
            Serial.println(bearing);
          }
          else if (channel_B_sound) {
            bearing = 6;
            no_direction = 0;
            Serial.println(bearing);
          }
        }
      }
      if (noise_state_C == HIGH) {//set the noise state of the current mic to be used elsewhere if needed
        channel_C_sound = 1;
      }
      else {
        channel_C_sound = 0;
      }
    }
    last_reading_C = loud_noise_flag_C;
    if (mux == 1) {ADMUX = ADMUX_A;}   //We want to move to the next plug channel, channel 6
  }  
  if (mux == 1) {count++;}
  if (count > 2) {count = 0;}
  ADCSRA |= 1<<ADSC;                                    // Re-start Conversion
  switch (bearing) {
    case 0: 
      soundHeading = 0;
    break;
    case 1: 
      soundHeading = 270;
    break;
    case 2: 
      soundHeading = 330;
    break;
    case 3: 
      soundHeading = 30;
    break;
    case 4: 
      soundHeading = 90;
    break;
    case 5: 
      soundHeading = 150;
    break;
    case 6: 
      soundHeading = 210;
    break;
    default: 
      soundHeading = 0;
    break;
  }
}
  
void setup()
{
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  Serial.begin(115200);
  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
 
  //Setup for the ADC
  ADCSRA = 0b10001100;  //0x8F; //Verified
                        //bit 7 = ADC Enable
                        //bit 6 = ADC Start Conversion
                        //bit 5 = ADC Auto Trigger Enable
                        //bit 4 = ADC interrupt flag
                        //bit 3 = adc interrupt enable
                        //bit 2:0 = ADC Prescaler Select Bits
  ADCSRB = 0b00001000;
  ADMUX = ADMUX_A;   //0xE5;  //Verified
                      //bit 7:6 = Reference voltage set to 2.56
                        //bit 5 = ADLAR, justification of results in ADC output register, 0 right, 1 left
                        //bit 4:0 = MUX for ADC input source
    
  sei(); // enable interrupts 
  
  ADCSRA |= 1<<ADSC;  //Start ADC conversion

  // Set up Encoder / pids
  Encoder_1.setPulse(9);
  Encoder_2.setPulse(9);
  Encoder_1.setRatio(39.267);
  Encoder_2.setRatio(39.267);
  Encoder_1.setPosPid(1.8,0,1.2); // 1.8 0 1.2 orig
  Encoder_2.setPosPid(1.8,0,1.2);
  Encoder_1.setSpeedPid(.18,0.0,0); // 0.18 0 0 orig
  Encoder_2.setSpeedPid(.18,0.0,0); // 0.18
  
  xTaskCreate(
    TaskAnalogRead
    , (const portCHAR *) "AnalogRead"
    , 128 // This stack size can be checked & adjusted by reading Highwater
    , NULL
    , 1 // priority
    , NULL );
    // Now the task scheduler, which takes over control of scheduling
    // individual tasks, is automatically started.
   xTaskCreate(
      TaskMotion
      , (const portCHAR *)"Motion" // A name just for humans
      , 128 // Stack size
      , NULL
      , 1 // priority
      , NULL );

   xTaskCreate(
      TaskController
      , (const portCHAR *)"Controller"
      , 128 // Stack size
      , NULL
      , 1 // priority
      , NULL );                                             
}
   
void loop()
{
  // Empty. Things are done in Tasks.
}

/*---------------------- Tasks ---------------------*/
// This task need to wait until a valid bearing is determined,
// then rotate toward that bearing and move straight towards it
// for a reasonable duration and speed (Move 2 ft towards sound)
void TaskMotion(void *pvParameters) 
{
   (void) pvParameters;
   for (;;) // A Task shall never return or exit.
   {
      // Need to move, valid sound sensed, probably switch-case for cleaner code
      if(marcoMotionState == IN_PURSUIT_TURNING )
      {
        // But do this so we don't turn more than 180 degs :)
        int degToRotate = DEGREES_NEEDED_FOR_ROTATION * (soundHeading / 360.0);
        // 90 deg rotation
        Encoder_1.moveTo(degToRotate, 150);
        Encoder_2.moveTo(degToRotate, 150);
        if(abs(Encoder_1.getCurPos()-degToRotate) < 15 && abs(Encoder_2.getCurPos()-degToRotate) < 15)
        {
          // Finished turning, now move forward
          marcoMotionState = IN_PURSUIT_CHASING; 
          Encoder_1.setPulsePos(0);
          Encoder_2.setPulsePos(0); 
        }        
      }
      else if (marcoMotionState == IN_PURSUIT_CHASING)
      {
        // temps for testing
        double inchesToMove = 24.0;
        double degToMove = (inchesToMove/INCHES_PER_TURN)*360;

        // Move 2 ft forward toward sound
        Encoder_1.moveTo(-degToMove, 150);
        Encoder_2.moveTo(degToMove, 150);
        if(abs(Encoder_1.getCurPos()+degToMove) < 15 && abs(Encoder_2.getCurPos()-degToMove) < 15)
        {
          // Finished turning, now move forward
          marcoMotionState = WAITING_FOR_HEADING; 
          Encoder_1.setPulsePos(0);
          Encoder_2.setPulsePos(0); 
        }         
      }
      else // WAITING_FOR_HEADING Stay Motionless
      {
        no_direction = 1; //Set the no_direction flag so that we can get a bearing from the sensors
        Encoder_1.setPulsePos(0);
        Encoder_2.setPulsePos(0);  
        Encoder_1.moveTo(0, 128);
        Encoder_2.moveTo(0, 128);
        if(!no_direction) {marcoMotionState = IN_PURSUIT_TURNING;}  //if the ADC ISR has cleared the no_direction flag, then we have a new bearing, so we want to change to the next state to start turning
      }      
      Encoder_1.loop();
      Encoder_2.loop();
      vTaskDelay(1);     
   }
}

// This task needs to read the ADC of multiple sound sensors
// and calculate the heading towards the sound source.
// 0 deg = forward -> increasing clockwise until 359
// Store in global var int soundHeading ?
void TaskAnalogRead(void *pvParameters) // This is a task.
{
  (void) pvParameters;
  // initialize serial communication at 9600 bits per second:
  for (;;)
  {
    // read the input on analog pin 1:
    int soundValue = analogRead(A1); 
    // Set up ADC "For real" via something like below in setup()
    // in conjunction with ISR(ADC_vect)
    //  ADMUX &= 0b11011111;    // right adjust by clearing ADLAR 
    //  ADMUX |= 0b01000000;    // set normal reference AVCC
    //  ADMUX |= 0b00000001;    // ADC 1 for sound sensor read in
    //  ADCSRA |= (1 << ADEN);  // ADC enable
    //  ADCSRA |= (1 << ADATE); // ADC auto trigger enable
    //  ADCSRA |= (1 << ADIE);  // ADC interrupt enable
    //  ADCSRA |= 0b00000111;   // 128 prescalar for 125 khz adc (maybe we want SPEEED for TDOA sound locating?)
    //  ADCSRB &= 0b11111000;   // ADTS bits set to Free Running ADC Auto Trigger Source
    //  ADCSRA |= (1 << ADSC); // ADC Start conversion
    if(soundValue > 300 && ((millis()-clapTime) > 5000)) // clap and at least 5 seconds since last clap
    {
      if(marcoMotionState == WAITING_FOR_HEADING)
      {
        soundHeading = 180; // default for testing
        marcoMotionState = IN_PURSUIT_TURNING;
      }
      else
      {
        marcoMotionState = WAITING_FOR_HEADING;   
      }
      clapTime = millis();
    }
    
    // print out the value you read:
    vTaskDelay(1); // one tick delay (15ms) in between reads for stability
    // Formatted for use with serialPlotter
    Serial.print(soundValue);
    Serial.print(" ");
    Serial.print(marcoMotionState*100);
    Serial.print(" ");
    Serial.print(-Encoder_1.getCurPos());
    Serial.print(" ");
    Serial.println(Encoder_2.getCurPos());
  }
}

