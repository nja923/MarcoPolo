
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
  
  sei(); // enable interrupts 

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
        Encoder_1.setPulsePos(0);
        Encoder_2.setPulsePos(0);  
        Encoder_1.moveTo(0, 128);
        Encoder_2.moveTo(0, 128);
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

