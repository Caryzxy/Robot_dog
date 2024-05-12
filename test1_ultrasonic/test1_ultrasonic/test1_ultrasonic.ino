/*
 * test1_MPU6050
 * Robot prints out acceleration and gryo readings from MPU6050, as
 * well as their averages.
 * You need to restart the program (press the reset button on the Arduino
 * board) to restart the averaging.
 * 
 * Need to have "I2C dev" folder in Arduino libraries
 * Need to have "MPU6050" folder in Arduino libraries
 *
 */
/*********************************************************************/

// #include "I2Cdev.h"
#include "MPU6050.h"

/*********************************************************************/

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// Current accelerometer and gyro zero values.
int ax0 = 0;
int ay0 = 0;
// adjustment of az is not useful.
int gx0 = 0;
int gy0 = 0;
int gz0 = 0;

#define N_AVERAGES 6
double sums[N_AVERAGES];
unsigned long count = 0;
unsigned int loop_count = 0;

int measure_flag = 1;
float measure_prev_time = 0;

#define ECHO_PIN A3
#define TRIG_PIN 11
#define RECV_PIN 9
#define IR_SEND_PIN 9
#define LEFT_RECEIVE_PIN A0
#define RIGHT_RECEIVE_PIN A1

volatile bool left = false;
volatile bool right = false;
double distance = 0;
float get_distance_prev_time = 0;

/**********************************************************************
Drive the system with sinusoids
Print out data concurrently
What sampling rate can we achieve?
/**********************************************************************/

#include "Pins.h"
#include "My_encoders.h"
#include <Encoder.h>

/**********************************************************************/

#define MAX_ULONG 0xFFFFFFFF

#define SERVO_INTERVAL 2000 // microseconds

// start data collection after N milliseconds from go 
#define START_COLLECT  4900 // milliseconds
#define DEBUG_PRINT_INTERVAL 500 // milliseconds, 

#define MAX_COMMAND 255

// at x/sample should take 1s to get to 1.0
#define SAFE_STARTUP_INCREMENT (SERVO_INTERVAL*1e-6)

/**********************************************************************/

bool go = false;  /* motor enable */

// Encoder readings: these can be positive or negative.
long left_angle = 0;
long right_angle = 0;

// used to keep track of time intervals in servo
unsigned long last_micros;
unsigned long last_millis;

// Servo clock
int servo_time = 0; // needs to be signed, can be int
// how long have we been running? Safety feature
unsigned long run_time = 0; // milliseconds
// Clocks for printing and other stuff
unsigned long debug_print_time = 0; // milliseconds
  
// Keep track of late control cycles. 
unsigned long servo_late = 0; // How many times am I late?
unsigned long max_servo_late = 0; // What was the worst one?

float current_distance = 0;

/**********************************************************************/
/**********************************************************************/

void motor_init()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
}

void motor_stop()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void motor_left_command( int speed )
{
    if ( speed >= 0 )
    {
      digitalWrite( AIN1, 1 );
      analogWrite( PWMA_LEFT, speed );
    }
  else
    {
      digitalWrite( AIN1, 0 );
      analogWrite( PWMA_LEFT, -speed );
    }
}

// reverses the sign of "speed"
void motor_right_command( int speed )
{
  if ( speed >= 0 )
    {
      digitalWrite( BIN1, 1 );
      analogWrite( PWMB_RIGHT, speed );
    }
  else
    {
      digitalWrite( BIN1, 0 );
      analogWrite( PWMB_RIGHT, -speed );
    }
}

/**********************************************************************/

void setup()
{

  Serial.begin( 1000000 );
  // Serial.begin(115200);
  analogReference(INTERNAL1V1); // was INTERNAL on 328P, voltage
  while( !Serial );
  Serial.println( "sinusoids4 version 1" );
  delay(1000); // Delay to make sure above message prints out.

  motor_init();
  Serial.println( "motor_init done." );
  delay(1000); // Delay to make sure above message prints out.

  Encoder_init( &left_angle, &right_angle );
  Serial.println( "Initialized encoders" );
  Serial.println( "Wheels should be off the ground."  );
  Serial.println( "Type g <return> to run test, s <return> to stop."  );
  Serial.println( "Typing window is at the top of the Arduino serial monitor window." );
  Serial.println( "Type into the main window of a Putty serial monitor window." );
  accelgyro.initialize();
  ultrasonicInit();
  go = false; // start turned off

  // zero everything
  servo_time = 0;
  debug_print_time = 0;
  servo_late = 0;
  max_servo_late = 0;

  // check how rounding works
  /*
  int a = 1.4 + 0.5;
  int aa  = 1.6 + 0.5;
  int b = -1.4 - 0.5;
  int c = -1.6 - 0.5;
  Serial.print( "test: " );
  Serial.print( a );
  Serial.print( " " );
  Serial.print( aa );
  Serial.print( " " );
  Serial.print( b );
  Serial.print( " " );
  Serial.println( c );
  */
}

/******************************************************************/

// Take user input
void ProcessCommand()
{

  if ( Serial.available() <= 0 )
    return;

  int c = Serial.read();
  switch (c)
    {
      case 'S': case 's':
        Serial.println( "Stop!" );
        go = false;
        break;
      case 'G': case 'g':
      Serial.println( "Go!" );
        go = true;
        break;
      default:
        break;
    }
}

/**********************************************************************/

void apply_sinusoid( float freq, unsigned long run_time_limit,
     		     float max_command )
{
  int millis_increment = 0;
  unsigned long run_time = 0;
  run_time_limit += START_COLLECT + 1000;
  debug_print_time = 0;
  servo_time = 0;
  last_micros = micros();
  last_millis = millis();
  Serial.print( "Max command: " );
  Serial.println( max_command );
  Serial.print( "Frequency: " );
  Serial.println( freq );
  int voltage_raw = analogRead(VOL_MEASURE_PIN); //Read voltage value
  double voltage = (voltage_raw*1.1/1024)*((10+1.5)/1.5);
  Serial.print("Current voltage ");
  Serial.print( voltage );
  Serial.print(" ");
  Serial.println( voltage_raw );
  int started = 0;
  float safe_on_off = 0;
  if ( max_command > MAX_COMMAND )
    max_command = MAX_COMMAND;
  unsigned long real_run_time_limit = run_time_limit + 1000; // add 1 second

  for( ; ; )
    {
      unsigned long current_micros = micros();
      if ( current_micros > last_micros )
        servo_time += current_micros - last_micros;
      else // rollover
        servo_time += (MAX_ULONG - last_micros) + current_micros;
      last_micros = current_micros;

      // It isn't time yet ...
      if ( servo_time < SERVO_INTERVAL )
        continue;

      // It is time! Reset the servo clock.
      servo_time -= SERVO_INTERVAL;

      // We are a little late.
      if ( servo_time > 0 )
        if ( max_servo_late < servo_time )
          max_servo_late = servo_time;

      // Let's have some slop in counting late cycles.
      if ( servo_time > 50 )
        servo_late += 1;
    
      // handle the millisecond clocks
      unsigned long current_millis = millis();
      if ( current_millis > last_millis )
        millis_increment = current_millis - last_millis;
      else // rollover
        millis_increment = (MAX_ULONG - last_millis) + current_millis;
      last_millis = current_millis;
  
      run_time += millis_increment;
      debug_print_time += millis_increment;

      // Read the sensors
      Read_encoders( &left_angle, &right_angle );

      if ( safe_on_off < (1.0 - SAFE_STARTUP_INCREMENT) )
        safe_on_off += SAFE_STARTUP_INCREMENT;
      float commandf = safe_on_off*max_command*
                               sin( 0.000002*M_PI*freq*current_micros );
        int command = round( commandf );

      if ( ( run_time >= START_COLLECT ) && ( run_time <= run_time_limit ) )
        {
          if ( !started )
            {
              servo_late = 0;
              max_servo_late = 0;
              started = 1;
              Serial.println( "Data" );
            }
          Serial.print( current_micros );
          Serial.print( " " );
          Serial.print( left_angle );
          Serial.print( " " );
          Serial.print( right_angle );
          Serial.print( " " );
          Serial.println( command );
        }

      if ( ( debug_print_time > DEBUG_PRINT_INTERVAL ) && ( run_time < START_COLLECT ) )
        {
/*
       Serial.print( ((float) current_millis)/1000.0 );
       Serial.print( " " );
       Serial.print( run_time );
       Serial.print( " " );
       Serial.print( servo_late );
       Serial.print( " " );
       Serial.print( max_servo_late );
       Serial.print( " " );
       Serial.print( left_angle );
       Serial.print( " " );
       Serial.print( right_angle );
       Serial.print( " " );
       Serial.print( left_command );
       Serial.print( " " );
       Serial.println( right_command );
       max_servo_late = 0;
*/
       Serial.print( servo_late );
       Serial.print( " " );
       Serial.println( max_servo_late );
       ProcessCommand();

         debug_print_time -= DEBUG_PRINT_INTERVAL;
       }

      if ( run_time > run_time_limit )
        safe_on_off *= 0.95; // gradually turn motor off

      // Turn off after a while to keep from running forever
      if ( ( run_time > run_time_limit ) || ( !go ) )
        {
          motor_stop();
          Serial.println( "Motor stopped." );
          Serial.println( "Wheels should be off the ground."  );
          Serial.println( "Type g <return> to run test, s <return> to stop."  );
          return;
        }
    
      motor_left_command( command );
      motor_right_command( command );
    }
}
void ultrasonicInit()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(IR_SEND_PIN, OUTPUT);
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_RECEIVE_PIN), left_recieve, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_RECEIVE_PIN), right_recieve, FALLING);
}

void left_recieve() {
  left = true;
}
void right_recieve() {
  right = true;
}

void measureDistance()
{
  if (measure_flag == 0)
  {
    measure_prev_time = micros();
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), measureDistance, FALLING);
    measure_flag = 1;
  }
  else if (measure_flag == 1)
  {
    distance = (micros() - measure_prev_time) * 0.017; //343 m/s * (10^-6 s / us)/ 2 = 0.017
    measure_flag = 2;
    Serial.println(distance);
    current_distance = distance;
  }
}

void getDistance()
{
  if (millis() - get_distance_prev_time > 50)
  {
    get_distance_prev_time = millis();
    measure_flag = 0;
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), measureDistance, RISING);
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  }
}


/*********************************************************************/
/*
void setup()
{
  Wire.begin();
  Serial.begin(1000000);


  Serial.println("HERE");

  while( !Serial );
  Serial.println( "test1_MPU6050 version 24" );
  Serial.println( "Printing out accelerometer (a) and gyro (g) readings and averages." );
  Serial.println( "If I hang here, the MPU is in a bad state. Power cycle the robot." );
  delay(1000); // take time to allow above messages to print.
  accelgyro.initialize();
  Serial.println( "Initialized" );
  for ( int i = 0; i < N_AVERAGES; i++ )
    sums[i] = 0;
  delay(1000); // take time to allow above messages to print.

  ultrasonicInit();
}
*/
/*********************************************************************/
static long target_distance_close =70;
static long target_distance_far = 80;
void loop()
{
  
  if ( !go )
    {
      motor_stop();
      delay(500);
      ProcessCommand();
      return;
    }
  getDistance();

  if (current_distance < target_distance_close)
  {
    motor_left_command(50);
    motor_right_command(50);
  }
  else if(current_distance > target_distance_far)
  {
    motor_left_command(-50);
    motor_right_command(-50);
  }
  else
  {
    motor_left_command(0);
    motor_right_command(0);
  }
  Serial.println(current_distance);

  if(millis() > 60000)
  {
    go = false;
    Serial.println("Stop");
  }

  delay(50);
}