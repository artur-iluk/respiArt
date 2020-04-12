/* respiArt - by Artur Iluk
 *  Robert Łabuz - added interrupt based motor control
 */

//Robert
#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include "U8glib.h"
#include<string.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);  // I2C / TWI 

#define TIMER_MIN_PERIOD_US 10
unsigned long timerTargetPos;
volatile unsigned long timerCurrentPos;
unsigned long timerUsIncrementTick;
unsigned long timerUsIncrementCounter;
volatile bool timerRun = false;

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
 
// Define stepper motor connections and steps per revolution:
#define dirPin 2           //motor direction pin
#define assistPin 3        //LED assist mode indicator
#define sleepPin 4         //stepper motor sleep pin
#define heaterPin 5        //relay pin to heat humidifier
#define alarmTriggerPin  6 //LED alarm - trigger not triggered
#define alarmVolumePin   7 //LED alarm - set volume not achieved 
#define alarmPressurePin 8 //LED alarm - set pressure not achieved
#define stepPin 9          //step pin, changer from 3 to 9 -> interrupts
#define alarmResetPin 10   //reset all alarm LED to off
#define StartStopPin 11    //start: 0.5s press, stop 2s press


#define freqPin 1 //frequency - breaths per minute - min switch to assist mode SIMPV
#define volPin 2  //volume of single breath
#define pressPin 3 // desired pressure of inspiration [Pa]


//parameters
float trigger_press=25.0;
float temp_set=32.0;
float temp_hysteresis=2.0;
//PID
double Kp=0.3, Ki=0.3, Kd=0.0;

//PID
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
//double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);
long int max_inspiration_press;

boolean alarmVolume=false;    //set volume not achieved
boolean alarmPressure=false;  //set pressure not achieved
boolean alarmTrigger=false;   //trigger in assist mode not triggered
boolean assistMode=false;     // true:  SIMPV mode active- assist vetilation triggered by pressure
                              // false:  

int freq; //frequency of breaths
int vol;  //set volume of breath;
long unsigned int total_cycle_time;
float init_press;
float temp_press;
int inspiration_press;
float temperature;
unsigned long t0;
unsigned long t1;
boolean start_pressed=false;
boolean start_ready=true;
unsigned long start_button_time=0;
boolean active=false;
boolean just_deactivated=false;
unsigned long LED_blink_timer;
boolean LED_blink=false;

char napis1[17]="";
char napis2[17]="";
char napis3[17]="";
int res[130];
int val = 0;  
float f; 
boolean hist=false; //true - display pressure history 
                    //false - display params 

/* distance in steps*/
void timerSetDistance(unsigned long distance)
{
  timerTargetPos = distance;
}

/*increment period by 1us by every x tick, 0 - disable */
void timerSetPeriodIncrement(unsigned long tick)
{
  timerUsIncrementTick = tick;
}

/* set timer period in us*/
void timerSetSpeed(unsigned long period_us)
{
  ICR1 = max(period_us, TIMER_MIN_PERIOD_US);
  if (TCNT1 >= ICR1)
    TCNT1 = 0; // restart counter if period is smaller than current value
}

/* get timer period in us*/
unsigned long timerGetSpeed()
{
  return(ICR1);
}

void timerStop()
{
  TCCR1B = 0; //timer stop;
  timerRun = false;
}

/* timer initialization */
void timerStart()
{
  timerCurrentPos = 0;
  timerUsIncrementCounter = timerUsIncrementTick;
#define TIMER1_A_PIN 9                    //only pin 9 supported
  OCR1A = (TIMER_MIN_PERIOD_US - 1) / 2;  //GPIO high pulse time
  TCCR1B = _BV(WGM13);                    // set mode as phase and frequency correct pwm, stop the timer
  TCCR1A = 0;                             // clear control register A
  TCNT1 = 0;                              // init value
  TIMSK1 = _BV(TOIE1);                    // enable overflow interrupt
  pinMode(TIMER1_A_PIN, OUTPUT);
  TCCR1A |= _BV(COM1A1);                  //link gpio 9 to timer
  TCCR1B |= (1 << CS11);                  // timer start;
  sei();                                  // enable global interrupts
  timerRun = true;
}

void play(int freq, int delay_time){ //freqency 1-3, duration of delay in microseconds
  int f;
  int d;
  int j;
  int k;

  switch (freq) {
    case 1:
      f=25;
      d=26;
      break;
    case 2:
      f=37;
      d=20;
      break;
    default:
      f=57;
      d=17;
      break;
  }
  digitalWrite(sleepPin, HIGH);
  for (j = 0; j < f; j++) {
    digitalWrite(dirPin, LOW);
    timerSetDistance(32);
    timerSetPeriodIncrement(0);
    timerSetSpeed(d);
    timerStart();
    while (timerRun);
    timerStop();

    digitalWrite(dirPin, HIGH);
    timerSetDistance(32);
    timerSetPeriodIncrement(0);
    timerSetSpeed(d);
    timerStart();
    while (timerRun);
    timerStop();
  }
  //digitalWrite(sleepPin, LOW);
  delay(delay_time);
  return;
}

void print_and_buttons() { //print out parameters and check buttons

  //buttons control
  boolean pressed=digitalRead(StartStopPin);
  if (!start_pressed and pressed) { //press detected
    start_button_time=millis();
    start_pressed=true;
  }
  if (start_pressed && start_ready && !active && (millis()-start_button_time>200)) { //start pressed for 0.2s ->start
    napis1[0]='\0';
    napis2[0]='\0';
    napis3[0]='\0';
    strcat(napis2,"   --START--");
    u8g.firstPage();  
    do {
      drawOLED();
    } while( u8g.nextPage() );
    play(1,100);
    play(2,100);
    play(3,100);

    active=true;
    start_ready=false;
    alarmVolume=false;
    alarmPressure=false;
    alarmTrigger=false;
  }
  if (start_pressed && start_ready && active && (millis()-start_button_time>2000)) { //start pressed for 2s ->stop
    active=false;
    start_ready=false;
    just_deactivated=true;
  }
  if (start_pressed && !pressed) { //release detected
    start_pressed=false;
    start_ready=true;
  }
  //alarmReset button
  if (digitalRead(alarmResetPin)) {
    alarmVolume=false;
    alarmPressure=false;
    alarmTrigger=false;
  }
  digitalWrite(assistPin,assistMode);
  if (millis()-LED_blink_timer>500) {
    LED_blink=!LED_blink;
    LED_blink_timer=millis();
  }
  digitalWrite(alarmVolumePin,  LED_blink && alarmVolume);
  digitalWrite(alarmPressurePin,LED_blink && alarmPressure);
  digitalWrite(alarmTriggerPin, LED_blink && alarmTrigger);

  //to print on console 
/*  Serial.print(" active:");
  Serial.print(active);
  Serial.print(" start_pressed:");
  Serial.print(start_pressed);
  Serial.print(" pressed:");
  Serial.print(pressed);
*/
  //TO PLOT  
  Serial.print(temp_press);
  Serial.print(",");
  Serial.print(max_inspiration_press); //temporary speed
  Serial.print(",");
  Serial.print(ICR1); //temporary speed
  Serial.print(",");
  Serial.print(100*alarmVolume+200*alarmPressure+400*alarmTrigger);
  Serial.print(",");
  Serial.println(active*100);

  //to display
  if (!active) {  
    char a[10]="";

    int i=freq/10;
    itoa(i, a, 10);                  
    napis1[0]='\0';
    strcat(napis1,"freq:  ");
    strcat(napis1, a);
    strcat(napis1, " br/min");
    
    i=vol*10;
    itoa(i, a, 10);                  
    napis2[0]='\0';
    strcat(napis2,"vol:  ");
    strcat(napis2, a);
    strcat(napis2, " mL");

    i = max_inspiration_press/100;
    itoa(i, a, 10);                  
    napis3[0]='\0';
    strcat(napis3,"press: ");
    strcat(napis3, a);
    strcat(napis3, " cmH2O");

    u8g.firstPage();  
    do {
      drawOLED();
    } while( u8g.nextPage() );
  }
}

void drawOLED(void) {  //redraw screen
  if (active) {
    int j = max_inspiration_press/100;
    for (int i=0; i<127; i+=2) {
      u8g.drawPixel(i,62-j);
    }  
    for (int i=1; i<127; i++) {
      u8g.drawLine(i-1,62-res[i-1],i,62-res[i]);
    }  
  }
  else {
    u8g.setFont(u8g_font_unifont);
    u8g.drawStr( 0, 20, napis1);
    u8g.drawStr( 0, 40, napis2);
    u8g.drawStr( 0, 60, napis3);
  }
}

//----------------SETUP------------
void setup()
{
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(assistPin, OUTPUT);
  pinMode(sleepPin, OUTPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(alarmVolumePin, OUTPUT);
  pinMode(alarmPressurePin, OUTPUT);
  pinMode(alarmTriggerPin, OUTPUT);
  pinMode(alarmResetPin, INPUT);
  pinMode(StartStopPin, INPUT);
  Serial.begin(115200);
  Serial.println("Initialization.");

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. _X16*/
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
  init_press=bmp.readPressure();

  //display setup
  // flip screen, if required
  u8g.setRot180();
  
  // set SPI backup if required
  //u8g.setHardwareBackup(u8g_backup_avr_spi);

  // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
  napis1[0]='\0';
  napis2[0]='\0';
  napis3[0]='\0';
  strcat(napis1,"  * respiArt *");
  strcat(napis2,"       v4");
  strcat(napis3," by Artur Iluk");
  
  u8g.firstPage();  
  do {
    drawOLED();
  } while( u8g.nextPage() );
  play(1,100);
  play(1,100);  
  digitalWrite(sleepPin, LOW);
  delay(1500);

  digitalWrite(assistPin, HIGH);
  digitalWrite(alarmVolumePin,HIGH);
  digitalWrite(alarmPressurePin,HIGH);
  digitalWrite(alarmTriggerPin,HIGH);
  digitalWrite(assistPin, LOW);
  digitalWrite(alarmVolumePin,LOW);
  digitalWrite(alarmPressurePin,LOW);
  digitalWrite(alarmTriggerPin,LOW);

  //PID
  Input = temp_press;
  Setpoint = max_inspiration_press;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  LED_blink_timer=0;
  //Serial.println("setup end");
}

long int curr_speed = 10;
//------------LOOP---------------------------
void loop()
{
  float max_press=0; //for statistics
  float min_press=0; //for statistics
  boolean trigger=false;
  
  if (!active) { //setting params locked wen active
    
    int potVal = analogRead(freqPin);
    freq =map(potVal,0,1023,98,200);
    if ( freq < 100 ) {
      if (!assistMode) {
        Serial.println("Automatic trigger activated.");
      }
      assistMode=true;
      freq=100;
      digitalWrite(assistPin,HIGH);
    }
    else {
      if (assistMode) {
        Serial.println("Automatic trigger deactivated.");
      }
      assistMode=false;
      digitalWrite(assistPin,LOW);
    }
    
    potVal = analogRead(volPin);
    vol =map(potVal,0,1023,40,85);
    potVal = analogRead(pressPin);
    max_inspiration_press =map(potVal,0,1023,1000,5000);
  
  
    print_and_buttons();
  } //end of inactivity loop

    long int steps=vol*320;
    int step_exp=50;
    int delay_after_ins=500;
    total_cycle_time=600000/freq;
    long int stopped_at=0;
  
//----------activity loop----------------------------------------------------
  if (active) {
    init_press=bmp.readPressure();
    max_press=0;
    min_press=0;
    t0=millis(); //cycle start time
    for (int i=0; i<130; i++) { //clear history
      res[i]=0;
    }
  
    digitalWrite(sleepPin, HIGH);
    digitalWrite(dirPin, HIGH); // Set the spinning direction  - inspiration
    timerSetDistance(320*vol);
    timerSetPeriodIncrement(0);
    stopped_at=0;
  
    int init_ramp_speed=130; //initial speed for start ramp
    int final_ramp_speed=40; //speed after ramp - maximum speed of movement
    timerSetSpeed(init_ramp_speed);
    timerStart();
    for ( int i=init_ramp_speed-1; i>final_ramp_speed; i--) { //start ramp
        timerSetSpeed(i);
        delay(1);
    }
    print_and_buttons();
    t1=millis();
  
    //PID
    //initialize the variables we're linked to
    Input = temp_press;
    Setpoint = max_inspiration_press;
    myPID.SetSampleTime(1);
  
    //start controlled compression
  
    int prev_speed=timerGetSpeed();
    
    while(timerRun){
      temp_press=bmp.readPressure()-init_press;
      Input = temp_press;
      myPID.Compute();
      curr_speed=Output*10+final_ramp_speed;    
  
      //limiting accelerations
      if (curr_speed > 10*prev_speed) {
        curr_speed = 10*prev_speed;
      }
      else if (curr_speed < prev_speed/10) {
        curr_speed = prev_speed/10;
      }
  
      timerSetSpeed(curr_speed);
  
      //-------------------------------delay(10);
  
      if ( millis()-t1 > 100) { //plot press 10Hz
        t1=millis();
        print_and_buttons();
      }
      
      //statistics
      if (temp_press > max_press){
        max_press=temp_press;
      }
      if (temp_press < min_press){
        min_press=temp_press;
      }
  
      //update history
      int i=total_cycle_time/128; //time of single line on draw
      int j=(millis()-t0)/i;
      if (temp_press>res[j] ) {
        res[j]=temp_press/100; //vertical scale, 1pix=100Pa
      }
      if (millis()-t0 > total_cycle_time-delay_after_ins-500 ) {//500 expiration??
        stopped_at=timerCurrentPos;
        timerStop();
        alarmVolume=true;
      }
    }
    
    timerStop();
    if (max_press < max_inspiration_press) {
      alarmPressure=true;
    }
  
    //Serial.print("Delay_after_inspiration...");
  
    t1=millis();
    while ( millis()-t1<delay_after_ins ) { 
      temp_press=bmp.readPressure()-init_press;
      print_and_buttons();
      delay(100);
    }
  
    //Serial.println("...Done.");
  
    //Serial.println("Expiration");
    // Set the spinning direction  - expiration:
    digitalWrite(dirPin, LOW);
    digitalWrite(sleepPin, HIGH);
    if (!stopped_at){
      timerSetDistance(320*vol);
    }
    else {
      timerSetDistance(stopped_at);
    }
    timerSetPeriodIncrement(0);
    timerStart();
    for ( int i=init_ramp_speed-1; i>final_ramp_speed; i--) { //start exp ramp
      delay(1);
      timerSetSpeed(i);
    }
    print_and_buttons();
    while (timerRun) {
      temp_press=bmp.readPressure()-init_press;
      print_and_buttons();
      delay(50);
      if (timerTargetPos-timerCurrentPos < 3200){
        timerSetSpeed(timerGetSpeed()+5);
      }
    }
    timerStop();
    digitalWrite(sleepPin, LOW); //sleep motor driver to safe power and avoid overheat
    
    temperature=bmp.readTemperature();
    //Serial.print("Temperature: ");
    //Serial.print(temperature);
    //Serial.print("C. ");
    if ( temperature < temp_set-temp_hysteresis ) {
      digitalWrite(heaterPin, HIGH); //turn on heater
      //Serial.print("Heating...");
    }
    //Serial.println("");
        
    //delay=total_cycle_time or pressure trigger
    //Serial.print("Delay_after_expiration...");
    t1=millis();
    while ( millis()-t1<500 ) { 
      temp_press=bmp.readPressure()-init_press;
      print_and_buttons();
      delay(50);
    }
  
    init_press=bmp.readPressure();
  
    trigger=false;
    while ( (millis() - t0 < total_cycle_time) && !trigger ) {
      
      temp_press=bmp.readPressure()-init_press;
      print_and_buttons();
      delay(50);
      if (temp_press > max_press){
        max_press=temp_press;
      }
      if (temp_press < min_press){
        min_press=temp_press;
      }
  
      if ((-min_press > trigger_press) && assistMode) {
        t0=millis()-total_cycle_time;
        //Serial.print("Trigger!");
        trigger=true;
        play(3,0);
      }
    }
    if (assistMode && !trigger) {
      alarmTrigger=true;
      play(3,100);
      play(1,100);
      //play(3,300);
    }
    digitalWrite(heaterPin, LOW); //turn off heater
  } //end of activity loop

  //draw pressure history of last cycle
  u8g.firstPage();  
  do {
    drawOLED();
  } while( u8g.nextPage() );

  if (just_deactivated) {
    just_deactivated=false;
    napis1[0]='\0';
    napis2[0]='\0';
    napis3[0]='\0';
    strcat(napis2,"   --STOP--");
    u8g.firstPage();  
    do {
      drawOLED();
    } while( u8g.nextPage() );
    play(3,100);
    play(2,100);
    play(1,300);
    digitalWrite(sleepPin, LOW);
  }
} 
  
ISR(TIMER1_OVF_vect)
{
  timerCurrentPos++;
  if (timerCurrentPos > timerTargetPos)
  {
    timerStop();
  }
  if (timerUsIncrementTick == 0)
    return;
  timerUsIncrementCounter--;
  if (timerUsIncrementCounter == 0)
  {
    timerUsIncrementCounter = timerUsIncrementTick;
    ICR1++; //increase period by 1us
  }
}
