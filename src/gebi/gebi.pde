/*
   Copyright 2009 Alex Gourley

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include <avr/sleep.h>

// PINS
// pins 0-5 reserved for sensors. See 'numSensors' constant determine how many are activated.
const int speedPotPin = 5;    //connected to speed control potentiometer
const int outputPin = 6;    // To Game Controller
const int ledStartPin = 9; //pins 9, 10, 11, 12, 13 reserved for speedometer LEDs, see 'numLeds' constant
const int numLeds = 5;
const int numSensors = 5;
const int wakePin = 2;                 // pin used for waking up

const boolean debugSerial = false;
const boolean debugLed = false;
const boolean sleepEnabled = false;

const float maxLedIntensity = 0.95;
const float intensityPerLed =  maxLedIntensity / numLeds;

// speedPot constants assuming 3.3 volts vcc and 
// assuming the sensor pin on the pot goes from 0.0->2.99 volts
// 2.99/3.30 * 1023 = 927
const int speedPotHigh = 927;
const int speedPotLow = 0;

// The value (between 0->1023) that constitutes the hall effect sensors
// going high.
// 3.3 volt VCC:
//   2.10 volts normal     2.1/3.3 * 1023 = 651
//   2.50 volts w/ magnet  2.5/3.3 * 1023 = 775
const int sensorHighTrigger = 630;

// 255 is the max duty cycle for analog out. The xbox 360 pad I use
// expects 0.2 volts normally and 1.2 volts when the trigger is 
// depressed fully. 
const int outputHigh = 93;    // 1.2/3.3 * 255 = 93
const int outputLow = 15;     // 0.2/3.3 * 255 = 15

const long sleepMicros = 5000000; //microseconds after last activity until chip goes to sleep.
//state variables
long microsBetweenSensorsForMax = 0; // Microseconds between sensors which will result in max speed
int lastSensor = 0;                  // Last valid sensor that lit up (0 for none)
long lastTime = 0;                   // Last time the sensor lit up (in micros since start of program)
long lastDelta = 0;                  // Time between the last two sensor hits
long lastPrintTime = 0;              // Used by some debug code

long lastActivityTime = 0;             // set each time we detect interaction with the device

// Calculate how many microseconds between sensors will constitute full speed. 
long microsBetweenSensors(float rotationsPerSec){
  // Time measurements are done with microseconds, there are 1000000
  // microseconds in a second. 
  return (1000000.0)/(float(numSensors) * rotationsPerSec);
}

// Poll speed potentiometer
float readPot(){
  int potReading = analogRead(speedPotPin);
  
  // Debug
  //Serial.print("pot reading: ");
  //Serial.println(potReading, DEC);
  
  //convert voltage to 0-1.0
  return float(potReading) / float(speedPotHigh);
}

// Pass in [0.0, 1.0] to specify max pedal speed. 
void setMaxSpeed(float level){
  microsBetweenSensorsForMax = microsBetweenSensors(3.0 * level + 1.0);
  // Debug
  //Serial.print("msForMax: ");Serial.print(microsBetweenSensorsForMax/1000, DEC);
  //Serial.print(" usForMax: ");Serial.print(microsBetweenSensorsForMax, DEC);
  //Serial.println("");
}

// Get the previous sensor number given forward pedaling
int previousSensor(int sensor){
  if(sensor == 0){
    return numSensors-1;
  } else return sensor-1;
}

boolean isTriggered(int sensor){
  return analogRead(sensor) >= sensorHighTrigger;
}

void clearOutput(){
  for(int i = 0; i < numLeds; i++){
    digitalWrite(i + ledStartPin, LOW);
  }
  analogWrite(outputPin, 0);
}

// Light up a single LED, useful for debug
void onlyLight(int ledNum){
  for(int i = 0; i < numLeds; i++){
    if(i == ledNum)
        digitalWrite(i + ledStartPin, HIGH);
    else
        digitalWrite(i + ledStartPin, LOW);
  }
}

void prod(){
  lastActivityTime = micros();
}

void checkSensor(int sensor){
  
  if(lastSensor == sensor){
    return; //only check the sensor if it wasn't the last one we were on
  }
  if(lastSensor == previousSensor(sensor) && isTriggered(sensor)) {

    if(debugLed) {
      onlyLight(sensor);
    }
    if(debugSerial) {
      Serial.print("Sensor triggered in order: "); Serial.println(sensor, DEC);
    }
    if(sleepEnabled){
      prod(); //keep awake
    }
    
    //record the time between sensor reads
    long thisTime = micros();
    lastDelta = thisTime - lastTime;
    lastTime = thisTime;
    lastSensor = sensor;
  }
}

// Light up 0 to 5 lights depending on intensity of pedaling of value [0.0, 1.0]
// Light up highest pin number in the case of the lowest intensity (because of how board was wired)
void lightSpeedLeds(float intensity) {
  int numToLight = intensity / intensityPerLed;
  for(int i = 0; i < numLeds; i++){
    if( i < numToLight ) {
      digitalWrite(ledStartPin + numLeds - i - 1, HIGH);
    }
    else {
      digitalWrite(ledStartPin + numLeds - i - 1, LOW);
    }
  }
}


void setup()  // run once, when the sketch starts
{
  
   CLKPR = (1<<CLKPCE);
   CLKPR = B00000001; // Divide by 2
   
  if(sleepEnabled){
    pinMode(wakePin, INPUT);
    attachInterrupt(0, wakeUpNow, HIGH); // use interrupt 0 (pin 2) and run function
                                      // wakeUpNow when pin 2 gets HIGH 
  }
  lastPrintTime = micros();
  
  readPot();

  //TODO try this with 2,147,483,647 which is the max a long can hold.
  lastDelta = microsBetweenSensorsForMax * 10; // set our current pedalling speed to something very slow. 
  
  for(int i = 0; i < numSensors; i++) {
    pinMode(i+ledStartPin, OUTPUT);
  }
  pinMode(outputPin, OUTPUT);
  pinMode(speedPotPin, INPUT);
  if(debugSerial) {
    Serial.begin(9600);
  } 
}

void loop()
{
  setMaxSpeed(readPot());
  
  // check sensors. Slightly inefficient to read them all instead of just the next,
  // although some later debug code assumes this bahavior. 
  for(int sensor = 0; sensor < numSensors; sensor++) {
    checkSensor(sensor);
  }
  // The lastTime variable is the time at which we hit the previous sensor. Set the voltage to
  // the level we would want if we JUST were to hit the next sensor BUT no higher than the 
  // speed indicated by the last two sensor hits. 
  long thisTime = micros();
  
  if(sleepEnabled && (thisTime - lastActivityTime > sleepMicros)) {
    if(debugSerial){
      Serial.println("Timer: Entering Sleep mode");
      delay(100);     // this delay is needed, the sleep 
                      //function will provoke a Serial error otherwise!!
    }
    sleepNow();
    
  }
  
  float delta = max((float(thisTime-lastTime)), float(lastDelta));
  
  float intensity = min(microsBetweenSensorsForMax/delta, 1.0);//cap intensity at 1.0
 
  // TODO store this information alongside the consts in some kind of object. 
  // Wireless xbox 360 controller formula for determining output voltage.
  byte out = outputLow + (outputHigh-outputLow)*intensity;
  // Wired xbox 360 controller formula (normally high, trigger press goes low)
  //byte out = outputHigh - (intensity * (outputHigh-outputLow));
  analogWrite(outputPin, out);
  if(!debugLed) {
    lightSpeedLeds(intensity);
  }
  if(debugSerial){


    if(thisTime > (1000000 + lastPrintTime))
    {
      lastPrintTime = thisTime;
      Serial.print("output: "); Serial.print(out, DEC); Serial.print(" intensity: ");Serial.print(intensity*100.0, DEC);Serial.print(" lastDelta: ");Serial.print(lastDelta, DEC);
      Serial.print(" microsBetweenSensorsForMax: "); Serial.print(microsBetweenSensorsForMax, DEC);
      Serial.print(" 1: "); Serial.print(analogRead(0), DEC);
      Serial.print(" 2: "); Serial.print(analogRead(1), DEC);
      Serial.print(" 3: "); Serial.print(analogRead(2), DEC);
      Serial.print(" 4: "); Serial.print(analogRead(3), DEC);
      Serial.print(" 5: "); Serial.print(analogRead(4), DEC);
     
       
      Serial.println("");
    }
  
    if (Serial.available() > 0) {
  	byte incomingByte = byte(Serial.read());
  
  	Serial.print("I received: ");
  	Serial.println(incomingByte, DEC);
        analogWrite(outputPin, (incomingByte-48) * 5);
    }

  }
}






void wakeUpNow()        // here the interrupt is handled after wakeup
{
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
}


void sleepNow()         // here we put the arduino to sleep
{
  clearOutput();
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and 
     * wake up sources are available in which sleep mode.
     *
     * In the avr/sleep.h file, the call names of these sleep modes are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings 
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we 
     * choose the according 
     * sleep mode: SLEEP_MODE_PWR_DOWN
     * 
     */  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin 

    /* Now it is time to enable an interrupt. We do it here so an 
     * accidentally pushed interrupt button doesn't interrupt 
     * our running program. if you want to be able to run 
     * interrupt code besides the sleep function, place it in 
     * setup() for example.
     * 
     * In the function call attachInterrupt(A, B, C)
     * A   can be either 0 or 1 for interrupts on pin 2 or 3.   
     * 
     * B   Name of a function you want to execute at interrupt for A.
     *
     * C   Trigger mode of the interrupt pin. can be:
     *             LOW        a low level triggers
     *             CHANGE     a change in level triggers
     *             RISING     a rising edge of a level triggers
     *             FALLING    a falling edge of a level triggers
     *
     * In all but the IDLE sleep modes only LOW can be used.
     */

    attachInterrupt(0, wakeUpNow, HIGH); // use interrupt 0 (pin 2) and run function
                                       // wakeUpNow when pin 2 gets HIGH 

    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

    sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
    detachInterrupt(0);      // disables interrupt 0 on pin 2 so the 
                             // wakeUpNow code will not be executed 
                             // during normal running time.
    prod(); //reset sleep timer
    if(debugSerial){
      Serial.println("Waking up from interrupt.");
    }    
}


