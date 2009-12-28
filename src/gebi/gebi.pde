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


// PINS
// pins 0-5 reserved for sensors. See 'numSensors' constant determine how many are activated.
const int speedPotPin = 5;    //connected to speed control potentiometer
const int outputPin = 6;    // To Game Controller
const int ledStartPin = 9; //pins 9, 10, 11, 12, 13 reserved for speedometer LEDs, see 'numLeds' constant
const int numSensors = 5;
const int numLeds = 5;
const boolean debug = true;
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
const int sensorHighTrigger = 700;

// 255 is the max duty cycle for analog out. The xbox 360 pad I use
// expects 0.2 volts normally and 1.2 volts when the trigger is 
// depressed fully. 
const int outputHigh = 93;    // 1.2/3.3 * 255 = 93
const int outputLow = 15;     // 0.2/3.3 * 255 = 15

//state variables
long microsBetweenSensorsForMax = 0; // Microseconds between sensors which will result in max speed
int lastSensor = 0;                  // Last valid sensor that lit up (0 for none)
long lastTime = 0;                   // Last time the sensor lit up (in micros since start of program)
long lastDelta = 0;                  // Time between the last two sensor hits
long lastPrintTime = 0;              // Used by some debug code

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
  microsBetweenSensorsForMax = microsBetweenSensors(3.0 * level + 2.0);
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

// Light up a single LED, useful for debug
void onlyLight(int ledNum){
  for(int i = 0; i < numLeds; i++){
    if(i == ledNum)
        analogWrite(i + ledStartPin, 255);
    else
        analogWrite(i + ledStartPin, 0);
  }
}

void checkSensor(int sensor){
  
  if(lastSensor == sensor){
    return; //only check the sensor if it wasn't the last one we were on
  }
  if(lastSensor == previousSensor(sensor) && isTriggered(sensor)) {

    //onlyLight(sensor);
    //record the time between sensor reads
    long thisTime = micros();
    lastDelta = thisTime - lastTime;
    lastTime = thisTime;
    lastSensor = sensor;
  }
}

// Light up 0 to 5 lights depending on intensity of pedaling of value [0.0, 1.0]
void lightSpeedLeds(float intensity) {
  int num = intensity / intensityPerLed;
  for(int i = 0; i < numLeds; i++){
      if( i < num ) {
        digitalWrite(i + ledStartPin, HIGH);
        //Serial.print("H ");
      }
      else {
        digitalWrite(i + ledStartPin, LOW);
        //Serial.print("L ");
      }

  }
  //Serial.println("");
}

void setup()  // run once, when the sketch starts
{
  readPot();

  //TODO try this with 2,147,483,647 which is the max a long can hold.
  lastDelta = microsBetweenSensorsForMax * 5; // set out current pedalling speed to something very slow. 
  
  for(int i = 0; i < numSensors; i++) {
    pinMode(i+ledStartPin, OUTPUT);
  }
  pinMode(outputPin, OUTPUT);
  pinMode(speedPotPin, INPUT);
  //Serial.begin(9600); 
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
  float delta = max((float(thisTime-lastTime)), float(lastDelta));
  
  float intensity = min(microsBetweenSensorsForMax/delta, 1.0);//cap intensity at 1.0
 
  // TODO store this information alongside the consts in some kind of object. 
  // Wireless xbox 360 controller formula for determining output voltage.
  byte out = outputLow + (outputHigh-outputLow)*intensity;
  // Wired xbox 360 controller formula (normally high, trigger press goes low)
  //byte out = outputHigh - (intensity * (outputHigh-outputLow));
  analogWrite(outputPin, out);
  lightSpeedLeds(intensity);
   
  // Debug
  //Serial.print("output: "); Serial.print(out, DEC); Serial.print(" intensity: ");Serial.print(intensity*100.0, DEC);Serial.print(" lastDelta: ");Serial.print(lastDelta, DEC);
  //Serial.print(" 1: "); Serial.print(analogRead(0), DEC);
  //Serial.print(" 2: "); Serial.print(analogRead(1), DEC);
  //Serial.print(" 3: "); Serial.print(analogRead(2), DEC);
  //Serial.print(" 4: "); Serial.print(analogRead(3), DEC);
  //Serial.print(" 5: "); Serial.print(analogRead(4), DEC);
  //Serial.println("");

  // Debug 
  //if(thisTime > (500000 + lastPrintTime))
  //{
  //  lastPrintTime = thisTime;
  //}
  //digitalWrite(13, HIGH);  
  if (Serial.available() > 0) {
		// read the incoming byte:
		byte incomingByte = byte(Serial.read());

		// say what you got:
		Serial.print("I received: ");
		Serial.println(incomingByte, DEC);
                // Debug - used to send different voltages to the xbox pad. 48 is ascii for 0
                //analogWrite(outputPin, (incomingByte-48) * 5);
  }

}
