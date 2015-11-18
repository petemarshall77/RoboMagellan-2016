#include <Wire.h>

int compass_slave_address = 0x42;     //address of compass controller
int compass_read_address = 0x41;      //read command for compass

int HallState;
int HallState_Last = 0;
volatile unsigned int count = 0;

int bump;

const int bump_pin = 4;
const int hall_effect_pin = 2;

void setup()
{
  pinMode(bump_pin, INPUT_PULLUP);
  //pinMode(hall_effect_pin,INPUT);
  Serial.begin(9600);
  //attachInterrupt(0, hall_effect_routine, HIGH);  //pin 2
  compass_slave_address = compass_slave_address >> 1;      //magic!! (weird compass thing)
  Wire.begin();
}

void loop()
{
  
  /*
   //========== Rotation Sensor ============
    HallState = digitalRead(hall_effect_pin);
    if (HallState != HallState_Last) {
        // if the state has changed, increment the counter
        if (HallState == HIGH) {
          // if the current state is HIGH then the button
          // went from off to on:
          count++;
        }
        else {
          // if the current state is LOW then the button
          // went from on to off:
        }
      }
    // save the current state as the last state,
    //for next time through the loop
    HallState_Last = HallState;
    */
    
    //=========== Compass =============
    Wire.beginTransmission(compass_slave_address);
    Wire.write(compass_read_address);              
    Wire.endTransmission();
    delay(6);            
    
    Wire.requestFrom(compass_slave_address, 2);      
    byte MSB = Wire.read();
    byte LSB = Wire.read();
    int headingValue = (MSB << 8) + LSB;
    headingValue = headingValue / 10;
    
    //============ Bump Switch ==============
    if(digitalRead(bump_pin) == LOW) bump = 1;
    else bump = 0;
    
    //===========Hall Switch================
    //if (digitalRead(hall_effect_pin) == HIGH) count = 0;
    //else count++;
    
    //========== Write out data =============
    Serial.print(count);
    Serial.print(",");
    Serial.print(headingValue);
    Serial.print(",");
    Serial.println(bump);
  }
  
  void hall_effect_routine()
  {
    count++;
  }

