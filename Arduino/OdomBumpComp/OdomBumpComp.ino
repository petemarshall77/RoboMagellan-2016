#include <Wire.h>
int HMC6352Address = 0x42;
int slaveAddress;
int ledPin = 13;
boolean ledState = false;
byte headingData[2];
int i, headingValue;
int HallState;
int bump;
int HallState_Last = 0;
int count = 0;
void setup()
{
  pinMode(2, OUTPUT);
  pinMode(4, INPUT_PULLUP);
  digitalWrite(2, HIGH);
  pinMode(7,INPUT);
  Serial.begin(9600);
  slaveAddress = HMC6352Address >> 1;   
  pinMode(ledPin, OUTPUT);    
  Wire.begin();
}

void loop()
{
    HallState = digitalRead(7);
    if (HallState != HallState_Last) {
        // if the state has changed, increment the counter
        if (HallState == HIGH) {
          // if the current state is HIGH then the button
          // wend from off to on:
          count++;
        }
        else {
          // if the current state is LOW then the button
          // wend from on to off:
        }
      }
    // save the current state as the last state,
    //for next time through the loop
    HallState_Last = HallState;
    Wire.beginTransmission(slaveAddress);
    Wire.write("A");              
    Wire.endTransmission();
    delay(10);                  
    Wire.requestFrom(slaveAddress, 2);      
    i = 0;
    if(digitalRead(4) == LOW) bump = 1;
    if(digitalRead(4) == HIGH) bump = 0;
    while(Wire.available() && i < 2)
    { 
      headingData[i] = Wire.read();
      i++;
    }
    Serial.print(count);
    Serial.print(",");
    headingValue = headingData[0]*256 + headingData[1]; 
    Serial.print(int (headingValue / 10));
    Serial.print(",");
    Serial.println(bump);
  }

