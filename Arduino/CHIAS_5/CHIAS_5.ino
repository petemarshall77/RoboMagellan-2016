//------------------------------------------------------------------------
// MONTY2 Control Hardware Interfacing Arduino (CHIA)
//
//     CHIA is the system that MONTY2 uses to interface between the main
//     controller laptop and the robot steering servo and motor
//     controller. It communicates with the controller via a serial port
//     which is used to send and recieve steering and throttle data.
//
//     The CHIA (and hence Monty's throttle/steering system) can is in
//     Juan of four states:
//      
//         1. STOPPED  - the robot is stopped and no power or steering
//                       data is sent or recieved. This is the initial
//                       state of the CHIA and the state in which the
//                       controller will use to compeletly stop the
//                       robot.
//
//         2. MANUAL   - this is use to simply drive Monty as a remote
//                       control vehicle. The CHIA simply passes the
//                       throttle and steering PWM signals from the
//                       radio receiver to the power contoller and
//                       steering servos.
//
//         3. RECORD   - Similar to MANUAL but the CHIA writes the 
//                       throttle and steering values to the controller
//                       via the serial port every 100mS.
//
//         4. PLAYBACK - The opposite of RECORD (naturally): the CHIA reads
//                       back the recorded throttle and steering data
//                       from the controller and drives the robot using the
//                       previously recorded data. (See "Waldo" a short
//                       story by Robert Heinlein; "Player Piano" by Kurt
//                       Vonnegut)
//
//                       PLAYBACK mode can also be used by Monty in automous
//                       mode. The only difference is the data being sent
//                       to the CHIA is computed and not recorded and is
//                       sent asynchonously rather than just every 100mS.
//                       To avoid any confusion, PLAYBACK has an alias of
//                       SLAVE.
//
//     The CHIA's state is changed by the controller sending commands. On
//     startup the CHIA will be in the STOPPED state. The controller will
//     send a +++MAUNUAL+++, +++RECORD+++, +++PLAYBACK+++ or +++SLAVE+++
//     command to the CHIA to change state. Once in the state asked for by 
//     the controller, the CHIA will keep going until a +++STOP+++ command
//     is received.
//
//     Note: the CHIA expects data to be send as newline ('\n') terminated
//     strings. Don't send it CRs/LFs or CR/LFs, just newlines please.
//     Failure to comply will void your warranty and may lead to serious
//     injury or death (or, worse still, a robot that just sits there and
//     does nothing). You have been warned.
//
#include <Servo.h>

//------------------------------------------------------------------------
// Dramatis personae
//------------------------------------------------------------------------

// Safety
#define MAX_POWER 1635

// Serial buffer
#define BUFFER_LENGTH 80
char serialBuffer[BUFFER_LENGTH];
String inputString;

// System can be in Juan of four states...
#define STOPPED  1
#define MANUAL   2
#define RECORD   3
#define PLAYBACK 4   // this is also the SLAVE state
#define DRIVE    5
int system_state;

// Pin declarations
const int steer_Pin = 2;  // do not change: mapped to interrupt 0
const int power_Pin = 3;  // do not change: mapped to interrrup 1
const int steer_Out = 8;
const int power_Out = 9;
const int LED_pin = 13;

// Interrupt routine values
volatile unsigned long steer_Microseconds;
volatile unsigned long steer_Initial_Result;
volatile unsigned long steer_Result;

volatile unsigned long power_Microseconds;
volatile unsigned long power_Initial_Result;
volatile unsigned long power_Result;

volatile unsigned long safety_Microseconds;
volatile unsigned long safety_Initial_Result;
volatile unsigned long safety_Result;

//------------------------------------------------------------------------
// SETUP
//------------------------------------------------------------------------
void setup()
{
  // System start in stopped state
  system_state = STOPPED;
  
  // Set up serial communications
  Serial.begin(9600);
  Serial.println("Serial monitor connected");
  
   // Pin initializations
  pinMode(steer_Pin, INPUT);
  pinMode(power_Pin, INPUT);
  pinMode(steer_Out, OUTPUT);
  pinMode(power_Out, OUTPUT);
  pinMode(LED_pin,OUTPUT);

  // Ensures that the two ouput pins are set to low at the start
  digitalWrite(steer_Out, LOW);
  digitalWrite(power_Out, LOW);
}

//------------------------------------------------------------------------
// Main LOOP
//------------------------------------------------------------------------
void loop()
{
  switch(system_state) {
    
    case STOPPED:
      do_stopped();
      break;
      
    case MANUAL:
      do_manual();
      break;
      
    case RECORD:
      do_record();
      break;
 
    case DRIVE:
      do_drive();
      break;
 
    default:
      // Shouldn't happen!
      Serial.write("Fatal error - system_state is incorrect.");
      break;
  }
}


//------------------------------------------------------------------------
// STOPPED Mode - robot does nothing but wait for a command
//
//      This is the intial robot state. It simply waits for the 
//      controlling computer to send a +++MANUAL+++, +++RECORD+++,
//      or +++PLAYBACK+++ command to change state.
//------------------------------------------------------------------------
void do_stopped()
{
  Serial.println("System state is now STOPPED");
  
  while (true) {
    if (readLine(serialBuffer)) {
      inputString = String(serialBuffer);
      if (inputString.compareTo("+++MANUAL+++") == 0) {
        system_state = MANUAL;
        return;
      }
      if (inputString.compareTo("+++RECORD+++") == 0) {
        system_state = RECORD;
        return;
      }
      if (inputString.compareTo("+++PLAYBACK+++") == 0) {
        system_state = PLAYBACK;
        return;
      }
      if (inputString.compareTo("+++DRIVE+++") == 0) {
        system_state = DRIVE;
        return;
      }
    }
  }
}


//------------------------------------------------------------------------
// MANUAL Mode - drive the robot as a remote control car
//
//      In manual mode, a pair of interrupt handlers service changes to 
//      the steering and throttle output pins of the radio reciever.
//      These pins present PWM signals that the interrupt handlers simply
//      pass on to the steering servo(s) and motor controller.
//------------------------------------------------------------------------
void do_manual()
{
  Serial.println("System state is now MANUAL");
  
  // Attach interrupt handters to pins 2 and 3 to capture the output
  // from the radio reciever.
  attachInterrupt(0, steerInterrupt, CHANGE); // interrupt 0 is pin 2
  attachInterrupt(1, powerInterrupt, CHANGE); // interrupt 1 is pin 3
  
  // In manual mode the interrupt handlers so all the work. All we need
  // to do here is to check for a +++STOP+++ signal from the controller.
  while (true) {
    if (readLine(serialBuffer)) {
      inputString = String(serialBuffer);
      if (inputString.compareTo("+++STOP+++") == 0) {
        // stop driving - disable interrupts and set power/steering low
        detachInterrupt(0);
        detachInterrupt(1);
        digitalWrite(steer_Out, LOW);
        digitalWrite(power_Out, LOW);
        // update the system state and return
        system_state = STOPPED;
        return;
        }
    }
  }
}


//------------------------------------------------------------------------
// RECORD Mode
//
//     RECORD MODE is much the same as MANUAL mode except that the power
//     and steering values are written to the controller computer every
//     100mS.
//
//------------------------------------------------------------------------
void do_record()
{
  unsigned long loop_start_time;
  
  Serial.println("System state is now RECORD");

  // Attach interrupt handters to pins 2 and 3 to capture the output
  // from the radio reciever.
  attachInterrupt(0, steerInterrupt, CHANGE); // interrupt 0 is pin 2
  attachInterrupt(1, powerInterrupt, CHANGE); // interrupt 1 is pin 3
  
  // Each time through the loop, write out the steering and throttle
  // values to the controller, check for a stop command and then wait
  // until 100mS has passed. As the Serial reads and writes take time
  // we use a microsecond timer rather than a fixed delay value.
  while (true) {
    loop_start_time = micros();
    
    // check for +++STOP+++ command
    if (readLine(serialBuffer)) {
      inputString = String(serialBuffer);
      if (inputString.compareTo("+++STOP+++") == 0) {
        // stop driving - disable interrupts and set power/steering low
        detachInterrupt(0);
        detachInterrupt(1);
        digitalWrite(steer_Out, LOW);
        digitalWrite(power_Out, LOW);
        // update the system state and return
        system_state = STOPPED;
        return;
        }
    }
    //
    // Write out current steering and power values, conditioning any
    // value to be > 1000 and < 2000
    if (steer_Result < 1000) {
      steer_Result = 1000;
    }
    else if (steer_Result > 2000) {
      steer_Result = 2000;
    }
    if (power_Result < 1000) {
      power_Result = 1000;
    }
    else if (power_Result > 2000) {
      power_Result = 2000;
    }
    
    Serial.print(steer_Result);
    Serial.print(",");
    Serial.println(power_Result);
    Serial.flush();
    
    // Wait for the remainder of the 100mS
    while ((micros() - loop_start_time) < 100000) {
      // do nothing!
    }
  }
}   
 

//------------------------------------------------------------------------
// DRIVE mode
//
//    DRIVE mode is like PLAYBACK with no safety checks,      
//------------------------------------------------------------------------
void do_drive()
{
  unsigned long loop_start_time;
  unsigned long steer_value, power_value;
  
  Servo steer_servo;
  Servo power_servo;
  
  Serial.println("System state is now DRIVE");
  
  // Attach servo outputs
  steer_servo.attach(steer_Out);
  power_servo.attach(power_Out);
  
  loop_start_time = micros();
  
  while (true) {
    
    
    // Process line of data if available
    if (readLine(serialBuffer)) {
      inputString = String(serialBuffer);      
      // Check for STOP
      if (inputString.compareTo("+++STOP+++") == 0) {
        // stop driving - disable interrupts and set power/steering low
        detachInterrupt(1);
        steer_servo.detach();
        power_servo.detach();
        digitalWrite(steer_Out, LOW);
        digitalWrite(power_Out, LOW);
        // update the system state and return
        system_state = STOPPED;
        return;
        }
        
      // Process the data string, should be nnnn,nnnn (steer,power)
      steer_value = inputString.substring(0,4).toInt();
      power_value = inputString.substring(5,9).toInt();
      if (steer_value < 1000 || steer_value > 2000 || power_value < 1000 || power_value > 2000) {
        Serial.println("Bad data from laptop. PANIC");
        Serial.flush();
        panic();  // bad data, assume communication error, panic
      }
      
      // Enforce maximum power value
      if (power_value > MAX_POWER) {
        power_value = MAX_POWER;
      }
      
      // Got good values, and safety switch is off: write power and steering values to hardware
      steer_servo.writeMicroseconds(steer_value);
      power_servo.writeMicroseconds(power_value);
      
      loop_start_time = micros();
      
    }
    if(micros() - loop_start_time > 2000000){
      power_servo.writeMicroseconds(1500);  
      Serial.println("Power value timed out.");
      loop_start_time = micros();
      }
  }
}

// PANIC Function, called when we assume communication with the controller is lost
void panic()
{
    noInterrupts();                  // stop servicing interrupts
    digitalWrite(power_Out, LOW);    // stop the robot's power
    while (true) {                   // blink the LED
      digitalWrite(LED_pin, HIGH);
      delay(200);
      digitalWrite(LED_pin, LOW);
      delay(200);
    }
}
      

//------------------------------------------------------------------------
// Miscellaneous functions
//------------------------------------------------------------------------

// Read a line of data from the serial port, placing the data as a well-
// formed, null-delimited string in buffer and returning the length of
// the input. Return 0 if no data is available.
int readLine(char* buffer)
{
  int byteCount = 0;
  int inputByte;
  
  if (Serial.available()) {
    // data is available: keep reeading until newline is recieved
    while ((inputByte = Serial.read()) != '\n') {
      if (inputByte != -1) {
        buffer[byteCount] = inputByte;
        byteCount++;
      }
    }
    buffer[byteCount] = 0;  // make end of string
    return byteCount;       // note: if no data is available, will return 0
  }
  else {
    return 0;
  }
}


//------------------------------------------------------------------------
// INTERRUPT Handlers
//
//     The steering and power interrupt handlers are used in both MANUAL
//     and RECORD mode. They are called whenever the output of the radio
//     control receiver changes state. The signals they are handling are
//     standard servo PWM signals and these routines do two things:
//
//       1 - mirror the input signals to an output attached to the
//           vehicle steering servo and motor controller, essentially
//           just passing the PWM signal through the arduino.
//
//       2 - calculate the pulse width in microseconds. These routines
//           are called on change of signal state so they don't know
//           if the current state is high or low. However, as standard
//           servo pulses last from 1000 to 2000 microseconds and the
//           overall cycle time is 10000 microseconds (i.e. the signal
//           is low for between 9000 and 8000 microseconds) then only
//           times between calls in the 1000 - 2000 range are the Juans
//           that represent PWM pulse widths.
//
//       The safety interrupt handler measures the throttle signal, just
//       like the powerinterrupt but does not mirror the signal to the
//       powercontrol. This is used in PLAYBACK/SLAVE mode to use the
//       throttle trigger as a fail-safe switch.
//
//------------------------------------------------------------------------

// STEERING interrupt handler
void steerInterrupt()
{
  // Get the time since the last interrupt
  steer_Initial_Result = micros() - steer_Microseconds;

  // If this time represents a pulse the signal has just gone low:
  // set the output low and update the steering value in steer result.
  // Note that real-world PWM values go down into 900's so we reflect
  // this here.
  if (steer_Initial_Result <= 2000 && steer_Initial_Result >= 900){
    steer_Result = steer_Initial_Result;
    digitalWrite(steer_Out, LOW);
  }
  else {
    // this is the begining of a new pulse: set the output high but
    // don't update the pulse width value
    digitalWrite(steer_Out, HIGH);  // See above comment
  }

  // Reset time for next interrupt
  steer_Microseconds = micros();
}


// POWER interrupt handler
//
//     This is a mirror of the steering handler above. Refer to the code
//     comments there.
//
void powerInterrupt()                                                             
{                                                                               
  power_Initial_Result = micros() - power_Microseconds;                         
                                                                                
  if (power_Initial_Result <= 2000 && power_Initial_Result >= 900){             
    power_Result = power_Initial_Result;                                        
    digitalWrite(power_Out, LOW);                                                                                                                               
  }
  else {
    digitalWrite(power_Out, HIGH);  // See above comment
  }

  // Reset time for next interrupt
  power_Microseconds = micros();
}

// SAFETY interrupt handler
//
//     This is a mirror of the power handler above but does not mirror
//     the output signal, just records the throttle value.
//
void safetyInterrupt()                                                             
{                                                                               
  safety_Initial_Result = micros() - safety_Microseconds;                         
                                                                                
  if (safety_Initial_Result <= 2000 && safety_Initial_Result >= 900){             
    safety_Result = safety_Initial_Result;                                                                                                                                                                       
  }
  else {
    // do nothing
  }

  // Reset time for next interrupt
  safety_Microseconds = micros();
}

