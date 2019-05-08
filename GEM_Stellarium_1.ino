#include <AccelStepper.h>

//======for the encoders===============================================================
//======void handleEncoder() function==================================================

// Interrupt information for encoder 1 (RA) 
// 3 on pin 20 as per documentation
// 4 on pin 21 
#define encoderI 20 
#define encoderQ 21      // Only use one interrupt in this example 
volatile long ECcountRA;  // Encoder value right ascension 
long ECsecangle;

//======for sidereal rate change with debounced buttons========================================
//======void buttonSidRate() function and void buttonSid()========================================================  

// definition for the sidereal on-off pushbutton
const int buttonPinSid = 26;    // the number of the sidereal on-off pushbutton pin
const int ledPin = 13;      // the number of the LED pin
int stateSid = HIGH;         // the current state of the output pin
int buttonStateSid;             // the current reading from the input pin
int lastButtonStateSid = LOW;   // the previous reading from the input pin
unsigned long lastSidTime = 0;  // the last time the output pin was toggled

//definitions for the increas and decrease sidereal rate buttons
const int inPinUp = 22;
const int inPinDown = 24;
const int sidonoff = 26;
static float rate = 60;
int buttonUpState = 0;
int buttonDownState = 0;
int prevBtnUp = LOW;
int prevBtnDwn = LOW;
unsigned long lastBtnUp = 0;
unsigned long lastBtnDwn = 0;
int transInt = 50;



//======for stepper control===============================================================
 
// Define the stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 3, 2);
long slewSteps = 0;


//======for serial communciation with stellarium ===========================================
//======void communication() function=======================================================

const byte numChars = 32;
char MeadeLXcom[numChars];// Meade LX200 communication protocol data type indicators
boolean newData = false;

String RAset = "";       // Values received for slews
String DECset = "";

long setHour;
long setMinute;
long setSecond;
long RAsetSec;


String RAencode = "00:00:00#";        // Formatted RA encoder sent to stellarium starting in home position
String DECencode = "+45:00:00#";      // Formatted DEC encoder sent to stellarium starting in home position


//======for getting sidereal time from user (or gps in future?) ==========================
//======void initializeSidTime() function ===============================================


unsigned long initSseconds;         // initial millis value for subtraction initial sideral seconds
unsigned long sid_hours;            // integer for initial hours value
unsigned long sid_minutes;          // integer for initial minutes value
unsigned long sid_seconds;          // integer for initial hours value

//======for calculating the sidereal time OR RA in seconds and HH:MM:SS format=============
//======void calcSidTime() function ===================================================
//======void calcRA() function

int lstHours;
int lstMinutes;
int lstSeconds;
unsigned long allseconds;
unsigned long LST;
int secsRemaining;
char buf[21];
long slewSec;
long pos;



// for debugging
//const long interval = 3000;
//unsigned long previousMillis = 0;

void setup() 
{
  Serial.begin(9600);               // Serial baudrate set by stellarium comm with lx200 telesccopes
  Serial.setTimeout(10);            // decrease wait time during serial.parseint
  

//======variables for button functions===============================================================================
  pinMode(inPinUp, INPUT);           //sideral rate up button
  pinMode(inPinDown, INPUT);         //sideral rate down button
 
  pinMode(buttonPinSid, INPUT);     //sidereal on-off button
  pinMode(ledPin, OUTPUT);          // on board LED lit when mount is ready to accept new slew coordinate an off when in sideral tracking mode
  digitalWrite(ledPin, stateSid);             // set initial LED state

//======variables for encoder function===============================================================================
  attachInterrupt(3, handleEncoder, CHANGE);  //make one of the encoder pins and interrupt pin
  ECcountRA=0;                                // set encoder to 0 counts
  pinMode(encoderI, INPUT);                   // set encoder pin to input
  pinMode(encoderQ, INPUT); 

//======variables for accelstepper===============================================================================  
  stepper.setMaxSpeed(10000);        // accelstepper values set
  stepper.setAcceleration(2000);
  

  Serial.println("make sure a 10uf capacitor is between reset and ground pins to use with stellarium!");
  initializeSidTime();  // Gets current LST value from user
    
    int i = 0;          // displays the LST time a couple times just to show user whether it makes sense for what entered
      while (i<3) {
      calcSidTime();
      sprintf(buf,"LST = %02d:%02d:%02d",lstHours,lstMinutes,lstSeconds);
      Serial.println(buf);
      delay(1000); 
      i++;
      }
}


void loop() 
{
    
//    unsigned long currentMillis = millis();                 //serial debugging 
//    if (currentMillis - previousMillis >= interval) {
//    previousMillis = currentMillis;
//    Serial.println("LST");
//    Serial.println(LST);
//    Serial.println("pos");
//    Serial.println(pos);
//   
//    Serial.println(slewSec);
//    Serial.println(RAencode);
//    Serial.println(setHour);
//    Serial.println(setMinute);
//    Serial.println(setSecond);
//    Serial.println(RAsetSec); 
//  }
      
calcRA();           // Calculates the current RA from initialized Local Sideral Time and Angle changes from the encoder, this definitely does not need to be calculated every loop as Stellarium only polls about every 1/2 second but doesn't seem to affect performance
readStell();        // Reads serial commands from Stellarium and stores them in the char array 'MeadeLXcom'
parseStell();       // Parses the serial commands from stellarium, replies with current RA and DEC if queried and initiates slews
buttonSid();        // Reads the button that turns sidereal tracking on and off 

  if (stateSid == LOW){                     // if sideral tracking is on read the other buttons which increment and decrement the sidereal tracking rate
    buttonSidRate();                        // reads button presses and increases or decreases tracking rate
    }
  if (stateSid == HIGH) {                   // if sideral tracking in off then the mount is free to slew when it accepts new slew coordinates from Stellarium, the stepper count is set to zero when the buttonSid() function shifts back to slew mode so none of the tracking steps are retained  
    if ((stepper.distanceToGo() != 0)) {
    stepper.run();                          // Move Stepper X into position
    }
    }
  }
//=====function - communicate with stellarium===================================================================

  void readStell()      // Function reads serial data in a non-blocking manner - Directly pilfered from proflific poster TOM on Arduino forum
  {
   static byte ndx = 0;
    char endMarker = '#'; // End marker in the LX200 command format
    char rc;
    
    while (Serial.available() > 0 && newData == false) {   // if end marker not recieved read the buffer and append to the MeadeLXcom char array
        rc = Serial.read();

        if (rc != endMarker) {
            MeadeLXcom[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {                                              // if end marker is recieved terminate the array and set flag to new data
            MeadeLXcom[ndx] = '\0'; 
            ndx = 0;
            newData = true;
        }
    }
}
//=====function - parse  stellarium data if received=================================================
void parseStell() {                                     // parse new data if recieved - the readStell() function sets sets new data to true after recieving the end character
    if (newData == true) {

        
        if(strcmp(MeadeLXcom, ":GR") == 0){             // if command received = get RA do...
            Serial.print(RAencode);
        }

        if(strcmp(MeadeLXcom, ":GD") == 0){             // if command received = get DEC do...
            Serial.print(DECencode);
        }

        if( ( MeadeLXcom[0] == ':' ) && ( MeadeLXcom[1] == 'S' ) && ( MeadeLXcom[2] == 'd' ) ){      // if command received = set DEC. DEC data arrives as :SdsDD*MM:SS
            char *DECtrim = MeadeLXcom +4;        // reformat to data values by moving the point forward to cutoff the start characters which include a leading space (' :Sd') to only leave the new DEC (sDD*MM:SS) where s means sign
            DECtrim[9] = '#';                     // reappend the end character since Stellarium expects it
            DECtrim[10] = '\0';                   // add the char array end character
            Serial.print("1");                    // recieved data successful to stellarium - this is supposed to be an actual check but this program says yes no matter what
            DECencode = DECtrim;                  // new dec set to recieve RA for testing coms, will adjust after parts arrive    
            }

        if( ( MeadeLXcom[0] == ':' ) && ( MeadeLXcom[1] == 'S' ) && ( MeadeLXcom[2] == 'r' ) ){     // if command received = set RA. RA data arrives as :SrHH:MM:SS so it must be parsed into total seconds so that we can slew a certain amount of steps from the current LST seconds to the new seconds
            char *RAtrim = MeadeLXcom +3;         //trim the start characters
            char * strtokIndx;                    // this is used by strtok() as an index
            
            strtokIndx = strtok(RAtrim,":");      // get the first part - hours   
            setHour = atoi(strtokIndx);
        
            strtokIndx = strtok(NULL,":");      // get the second part - minutes  
            setMinute = atoi(strtokIndx); 

            strtokIndx = strtok(NULL,":");      // get the third part - seconds 
            setSecond = atoi(strtokIndx);
          
            RAsetSec = setHour * 3600UL + setMinute * 60UL + setSecond;   // calculate the set RA angle in seconds - the value arrives is hours minutes seconds and is converted to seconds then substraced from the current actual pointing direction and the diffrence is seconds is the hour angle to slew is seconds.
            ECsecangle = ECcountRA/6;                                     // this calculates the quadrature ticks for angle second, 6 will have to be adjusted for different mounts and encoders
            allseconds=millis()/997;
            LST = sid_hours * 3600UL + sid_minutes * 60UL + sid_seconds + allseconds - initSseconds + ECsecangle;
            slewSec = LST - RAsetSec;
            pos = slewSec * 48;
            Serial.print("1");                    // recieved data successful to stellarium - this is supposed to be a check for whether telescope can slew or not but this program is dumb and sends successful no matter what
            stepper.setCurrentPosition(0);        //resets the stepper position since slew to new angle, this might be a bad way to do this
            stepper.moveTo(pos);                  // Set new move position for X Stepper
            }
       
         if(strcmp(MeadeLXcom, ":MS") == 0){             // slew possible check - the progam again says yes though it doesn't actually check anything
            Serial.print("0");
            }
            newData = false;                            // sets new data to false so we're ready to read serial data again with trying to parse the buffer until # arrives
            }
    }



//=====function - keep track of encoder values===================================================================
  void handleEncoder() // simply counts up and down as the encoder ticks
    { 
      if(digitalRead(encoderI) == digitalRead(encoderQ)) {
        ECcountRA++; 
      }
      else{ 
        ECcountRA--; 
      }  
    }

//=====function - initialize sidereal time from user===================================================================
    void initializeSidTime(){                             //This only runs during setup so it doesn't matter than the serial method is very inefficient 

    //ask user for sidereal time: hours
    Serial.println("Enter current sidereal time hours:"); // Send prompt to user
    Serial.print("Hours: ");                              // where the entered value will be displayed
    while(Serial.available()==0) {}                       // waits for serial data
    sid_hours = Serial.parseInt();                        // parses a number entered by user and stores as sideral time variable
    Serial.println(sid_hours);                            // prints received value
    Serial.parseInt();                                    // clears the buffer so the next serial.available while loop won't start if something is left in buffer

    //ask user for sidereal time: minutes  
    Serial.println("Enter current sidereal time minutes:"); 
    Serial.print("Minutes: ");
    while(Serial.available()==0) {}
    sid_minutes = Serial.parseInt();
    Serial.println(sid_minutes);
    Serial.parseInt();                                    

    //ask user for sidereal time: seconds  
    Serial.println("Enter current sidereal time seconds:"); 
    Serial.print("Seconds: ");
    while(Serial.available()==0) {}
    sid_seconds = Serial.parseInt();
    Serial.println(sid_seconds);
    Serial.parseInt(); 

    // -saves the amount of seconds since microcontroller turned on until sidereal time entry completed
    // -this makes sure that recorded sideral time always starts from the point where the entry was finished
    // -diving by 997 converts milliseconds to sideral seconds
    initSseconds = millis()/997UL;
    }
    
//=====function - calculate sidereal time===================================================================  
   void calcSidTime() 
    {
    allseconds=millis()/997; // Keeps track of the LST based on when it was entered 
    LST = sid_hours * 3600UL + sid_minutes * 60UL + sid_seconds + allseconds - initSseconds;
    lstHours= LST/3600;
    secsRemaining=LST%3600;
    lstMinutes=secsRemaining/60;
    lstSeconds=secsRemaining%60;
    sprintf(buf,"LST = %02d:%02d:%02d",lstHours,lstMinutes,lstSeconds);
    }

//=====function - calculate Right Ascension===================================================================  
   void calcRA()
    {
    ECsecangle = ECcountRA/6; // Calculates the RA to send to stellarium based on the local sidereal time and change in the encoder values after turn on
    allseconds=millis()/997;
    LST = sid_hours * 3600UL + sid_minutes * 60UL + sid_seconds + allseconds - initSseconds + ECsecangle;
    lstHours= LST/3600;
    secsRemaining=LST%3600;
    lstMinutes=secsRemaining/60;
    lstSeconds=secsRemaining%60;
    sprintf(buf,"%02d:%02d:%02d#",lstHours,lstMinutes,lstSeconds);
    RAencode = buf;
    }
    
//======void readbutton() function - increases or decreases sideral rate=======================================
void buttonSid() {
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPinSid);

  if (reading != lastButtonStateSid) {            // check to see if you just pressed the button
    lastSidTime = millis();                       // reset the debouncing timer
    }
  if ((millis() - lastSidTime) > transInt) {      // whatever the reading is at, it's been there for longer than the debounce delay, so take it as the actual current state:
      if (reading != buttonStateSid) {            // if the button state has changed:
        buttonStateSid = reading;
      if (buttonStateSid == HIGH) {               // only toggle the LED if the new button state is HIGH
        stateSid = !stateSid;
        stepper.setCurrentPosition(0);            // sets stepper position to zero so steps during sideral tracking aren't taken as a position change by accelstepper function.
      }
    }
  }
  digitalWrite(ledPin, stateSid);                 // set the LED: 
  lastButtonStateSid = reading;                   // save the reading. Next time through the loop, it'll be the lastButtonState:  
}
 
//======void readbutton() function - increases or decreases sideral rate=======================================
void buttonSidRate()                              
{
  buttonUpState = digitalRead(inPinUp);            // Debounced button presses to set sidereal time - the end of the function also sets the runspeed for accelstepper and does none stop motor run from accelstepper function 'runspeed', happens every loop so probably not to efficient.
 `buttonDownState = digitalRead(inPinDown);
 
 if (buttonUpState == HIGH && prevBtnUp == LOW)
 {
   if (millis() - lastBtnUp > transInt)
   {
   rate = rate + 1;
   if(rate > 10000)
   {
    rate = 10000;
   }
   lastBtnUp = millis();
   }
 }
 prevBtnUp = buttonUpState;
 
 if (buttonDownState == HIGH && prevBtnDwn == LOW)
 {
   if(millis() - lastBtnDwn > transInt)
   {
   rate = rate - 1;

   if(rate < 0)
   {
    rate = 0;
   }
   lastBtnDwn = millis();
   }
 }
  prevBtnDwn = buttonDownState;
  stepper.setSpeed(rate);
  stepper.runSpeed();
} 
