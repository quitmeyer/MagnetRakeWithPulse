
/*
>> Pulse Sensor Amped 1.1 <<
 This code is for Pulse Sensor Amped by Joel Murphy and Yury Gitman
 www.pulsesensor.com 
 >>> Pulse Sensor purple wire goes to Analog Pin 0 <<<
 Pulse Sensor sample aquisition and processing happens in the background via Timer 2 interrupt. 2mS sample rate.
 PWM on pins 3 and 11 will not work when using this code, because we are using Timer 2!
 The following variables are automatically updated:
 Signal :    int that holds the analog signal data straight from the sensor. updated every 2mS.
 IBI  :      int that holds the time interval between beats. 2mS resolution.
 BPM  :      int that holds the heart rate value, derived every beat, from averaging previous 10 IBI values.
 QS  :       boolean that is made true whenever Pulse is found and BPM is updated. User must reset.
 Pulse :     boolean that is true when a heartbeat is sensed then false in time with pin13 LED going out.
 
 This code is designed with output serial data to Processing sketch "PulseSensorAmped_Processing-xx"
 The Processing sketch is a simple data visualizer. 
 All the work to find the heartbeat and determine the heartrate happens in the code below.
 Pin 13 LED will blink with heartbeat.
 If you want to use pin 13 for something else, adjust the interrupt handler
 It will also fade an LED on pin fadePin with every beat. Put an LED and series resistor from fadePin to GND.
 Check here for detailed code walkthrough:
 http://pulsesensor.myshopify.com/pages/pulse-sensor-amped-arduino-v1dot1
 
 Code Version 02 by Joel Murphy & Yury Gitman  Fall 2012
 This update changes the HRV variable name to IBI, which stands for Inter-Beat Interval, for clarity.
 Switched the interrupt to Timer2.  500Hz sample rate, 2mS resolution IBI value.
 Fade LED pin moved to pin 5 (use of Timer2 disables PWM on pins 3 & 11).
 Tidied up inefficiencies since the last version. 
 */

/*
Magnet Rake with Pot
 
 An array of magnets hooked up to an DUAL  H Bridge DIP chip frequency controlled by
 */



///Magnets in comb
int magA=  13; //Mag A from the comb will also be mapped to the stylus
int magB = 12;
int magC = 11;
int magD = 10;
//Magnet on pen == MagA

int magArray[] = {
  magA, magB, magC, magD};
int magStateA= LOW;
int magStateB= LOW;
int magStateC= LOW;
int magStateD= LOW;
int magState[] = {
 magStateA, magStateB, magStateC, magStateD};


// When cycling through the comb
int currentmag = 13; //just default value


//LED's flashing on the comb

int CombLEDpinA = 9;
int CombstateA = LOW; 

int CombLEDpinB = 8;
int CombstateB = LOW; 

int CombLEDpinC = 7;
int CombstateC = LOW; 

int CombLEDpinD = 6;
int CombstateD = LOW; 

//LED's flashing on Pen
int PenLEDpin = 2;
int PenstatebuttonLEDs = LOW; 

int CombLEDPinArray[]= {CombLEDpinA, 0, 0, 0}; //All zeroes for now because of hack rewiring because digital pins 0 and 1 are used for RX TX
int CombStateArray[] = {CombstateA, CombstateB, CombstateC, CombstateD};
//LED's on the button jars
//note these are overloaded since the analog pins and the digital pins will correspond to the same number
const int buttonledPin1 =  3;      // BL1
int buttonLEDState1 = LOW; 
const int buttonledPin2 =  4;     // BL2
int buttonLEDState2 = LOW;
const int buttonledPin3 =  5;     // BL3
int buttonLEDState3 = LOW;
const int buttonledPin4 =  6;     // BL4
int buttonLEDState4 = LOW;
const int buttonledPin5 = 7;       // BL5 - constant on
int buttonLEDState5 = LOW;
const int buttonledPin[] = {buttonledPin1, buttonledPin2, buttonledPin3, buttonledPin4,buttonledPin5};

int buttonLEDState[] = {
  buttonLEDState1, buttonLEDState2, buttonLEDState3, buttonLEDState4, buttonLEDState5};


//Sensors reading connections from Button jars
//note these are overloaded since the analog pins and the digital pins will correspond to the same number
const int ButtonSensePin1 =  1;      // BL1

const int ButtonSensePin2 =  2;     // BL2
const int ButtonSensePin3 =  3;     // BL3
const int ButtonSensePin4 =  4;     // BL4
const int ButtonSensePin5 = 5;       // BL5 - constant on
const int buttonSensePin[] = {ButtonSensePin1, ButtonSensePin2, ButtonSensePin3, ButtonSensePin4,ButtonSensePin5};

// variables will change:
//Default of 0 (not selected), selections usually give results around 300-400;
int buttonState1 = 0;         // variable for reading the pushbutton status
int buttonState2 = 0; 
int buttonState3 = 0; 
int buttonState4 = 0; 
int buttonState5 = 0; 



int matchedButton = 0; 

int buttonThreshold = 250;



int potpin = 0;  // analog pin used to connect the potentiometer in leiu of pulse pin
int pulseval;    // variable to read the value from the analog pin 
int pulseperiod;

long magpreviousMillis = 0;
long magmillisArray[] = {0,0,0,0};
long previousMillis[] = {
  0, 0, 0, 0, 0};

int modeSelection =1;  //The mode points to what behavior mode is currently selected



long delay1 =  200; // //  eg. 0 = potentiometer / pulse reading (or default 75 bpm  of 5hz 200 ms)

long delay2 =  100; //    2 = 10hz  == 100 ms delay

long delay3 =  10; //    3 = 100hz == 10 ms delay

long delay4 =  10; //    4 is random // this will randomly change through the loop, ignore this default value

long delay5 =  -1; //    5 = OFF


int delayX[] = {
  delay1, delay2, delay3, delay4, delay5}; //Update array everytime it loops






//***Heart Monitor stuff
//  VARIABLES
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
//int blinkPin = 8;                // pin to blink led at each beat
//int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
int fadeRate = 0;                 // used to fade LED on with PWM on fadePin


// these variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, the Inter-Beat Interval
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

///***Heart Monitor stuff***/





void setup(){
  Serial.begin(115200);             // we agree to talk fast!

  //******Heart Monitor thing *******

  //  pinMode(blinkPin,OUTPUT);         // pin that will blink to your heartbeat!
  //  pinMode(fadePin,OUTPUT);          // pin that will fade to your heartbeat!

  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS 
  // UN-COMMENT THE NEXT LINE IF YOU ARE POWERING The Pulse Sensor AT LOW VOLTAGE, 
  // AND APPLY THAT VOLTAGE TO THE A-REF PIN
  //analogReference(EXTERNAL);   
  //*****Heart Monitor****\\

  // initialize the digital pin as an output for magnets
  pinMode(magA, OUTPUT);     
  pinMode(magB, OUTPUT);     
  pinMode(magC, OUTPUT);  
  pinMode(magD, OUTPUT);  
  //LED's on the button jars         
  pinMode(buttonledPin1, OUTPUT);    
  pinMode(buttonledPin2, OUTPUT); 
  pinMode(buttonledPin3, OUTPUT); 
  pinMode(buttonledPin4, OUTPUT); 
  pinMode(buttonledPin5, OUTPUT);

  //Led on Pen
  pinMode(PenLEDpin, OUTPUT);

  //LED's flashing on the comb
  pinMode(CombLEDpinA, OUTPUT);
  pinMode(CombLEDpinB, OUTPUT);
  pinMode(CombLEDpinC, OUTPUT);
  pinMode(CombLEDpinD, OUTPUT);



}



void loop(){


  // *** Heart Monitor stuff *** \\\

  sendDataToProcessing('S', Signal);     // send Processing the raw Pulse Sensor data
  if (QS == true){                       // Quantified Self flag is true when arduino finds a heartbeat
    fadeRate = 255;                  // Set 'fadeRate' Variable to 255 to fade LED with pulse
    sendDataToProcessing('B',BPM);   // send heart rate with a 'B' prefix
    sendDataToProcessing('Q',IBI);   // send time between beats with a 'Q' prefix
    QS = false;                      // reset the Quantified Self flag for next time    
  }

  ledFadeToBeat();
  // *** Heart Monitor stuff *** \\\

  //Read pulse or potpin
  pulseval = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  pulseperiod = map(pulseval, 0, 1023, 0, 1000 ); 

  //Pulse Reading
//  Serial.print("pulseperiod =  ");
//  Serial.println(pulseperiod);
//  Serial.print("Heartbeat =  ");
//  Serial.println(BPM);
//  float bpmtoperiod = 1/(BPM/60);
  //    scanMags(period);

  // *** Heart Monitor stuff *** \\\


  //Check for Buttons being activated
  //Decide which MODE we are operating in
  determineMode();
  
//   Serial.println(matchedButton);

  //update the values for all delays

  updateDelays();


  // Perform the selected behavior of the mode
  // Do all the blinking for the cups
blinkButtons();

    //Do all the blinking and pulsing for the Pen and Comb
pulseMags();


  //Old function to scan through comb Magnet
  //scanMags(bpmtoperiod);


  //  delay(20);                             //  take a break
}


void determineMode(){

Serial.print("Buttons State Readings == ");
  for (int i = 0; i < 5; i++) {
     int sensorvalue = analogRead(buttonSensePin[i]);            // reads the value of the potentiometer (value between 0 and 1023) 
    Serial.print(sensorvalue);
Serial.print("  ");
    if (sensorvalue >= buttonThreshold) { 

      matchedButton = i;
      //  previousMillis[4] = previousMillis[i];

    } 


  }
  
  Serial.print(" --  matched button =  ");
  Serial.println(matchedButton);



}

void updateDelays(){

  // delay1 = pulseperiod;
  delay1 =  800; // //  eg. 0 = potentiometer / pulse reading (or default 75 bpm  of 1.25hz 800 ms)
  delayX[0] = delay1;
  delay2 =  100; //    2 = 10hz  == 100 ms delay
  delayX[1] = delay2;

  delay3 =  10; //    3 = 100hz == 10 ms delay
  delayX[2] = delay3;

  delay4 =  random(2, 2000); //    4 is random // this will randomly change through the loop, ignore this default value
  delayX[3] = delay4;

  delay5 =  0; //    5 = OFF
  delayX[4] = delay5;

  //int delayX[] = {delay1, delay2, delay3, delay4, delay5}; //Update array everytime it loops



}

void blinkButtons(){
  unsigned long currentMillis = millis();
  for (int i = 0; i < 5; i++) {
//Toggle Button state after a timer for each
    if(currentMillis - previousMillis[i] > delayX[i]) {


      previousMillis[i] = currentMillis;
      // save the last time you blinked the LED 

      // if the LED is off turn it on and vice-versa:
      if (buttonLEDState[i] == LOW) {
        buttonLEDState[i] = HIGH;

      }
      else {
        buttonLEDState[i] = LOW;
        //     previousMillis1temp = currentMillis;

      }

    }
 digitalWrite(buttonledPin[i], buttonLEDState[i]);


  }

}

void pulseMags(){

  if(matchedButton == 4){ // Do nothing, turn off all magnets

    for (int z =0; z<5; z++){
      digitalWrite(magArray[z],LOW);   // turn the LED on (HIGH is the voltage level)
    }
  }
  else{

    scanMags(delayX[matchedButton]); // Scan through the magnets on the comb with a delay set by the behavior of the matched button 


  }




}


void scanMags(int P){

  P = P/4; //(divide the period by 4  because comb has 4 and so that Pen is consistent
  unsigned long currentMillis = millis();
  

/*
 for (int i = 0; i < 4; i++) {
//Toggle Magnet state after a timer for each
    if(currentMillis -  magmillisArray[i]  +i*(1/4) > P) {


      magmillisArray[i]  = currentMillis;
      // save the last time you pulse a mag 

      // if the Mag is off turn it on and vice-versa:
      if (magState[i] == LOW) {
       magState[i] = HIGH;
       currentmag= i;
      }
      else {
        magState[i] = LOW;

      }

    }
// digitalWrite(magArray[i], magState[i]);
// digitalWrite(CombLEDPinArray[i], magState[i]);

//TURN ALL OFF
  digitalWrite(magArray[i], LOW);
  digitalWrite(CombLEDPinArray[i], LOW);
 }
  digitalWrite(magArray[currentmag], HIGH);
    digitalWrite(CombLEDPinArray[currentmag], HIGH);
/* */
  if(currentMillis - magpreviousMillis > P) {
    magpreviousMillis = currentMillis;   
    //Activate the next magnet in the list and turn the rest off
    digitalWrite(currentmag, LOW);   // turn the LED on (HIGH is the voltage level)
        digitalWrite(9, LOW); //HACK FOR TESTING
                digitalWrite(2, LOW); //HACK FOR TESTING

    if(currentmag == magD){
      currentmag = magA ;
          digitalWrite(9, HIGH); //HACK FOR TESTING
                  digitalWrite(2, HIGH); //HACK FOR TESTING
    }
    else{
      currentmag--; 
    }

    digitalWrite(currentmag, HIGH);   // turn the LED on (HIGH is the voltage level)

  }

  /*  
   for(int i = magA; i>=magD;i--){
   //Turn all off
   digitalWrite(magA, LOW);   // turn the LED on (HIGH is the voltage level)
   digitalWrite(magB, LOW);   // turn the LED on (HIGH is the voltage level)
   digitalWrite(magC, LOW);   // turn the LED on (HIGH is the voltage level)
   digitalWrite(magD, LOW);   // turn the LED on (HIGH is the voltage level)
   //Turn selected on
   digitalWrite(i, HIGH);   // turn the LED on (HIGH is the voltage level)
   delay(P); 
   
   }
   */
}




// *** Heart Monitor stuff *** \\\

void ledFadeToBeat(){
  fadeRate -= 15;                         //  set LED fade value
  fadeRate = constrain(fadeRate,0,255);   //  keep LED fade value from going into negative numbers!
  //    analogWrite(fadePin,fadeRate);          //  fade LED
}


void sendDataToProcessing(char symbol, int data ){
//  Serial.print(symbol);                // symbol prefix tells Processing what type of data is coming
//  Serial.println(data);                // the data to send culminating in a carriage return
}




