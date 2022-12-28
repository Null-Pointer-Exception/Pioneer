 * Author Ethan Wang
 * Created: 9/18/20
 * Description:
 *    The Robot is programmed using the Arduino IDE. 
 *    
 *    The robot’s operation begins when the user places a package on the turntable, turns on the 
 *    robot, and pushes the start button. It then plays a tune with a buzzer and flashes an LED 
 *    to warn the user to leave the room before the UV-C lamp turns on in 13 seconds
 *    
 *    The robot will then use the long-range infrared distance sensor to determine the maximum 
 *    height of the turntable objects. It increments the linear actuator upwards by five 
 *    centimeters and turns the turntable while taking readings from the distance sensor.  
 *    If it does not detect anything while rotating 360 degrees, the height will be recorded 
 *    in the height variable.
 *    
 *    The robot then turns on the UV-C light and irradiates the vertical sides of the package. 
 *    It starts at the bottom and moves upward by an increment of 10 centimeters each time. 
 *    For each height, the robot disinfects all vertical sides of a box, keeping five to ten
 *    centimeters away from the surface at all times. It uses the variable stored from the 
 *    initial scan to determine how high to raise the lamp. 
 *    
 *    After completing the horizontal scans, the robot raises the lamp above the box and bends
 *    approximately 30 degrees horizontally to disinfect the top. The turntable rotates slowly 
 *    as the light scans the top side of the package. The process is completed after the 
 *    turntable rotates 360 degrees.  
 *    
 *    Finally, the robot sounds the buzzer by playing a tune and flashes the LED to call the user 
 *    back to pick up the package.
 *
 * Revision History:
 *    Ethan Wang - 9/18/20 - Initial Creation
 */


//Includes all required libraries
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <Encoder.h>
#include <Servo.h>  

//Define parameters
#define address 0x80

//For Roboclaw1 - This controls Roboclaw1, which is connected to the Linear Actuator and the 
//Guide Rail motor, the roboclaw is connected to pins 10 and 11
SoftwareSerial serial1(10,11);  
RoboClaw roboclaw1(&serial1,10000);

//For Roboclaw1 - This controlls Roboclaw1, which is connected to the Turntable
SoftwareSerial serial2(5,6);  
RoboClaw roboclaw2(&serial2,10000);

//For TurnTable Encoder - This object receives outputs from the Turntable encoder
Encoder LL(2, 4);        

//Object for controlling servo
Servo LM;

void setup() {
  //For Roboclaws
  roboclaw1.begin(38400);

  //For servo - Servo is connected to PWM pin 9
  LM.attach(9);

  //For Encoder
  roboclaw1.SetEncM1(address, 0);
  roboclaw1.SetEncM2(address, 0);
  
  //For Distance sensor connection
  pinMode(3, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(8, INPUT_PULLUP);
  
  Serial.begin(9600);
}

void loop() {
  //Begin Program
  LM.writeMicroseconds(1400); //Set servo to initial upright position
  delay(2000);
  LM.detach();  //Power down servo to prevent servo jitter.
 
  while(digitalRead(8) == HIGH); //Wait for Pushbutton to be pressed
  delay(5000);
  playTune();   //Play the tune, and flash the LED to warn the user to leave
  delay(10000); //Wait 10s for the user to leave the room

  //Begin scanning with distance sensors for height, and distance info
  const float initScanDistThreshold = 36; //If the distance is greater than this distance, 
                                          //it is safe to conclude that there is nothing on 
                                          //the turntable within its line of sight
  const float layerHeight = 5.5; //Increments by 5.5 centimeters per layer
  float maxLayers = 1;           //Max layers as determined by the initial scan
  float currentHeight = -3.5;    //Keeps track of the height of the Linear Actuator
  boolean hasInfDist = false;    //For detecting whether there is a package, or a collection 
                                 //of items.
                              
  //Scans for whether it is a package or a collection of items
  //Loops through increments of 5 degrees, and takes distance measurements with the distance 
  //Sensor for each increment. Using the measurement, it will determine whether it is a 
  //Collection of objects, or a package. A collection of items is when hasInfDist is true.
  for(int i = 0; i < 360; i++){
    setTurntablePos(i, 1);
    if(getFilteredLongDistance() > 36){ //if it detects a gap between two items, it 
                                        //determines that it is a collection of items
      hasInfDist = true;
    }
  }
  
  //This will give initial information about the
  //The Linear actuator will move the light up by increments of 5.5 centimeters until it no 
  //longer detects the box. Adjusts currentHeight to match Linear Actuator.Counts how many 
  //layers incremented so far
  while(getFilteredLongDistance() < initScanDistThreshold){ //scans, and increments layers 
                                                            //until it no longer detects the box.
    setLinearActuatorPos(currentHeight + layerHeight, 0.1); //Increments the Linear actuator by 
                                                            //layerHeight
    currentHeight += layerHeight; //Adjusts currentHeight to match Linear Actuator
    maxLayers++; //Counts how many layers incremented so far
    delay(500);
  }

 
  //Similar to the previous loop: it moves up until it no longer detects the box. However, 
  //in this loop, it turns the turntable 360 degrees to ensure all sides are checked, and
  //that there is nothing at that layer.
  while(true){
    boolean isClear = true; // True as long as distance sensor does not detect anything
    int initPos = getTurntablePos();
    for(int i = initPos; i < initPos + 360; i += 5){ // Checks every 5 degrees
      setTurntablePos(i, 1);
      if(getFilteredLongDistance() < initScanDistThreshold){
        isClear = false;
        break;
      }
    }
    if(isClear){ // If the layer is clear, it moves on to the next part of the program
      break;
    } else { // Otherwise, it goes up and scans another layer
      maxLayers++;
      setTurntablePos(0, 1); // Resets Turntable
      setLinearActuatorPos(getLinearActuatorPos() + layerHeight, 0.1); // Increments Linear Actuator height
      currentHeight += layerHeight; // Adjusts currentHeight to match Linear
      delay(500);
    }
  }

  // Sets Linear Actuator to 13cm for a horizontal scan
  setLinearActuatorPos(getLinearActuatorPos() - currentHeight + 13, 0.1);
  
  //Horizontal scan finds the minimum distance from the box to the light, this is used later 
  //for scanning the top of the box. The for loop loops through each 5 degree increment up 
  //to 360 degrees, and finds the minimum reading from the distance sensor for each increment
  int minDist = 100;
  int initPos = getTurntablePos();
  for(int i = initPos; i < initPos + 360; i += 5){ // Checks every 5 degrees
    setTurntablePos(i, 1);
    minDist = min(getFilteredDistance(), minDist); //Determines minimum distance
  }
  setLinearActuatorPos(getLinearActuatorPos() - 4, 0.1); // Sets the Linear actuator to 
                                                         // the first layer height


  //Pushes the button to turn on the UV-C lamp
  //detaches the servo to prevent servo jitter
  LM.attach(9);
  LM.writeMicroseconds(1450);
  delay(400);
  LM.writeMicroseconds(1370);
  delay(500);
  LM.detach();

  //Each disinfection layer height is twice the initial scanning layer Height
  float disinfectLayerHeight = layerHeight * 2; 
  //Because of above statement, the number of steps is the maxLayers / 2
  int numSteps = maxLayers / 2; 
  
  //This for loop loops numSteps times, since there are numSteps layers to be disinfected. 
  //The disinfectLayerHeight is 11.5cm since that is the length of the UV-C light bulb
  //On each layer, the robot will maintain a 8cm from the box
  for(int i = 0; i < numSteps; i++){
    int dps = 12; // Sets speed to 9
    float distFromObj = 8; //The robot will maintain a 8cm distance between the light 
                           //and the package
    if(!hasInfDist){ //If its just a collection of items, the light stays at the edge of 
                     //the turntable.
      while(getFilteredDistance() > distFromObj){ //Moves lamp up to object
        setRailPower(20);
      }
      setRailPower(0);
    } else {
      setRailPos(16, 0.49, 30, 15, 0.2); // Moves lamp up to turntable
    }
    delay(1000);

    float initTurntablePos = -getTurntablePos();
    //This loop turns the turntable 360 degrees (1 rotation), while maintaining an 8cm distance 
    //from the box. If it is collection of items, the light will just stay at the edge of the 
    //turntable.
    while(-getTurntablePos() < initTurntablePos + 360){ // Turns 360 degrees
      setTurntablePower(7);
      int power = 10 * (getFilteredDistance() - distFromObj);
      if(!hasInfDist){
        if(abs(power) < 35){
          setRailPower(power); // adjust the light position to maintain distance
        }
      } else {
        setRailPower(0); // Doesn't move light if it is a collection of objects
      }
    }
    setTurntablePower(0);
    delay(500);
    //Zero guiderail
    setRailPower(-15);
    delay(7000);
    zeroRail();
    setRailPower(0);
    //Increments the Linear Actuator by disinfectLayerHeight
    setLinearActuatorPos(getLinearActuatorPos() + disinfectLayerHeight - 2, 0.1); 
  }
  //Moves the linear Actuator above the package/items
  setLinearActuatorPos(getLinearActuatorPos() + 9, 0.1); 

  //Rotates the UV-C lamp horizontal
  LM.attach(9);
  LM.writeMicroseconds(500);
  delay(3000);
  LM.detach();

  int dps = 8; // Rotates the turntable slowly
  float initTurntablePos = -getTurntablePos();
  float power = 15;

  //If it is collection of items, the program moves the lamp up to the turntable, and if it is 
  //a box, it would use the minimum distance it determined in the initial scan to find out 
  //where to move
  if(hasInfDist){ // Moves the lamp above the object
    setRailPos(26, 0.49, 30, 15, 0.2);
  } else {
    if(minDist > 18){
      setRailPos(minDist + 3, 0.49, 30, 15, 0.2);
    } else {
      setRailPos(minDist + 3, 0.49, 30, 15, 0.2);
    }
  }
  
  //Rotates the turntable 360 degrees at 8 degrees/s
  while(-getTurntablePos() < initTurntablePos + 360){
    if(getTurntableSpeed() > dps){
     power--;
    } else if (getTurntableSpeed() < dps){
     power++;
    }
    setTurntablePower(power);
  }
  setTurntablePower(0);
  
  //Zero guiderail, back to original position
  setRailPower(-15);
  delay(7000);
  zeroRail();
  setRailPower(0);

  //Returns the lamp to its original position, and turns it off
  LM.attach(9);        
  LM.writeMicroseconds(1370);
  delay(3000);
  LM.writeMicroseconds(1450);
  delay(400);
  LM.writeMicroseconds(1300);
  delay(500);
  LM.writeMicroseconds(1370);
  LM.detach();
  
  //Resets the Linear Actuator
  roboclaw1.ForwardM1(address, 70);
  delay(30000);
  roboclaw1.ForwardM1(address, 0);
  
  //Plays tune, and flashes LED to tell the User to come back and pick up the package.
  playTune();
}

//Plays Twinkle Twinkle, and flashes the LED
void playTune(){
  digitalWrite(12, HIGH);
  play('c', 2);       
  digitalWrite(12, LOW);
  play('c', 2);       
  digitalWrite(12, HIGH);
  play('g', 2);       
  digitalWrite(12, LOW);
  play('g', 2);       
  digitalWrite(12, HIGH);
  play('a', 2);       
  digitalWrite(12, LOW);
  play('a', 2);       
  digitalWrite(12, HIGH);
  play('g', 4);
  digitalWrite(12, LOW);
  play(' ', 2);       
  digitalWrite(12, HIGH);
  play('f', 2);       
  digitalWrite(12, LOW);
  play('f', 2);       
  digitalWrite(12, HIGH);
  play('e', 2);       
  digitalWrite(12, LOW);
  play('e', 2);       
  digitalWrite(12, HIGH);
  play('d', 2);       
  digitalWrite(12, LOW);
  play('d', 2);       
  digitalWrite(12, HIGH);
  play('c', 4);  
  digitalWrite(12, LOW);
}

//Plays a note for a certain number of beats; This was copied from the Sparkfun Inventors Kit 
//Sample Code
void play( char note, int beats)
{
  int numNotes = 14;  // number of notes in our note and frequency array (there are 15 values, but arrays start at 0)

  //Note: these notes are C major (there are no sharps or flats)

  //this array is used to look up the notes
  char notes[] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C', 'D', 'E', 'F', 'G', 'A', 'B', ' '};
  //this array matches frequencies with each letter (e.g. the 4th note is 'f', the 4th frequency 
  //is 175)
  int frequencies[] = {131, 147, 165, 175, 196, 220, 247, 262, 294, 330, 349, 392, 440, 494, 0};

  //the frequency that we find when we look up a frequency in the arrays
  int currentFrequency = 0;
  //the length of one beat (changing this will speed up or slow down the tempo of the song)
  int beatLength = 150;   

  //look up the frequency that corresponds to the note
  for (int i = 0; i < numNotes; i++)  // check each value in notes from 0 to 14
  {
    if (notes[i] == note)  //does the letter passed to the play function match the letter 
                           //in the array?
    {
      currentFrequency = frequencies[i];   // Yes! Set the current frequency to match that note
    }
  }

  //play the frequency that matched our letter for the number of beats passed to the play 
  //function
  tone(3, currentFrequency, beats * beatLength);
  delay(beats * beatLength);  //wait for the length of the tone so that it has time to play
  delay(50);            //a little delay between the notes makes the song sound more natural

}
//For getting raw encoder outputs

//Returns encoder output for the Rail motor
int getRailEnc(){
  return (-roboclaw1.ReadEncM2(address)) % 4294967296;
}

//Returns encoder output for the Linear Actuator motor
int getLinearActuatorEnc(){
  return (roboclaw1.ReadEncM1(address)) % 4294967296;
}

//Returns encoder output for the Turntable motor
int getTurntableEnc(){
  return (LL.read());
}

//Returns encoder speed output for the Turntable motor in degrees/second
float getTurntableSpeed(){
  float init = getTurntableEnc();
  delay(15);
  float final = getTurntableEnc();
  return (abs(init - final) / 7.9020) / 0.015;
}

//For getting distances/degrees

//Returns the Rail position
float getRailPos(){
  return (float)(getRailEnc() / 24.0);
}

//Returns the Linear Actuator Position
float getLinearActuatorPos(){
  return (float)(getLinearActuatorEnc() / 650.0);
}

//Returns the Turntable position
float getTurntablePos(){
  return (float)(getTurntableEnc() / 7.9028);
}

//For setting power

/* 
 * setRailPower - Sets the Rail power to power var
 * Parameters:
 * power - power level of the rail motor, sign on power determines direction
 *///Sets the Rail power to power var: sign on power determines direction
void setRailPower(int power){
  if(power > 0){
    roboclaw1.ForwardM2(address, power);
  } else {
    roboclaw1.BackwardM2(address, -power);
  }
}

/*
 * setTurntablePower - sets the Turntable power to power
 * Parameters:
 * power - power level of the turntable motor, sign on power determines direction
 */
void setTurntablePower(int power){
  int initPosLinearActuator = roboclaw1.ReadEncM1(address);
  int initPosRail = roboclaw1.ReadEncM2(address);
  roboclaw2.begin(38400);
  if(power > 0){
    roboclaw2.ForwardM1(address, power);
  } else {
    roboclaw2.BackwardM1(address, -power);
  }
  roboclaw1.begin(38400);
  delay(10);
  roboclaw1.SetEncM1(address, initPosLinearActuator);
  roboclaw1.SetEncM2(address, initPosRail);
}

//For setting position

/*
 * setRailPositionForeward - sets the rail position forward to pos, within a given accuracy
 * Parameters:
 * pos - target position
 * fracSplit - fraction of route used for acceleration/deceleration
 * maxPower - maximum power
 * minPower - minimum power
 * accuracy - rail goes within +- accuracy of the target.
 */
void setRailPositionForeward(float pos, float fracSplit, int maxPower, int minPower, float accuracy){
  if(fracSplit > 0.5) return;
  float initPos = getRailPos();
  float distTrav = pos - initPos;
  float checkPoint1 = initPos + distTrav * fracSplit;
  float checkPoint2 = pos - distTrav * fracSplit;

  //Loop adjusts the power of the rail until it is within +- accuracy of the target position
  while(abs(pos - getRailPos()) > accuracy){
    
    boolean segment0 = (getRailPos() < initPos);
    boolean segment1 = (getRailPos() >= initPos && getRailPos() < checkPoint1);
    boolean segment2 = (getRailPos() >= checkPoint1 && getRailPos() < checkPoint2);
    boolean segment3 = (getRailPos() >= checkPoint2 && getRailPos() < pos);
    boolean segment4 = getRailPos() > pos;

    if(segment0){
      setRailPower(minPower);
    }
    if(segment1){
      int addPower  = (maxPower - minPower) * sqrt((getRailPos() - initPos) / (checkPoint1 - initPos));
      setRailPower(minPower + addPower);
    }
    if(segment2){
      setRailPower(maxPower);
    }
    if(segment3){
      int addPower = (maxPower) * sqrt((pos - getRailPos()) / (pos - checkPoint2));
      setRailPower(minPower + addPower);
    }
    if(segment4){
      setRailPower(-minPower);
    }
  }
  setRailPower(0);
}

/*
 * setRailPositionBackward sets the rail position backward to pos, within a given accuracy
 * Parameters:
 * pos - target position
 * fracSplit - fraction of route used for acceleration/deceleration
 * maxPower - maximum power
 * minPower - minimum power
 * accuracy - rail goes within +- accuracy of the target.
 */
void setRailPositionBackward(float pos, float fracSplit, int maxPower, int minPower, float accuracy){
  if(fracSplit > 0.5) return;
  float initPos = getRailPos();
  float distTrav = initPos - pos;
  float checkPoint1 = initPos - distTrav * fracSplit;
  float checkPoint2 = pos + distTrav * fracSplit;
  
  //Loop adjusts the power of the rail until it is within +- accuracy of the target position
  while(abs(pos - getRailPos()) > accuracy){
    boolean segment0 = (getRailPos() > initPos);
    boolean segment1 = (getRailPos() <= initPos && getRailPos() > checkPoint1);
    boolean segment2 = (getRailPos() <= checkPoint1 && getRailPos() > checkPoint2);
    boolean segment3 = (getRailPos() <= checkPoint2 && getRailPos() > pos);
    boolean segment4 = getRailPos() < pos;
    if(segment0){
      setRailPower(- minPower);
    }
    if(segment1){
      int addPower  = (maxPower - minPower) * sqrt((getRailPos() - initPos) / (checkPoint1 - initPos));
      setRailPower(- minPower - addPower);
    }
    if(segment2){
      setRailPower(- maxPower);
    }
    if(segment3){
      int addPower = (maxPower) * sqrt((pos - getRailPos()) / (pos - checkPoint2));
      setRailPower(- minPower - addPower);
    }
    if(segment4){
      setRailPower(minPower);
    }
  }
  setRailPower(0);
}

/*
 * setRailPos - sets the rail position to pos, within a set accuracy
 * Parameters:
 * pos - target position  
 * fracSplit - fraction of route used for acceleration/deceleration
 * maxPower - maximum power
 * minPower - minimum power  
 * accuracy - rail goes within +- accuracy of the target.
*/
void setRailPos(float pos, float fracSplit, int maxPower, int minPower, float accuracy){
  if(pos < getRailPos()){
    setRailPositionBackward(pos, fracSplit, maxPower, minPower, accuracy);
  } else {
    setRailPositionForeward(pos, fracSplit, maxPower, minPower, accuracy);
  }
}

/*
 * setLinearActuatorPos - sets Linear Actuator position to pos within a set accuracy.
 * Parameters:
 * pos - target position
 * accuracy - Linear Actuator goes within +- accuracy of the target.
 */
void setLinearActuatorPos(float pos, float accuracy){
  while(abs(getLinearActuatorPos() - pos) > accuracy){
    if(getLinearActuatorPos() < pos){
      roboclaw1.BackwardM1(address, 70);
    } else {
      roboclaw1.ForwardM1(address, 70);
     
    }
  }
  roboclaw1.BackwardM1(address, 0);
}

/*
 * setTurntablePos - sets Turntable to position to pos within a set accuracy.
 * Parameters:
 * pos - target position
 * accuracy - Turntable goes within +- accuracy of the target.
 */
void setTurntablePos(float pos, float accuracy){
  int initPosLinearActuator = roboclaw1.ReadEncM1(address);
  int initPosRail = roboclaw1.ReadEncM2(address);
  roboclaw2.begin(38400);
  while(abs(getTurntablePos() - pos) > accuracy){
    if(getTurntablePos() < pos){
      roboclaw2.BackwardM1(address, 30);
    } else {
      roboclaw2.ForwardM1(address, 30);
    }
    //Serial.println(getTurntablePos());
  }
  roboclaw2.BackwardM1(address, 0);
  roboclaw1.begin(38400);
  roboclaw1.SetEncM1(address, initPosLinearActuator);
  roboclaw1.SetEncM2(address, initPosRail);
}

//Distance sensor outputs

//Returns raw output for short range distance sensor
float getDistance(){
  float volt = (float)(analogRead(A0))/1023.0 * 5.0;
  //return volt;
  return (13.02)/(volt) - 0.42;
}

//Returns average of 10 distance sensor measurements
float getFilteredDistance(){
  float sum = 0;
  int numSamples = 10;
  for(int i = 0; i < numSamples; i++){
    float dist = getDistance();
    sum += dist;
  }

  return sum / numSamples;
}

//Returns average of 10 distance sensor measurements
float getLongDistance(){
  return (6762.0 /((float) analogRead(A1) - 9.0)) - 4;
}

//Returns raw output for short range distance sensor
float getFilteredLongDistance(){
  float sum = 0;
  int numSamples = 10;
  for(int i = 0; i < numSamples; i++){
    float dist = getLongDistance();
    sum += dist;
  }
  return sum / numSamples;
}

//Zeros the Rail encoder
void zeroRail(){
  roboclaw1.SetEncM2(address, 0);
}
