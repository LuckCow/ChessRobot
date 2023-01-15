#include <LobotServoController.h>
#include <FABRIK2D.h>

#define rxPin 10
#define txPin 11

SoftwareSerial mySerial(rxPin, txPin);
LobotServoController myse(mySerial);   

int lengths[] = {105, 90, 180}; // 3DOF arm where shoulder to elbow is 105mm and elbow to end effector is 180mm.
Fabrik2D fabrik2D(4, lengths); // This arm has 3 joints; one in the origin, the shoudler, elbow and the end effector.

void setup() {
  Serial.begin(9600);
  fabrik2D.setTolerance(0.5);

  pinMode(13,OUTPUT);
  mySerial.begin(9600);  //opens software serial port, sets data rate to 9600 bps
  Serial.begin(9600);
  digitalWrite(13,HIGH);

  // myse.runActionGroup(100,0);  //loop run No.100 action group
  // delay(5000);
  // myse.stopActionGroup(); //stop running action group
  // delay(2000);
  // myse.setActionGroupSpeed(100,200); //Set the running speed of No.100 action group at 200%
  // delay(2000);
  // myse.runActionGroup(100,5);  //run No.100 action group 5 times 
  // delay(5000);
  // myse.stopActionGroup(); //stop running action group
  // delay(2000);

  
  // Inverse Kinematics

  //solve for plane rotation angle
  theta6 = arctan(y/x)  //TODO; convert to servo positions

  // feed into 2D kinematics
  fy = z
  fx = sqrt(x**2 + y**2)

  // Solve inverse kinematics given the coordinates x and y and the list of lengths for the arm.
  fabrik2D.solve(fx,fy,lengths);


  // Angles are printed in degrees.
  // The function calls below shows how easy it is to get the results from the inverse kinematics solution.
  Serial.print(fabrik2D.getAngle(0)* 57296 / 1000);  // theta5
  Serial.print("\t");
  Serial.print(fabrik2D.getAngle(1)* 57296 / 1000);
  Serial.print("\t");
  Serial.print(fabrik2D.getAngle(2)* 57296 / 1000);

  // myse.moveServo(1,1500,1000); //move No.0 Servo in 1000ms to 1500 position
  // delay(5000);
  // myse.moveServo(1,2400,1000); //move No.2 servo in 1000ms to 800 position
  // //delay(10000);
  // delay(3000);
  // myse.unloadServos(1, 1);








  // myse.moveServo(2,800,1000); //move No.2 servo in 1000ms to 800 position
  // delay(2000);
  // myse.moveServos(5,1000,0,1300,2,700,4,600,6,900,8,790); 
  // //Control 5 servos, action time is 1000ms, move No.0 servo to 1300 position, move No.2 servo to 700 position, move No.4 servo to 600 position
  // //Move No.6 servo to 900 position, move No.8 servo to 790 position
  // delay(2000);

  // LobotServo servos[2];   //an array of struct LobotServo
  // servos[0].ID = 2;       //No.2 servo
  // servos[0].Position = 1400;  //1400 position
  // servos[1].ID = 4;       //No.4 servo
  // servos[1].Position = 700;  //700 position
  // myse.moveServos(servos,2,1000);  //control 2 servos, action time is 1000ms, ID and position are specified by the structure array "servos"


}

void loop() {
}
