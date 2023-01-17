#include <LobotServoController.h>
#include <InverseK.h>

#define rxPin 10
#define txPin 11

SoftwareSerial mySerial(rxPin, txPin);
LobotServoController myse(mySerial);   


void setup() {
  Serial.begin(9600);

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

  //middle coord: (200, 0, 300)
  double x = 200;
  double y = 0;
  double z = -20;
  
  // Inverse Kinematics

  // //solve for plane rotation angle
  // double theta6 = atan(y/x) * 318.31 + 1500; 

  // // feed into 2D kinematics
  // double fy = z;
  // double fx = sqrt(pow(x,2) + pow(y,2));

  // Solve inverse kinematics given the coordinates x and y and the list of lengths for the arm.
  Link base, upperarm, forearm, hand;


  base.init(0, b2a(0.0), b2a(180.0));
  upperarm.init(105, b2a(0.0), b2a(180.0));
  forearm.init(90, b2a(0.0), b2a(180.0));
  hand.init(180, b2a(0.0), b2a(180.0));

  // Attach the links to the inverse kinematic model
  InverseK.attach(base, upperarm, forearm, hand);

  float a0, a1, a2, a3;

  // InverseK.solve() return true if it could find a solution and false if not.

  // Calculates the angles without considering a specific approach angle
  // InverseK.solve(x, y, z, a0, a1, a2, a3)
  if(InverseK.solve(x, y, z, a0, a1, a2, a3)) {
    Serial.print(a2b(a0)); Serial.print(',');
    Serial.print(a2b(a1)); Serial.print(',');
    Serial.print(a2b(a2)); Serial.print(',');
    Serial.println(a2b(a3));

    Serial.print(a0); Serial.print(',');
    Serial.print(a1); Serial.print(',');
    Serial.print(a2); Serial.print(',');
    Serial.println(a3);

    Serial.print(a2l(a0)); Serial.print(',');
    Serial.print(a2l(a1)); Serial.print(',');
    Serial.print(a2l(a2)); Serial.print(',');
    Serial.println(a2l(a3));
  } else {
    Serial.println("No solution found!");
  }

  // Get the angles (in radians [-pi,pi]) and convert them to servo degrees [500, 2500]
  // to convert from radians to servo degrees,  servodeg = rad * 1000 / pi + 1500
  // double theta5 = fabrik2D.getAngle(0) * 318.31 + 1500;
  // double theta4 = fabrik2D.getAngle(1) * 318.31 + 1500;
  // double theta3 = fabrik2D.getAngle(2) * 318.31 + 1500;


  // // Angles are printed in degrees.
  // // The function calls below shows how easy it is to get the results from the inverse kinematics solution.
  // Serial.print(atan(y/x)* 57296 / 1000);  // theta5
  // Serial.print("\t");


  // Serial.print("\n");
  // Serial.print(theta6);  // theta6
  // Serial.print("\t");
  // Serial.print(theta5);  // theta5
  // Serial.print("\t");
  // Serial.print(theta4);  // theta4
  // Serial.print("\t");
  // Serial.print(theta3);  // theta3
  // Serial.print("\n\n");


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

  // LobotServo servos[4];   //an array of struct LobotServo
  // servos[0].ID = 6;
  // servos[0].Position = theta6;
  // servos[1].ID = 5;
  // servos[1].Position = theta5;
  // servos[2].ID = 4;
  // servos[2].Position = theta4;
  // servos[3].ID = 3;
  // servos[3].Position = theta3;
  // myse.moveServos(servos,4,5000);  //control 2 servos, action time is 1000ms, ID and position are specified by the structure array "servos"


}

void loop() {
}

// Quick conversion from the Braccio angle system to radians
float b2a(float b){
  return b / 180.0 * PI - HALF_PI;
}

// Quick conversion from radians to the Braccio angle system
float a2b(float a) {
  return (a + HALF_PI) * 180 / PI;
}

float a2l(float a) {
  return 636.62 * (a + HALF_PI) + 500;
}
