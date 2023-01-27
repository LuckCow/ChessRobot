#include <LobotServoController.h>
#include <InverseK.h>

#define rxPin 10
#define txPin 11

SoftwareSerial mySerial(rxPin, txPin);
LobotServoController myse(mySerial);   


bool is_connected = false; ///< True if the connection with the master is available
bool DEBUG = false;
const unsigned int MAX_MESSAGE_LENGTH = 12;
bool move_avail = false;

// coordinate array: x1 y1 z1, x2, y2, z2
float coords[6];

void setup() {
  Serial.begin(9600);

  pinMode(13,OUTPUT);
  mySerial.begin(9600);  //opens software serial port, sets data rate to 9600 bps
  Serial.begin(9600);
  digitalWrite(13,HIGH);

  // inverse kinematics configuration given the coordinates x and y and the list of lengths for the arm.
  Link base, upperarm, forearm, hand;

  base.init(0, l2a(-90.0), l2a(90.0));
  upperarm.init(105, l2a(-90.0), l2a(90.0));
  forearm.init(90, l2a(-90.0), l2a(90.0));
  hand.init(180, l2a(-90.0), l2a(90.0));

  // Attach the links to the inverse kinematic model
  InverseK.attach(base, upperarm, forearm, hand);

}

void loop() {
  
 //Check to see if anything is available in the serial receive buffer
 while (Serial.available() > 0) {  //TODO: remove this at some point

    //Read the next available byte in the serial receive buffer
    char order = Serial.read();

    switch (order) {
      case 'a':  // hello
        Serial.println("Hello World!");
        break;
      case 'b':  // move to
        // read in 3 coordinates (3 coords * 7 bytes/coord = 21)
        // Serial buffer is 64 bytes by default
        // Max coord length is 6 chars (example: -234.2)
        wait_for_bytes(21, 500);

        read_serial_coords();

        if (DEBUG) {
          Serial.print("Move To Command: ");
          Serial.print(coords[0]); Serial.print(' ');
          Serial.print(coords[1]); Serial.print(' ');
          Serial.print(coords[2]); Serial.print('\n');
        }

        move_to(coords[0], coords[1], coords[2]);

        Serial.print("z");

        break;
      case 'c':  // move piece
        // TODO: clean up communication so that it doesn't need to be padded. Raw bytes would be more efficient than strings
        // read in 6 coordinates (6*7)
        wait_for_bytes(42, 500);

        read_serial_coords();

        if (DEBUG) {
          Serial.print("Move Piece Command: ");
          Serial.print(coords[0]); Serial.print(' ');
          Serial.print(coords[1]); Serial.print(' ');
          Serial.print(coords[2]); Serial.print(", ");
          Serial.print(coords[3]); Serial.print(' ');
          Serial.print(coords[4]); Serial.print(' ');
          Serial.print(coords[5]); Serial.print('\n');
        }

        move_piece(coords[0], coords[1], coords[2], coords[3], coords[4], coords[5]);

        Serial.print("z");

        break;

      case 'd':  //open
        if (DEBUG) {
          Serial.println("Opening Gripper");
        }
        open_gripper();
        Serial.print("z");
        break;
      case 'e': //close
        if (DEBUG) {
          Serial.println("Closing Gripper");
        }
        close_gripper();
        Serial.print("z");
        break;

      default:
        Serial.print("Error: command not known: '");
        Serial.print(order);Serial.println("'");
        break;
    }
 }
 
}

void read_serial_coords(){
  //Create a place to hold the incoming message
  char message[MAX_MESSAGE_LENGTH];
  unsigned int message_pos = 0;
  unsigned int coord_pos = 0;

  // read in bytes and convert to floats
  while (Serial.available() > 0) {
    char inByte = Serial.read();
    // check for coord separators or the end
    if (inByte == ' ' || inByte == '\n'){
      // add end char for string
      message[message_pos] = '\0';

      //add into coord array
      coords[coord_pos] = atof(message);
      coord_pos += 1;

      //Reset for the next message
      message_pos = 0;
    }
    else {
      //Add the incoming byte to coord string
      message[message_pos] = inByte;
      message_pos++;
    }
  }
}

// Quick conversion from radians to the Braccio angle system
float a2b(float a) {
  return (a + HALF_PI) * 180 / PI;
}

// convert LeArm angle to radians system
float l2a(float b){
  return b / 180.0 * PI;
}

// convert radians to LeArm angle system
float a2l(float a) {
  return 636.62 * a + 1500;
}

void debug_angles(float a0, float a1, float a2, float a3) {
    Serial.print("degree: ");
    Serial.print(a2b(a0)); Serial.print(',');
    Serial.print(a2b(a1)); Serial.print(',');
    Serial.print(a2b(a2)); Serial.print(',');
    Serial.println(a2b(a3));

    // raw radians
    // Serial.print("radian: ");
    // Serial.print(a0); Serial.print(',');
    // Serial.print(a1); Serial.print(',');
    // Serial.print(a2); Serial.print(',');
    // Serial.println(a3);

    Serial.print("servos: ");
    Serial.print(a2l(a0)); Serial.print(',');
    Serial.print(3000 - a2l(a1)); Serial.print(',');
    Serial.print(a2l(a2)); Serial.print(',');
    Serial.println(a2l(a3));
    Serial.println("");
}

void move_piece(float x1, float y1, float z1, float x2, float y2, float z2) {
  // Moves to point 1, grabs piece, moves to point 2, release piece

  float z_offset = 80; // for when moving peice, do so 80mm above the board to avoid knocking things over

  move_to(x1, y1, z1+z_offset);
  open_gripper();
  move_to(x1, y1, z1);
  close_gripper();
  move_to(x1, y1, z1+z_offset);
  move_to(x2, y2, z2+z_offset);
  move_to(x2, y2, z2);
  open_gripper();
  move_to_idle();
  //TODO: unload servos
}

int move_time_ms = 750;

void move_to(float x, float y, float z){

  // Angle variables to put result into
  float a0, a1, a2, a3;
  float theta6, theta5, theta4, theta3;

  // InverseK.solve() return true if it could find a solution and false if not.

  // Calculates the angles without considering a specific approach angle
  if(InverseK.solve(x, y, z, a0, a1, a2, a3)) {  //-PI/4

    if (DEBUG){
      debug_angles(a0, a1, a2, a3);
    }
    

    theta6 = a2l(a0);
    theta5 = 3000 - a2l(a1);   // this servo is backwards, lol
    theta4 = a2l(a2);
    theta3 = a2l(a3);

    
  } else {
    Serial.println("No solution found!");
  }

  LobotServo servos[4];   //an array of struct LobotServo
  servos[0].ID = 6;
  servos[0].Position = theta6;
  servos[1].ID = 5;
  servos[1].Position = theta5;
  servos[2].ID = 4;
  servos[2].Position = theta4;
  servos[3].ID = 3;
  servos[3].Position = theta3;
  myse.moveServos(servos,4,move_time_ms);  //control 2 servos, action time is 5000ms, ID and position are specified by the structure array "servos"
  delay(move_time_ms+250);
  //myse.waitForStopping(10000);
}

void move_to_idle(){
  LobotServo servos[4];   //an array of struct LobotServo
  servos[0].ID = 6;
  servos[0].Position = 1500;
  servos[1].ID = 5;
  servos[1].Position = 1500;
  servos[2].ID = 4;
  servos[2].Position = 1500;
  servos[3].ID = 3;
  servos[3].Position = 1500;
  myse.moveServos(servos,4,move_time_ms);  //control 2 servos, action time is 5000ms, ID and position are specified by the structure array "servos"
  delay(move_time_ms+250);
  //myse.waitForStopping(10000);

}

void open_gripper(){
  // Gripper open
  myse.moveServo(1,1500,500); //Open Gripper
  delay(750);
  //myse.waitForStopping(10000);
}

void close_gripper(){
  // Gripper close
  myse.moveServo(1,2400,500); //Close Gripper
  delay(750);
  //myse.waitForStopping(10000);
}

void wait_for_bytes(int num_bytes, unsigned long timeout)
{
	unsigned long startTime = millis();
	//Wait for incoming bytes or exit if timeout
	while ((Serial.available() < num_bytes) && (millis() - startTime < timeout)){}
}
