#include <LobotServoController.h>
#include <InverseK.h>

#define rxPin 10
#define txPin 11

SoftwareSerial mySerial(rxPin, txPin);
LobotServoController myse(mySerial);   


bool is_connected = false; ///< True if the connection with the master is available
bool DEBUG = true;


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

  //move testing
  // move_to_idle();
  // // open_gripper();
  // Serial.println("Move 1: -120 z");
  // move_to(x, y, -120);
  // Serial.println("Move 2: +0 z");
  // move_to(x, y, -60);
  // // close_gripper();
  // move_to_idle();

  //coordinate testing
  // Serial.println("c1 -292, 0, 202");
  // move_to(-292, 0, -50);

  // //x and z are inversed
  // Serial.println("c3 -200, 100, 100");
  // move_to(-200, 100, 100);
  // Serial.println("c4 -200, 0, 0");
  // move_to(-200, 0, 0);

  // Serial.println("c5 -250, 0, 0");
  // move_to(-250, 0, 0);

}

const unsigned int MAX_MESSAGE_LENGTH = 12;

// coordinate array: x1 y1 z1, x2, y2, z2
float coords[6];
bool move_avail = false;

// Serial buffer is 64 bytes by default
// Max coord length is 6 chars (example: -234.2)

void loop() {
  
  // move_to(x, y, z+80);
  // open_gripper();
  // move_to(x, y, z);
  // close_gripper();
  
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
        wait_for_bytes(21, 500);

        read_serial_coords();

        if (DEBUG) {
          Serial.print("Move To Command: ");
          Serial.print(coords[0]); Serial.print(' ');
          Serial.print(coords[1]); Serial.print(' ');
          Serial.print(coords[2]); Serial.print('\n');
        }

        move_to(coords[0], coords[1], coords[2]);

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

        break;

      case 'd':  //open
        if (DEBUG) {
          Serial.println("Opening Gripper");
        }
        open_gripper();
        break;
      case 'e': //close
        if (DEBUG) {
          Serial.println("Closing Gripper");
        }
        close_gripper();
        break;

      default:
        Serial.print("Error: command not known: '");
        Serial.print(order);Serial.println("'");
        break;
    }

    // if (message_pos >= MAX_MESSAGE_LENGTH) {
    //   Serial.print("Critical serial read error"); 
    //   break;
    // }
      
    // // check for coord separators or the end
    // if (inByte == ' ' || inByte == '\n'){
    //   // add end char for string
    //   message[message_pos] = '\0';

    //   //add into coord array
    //   coords[coord_pos] = atoi(message);
    //   coord_pos += 1;

    //   // mark message as complete for move
    //   if (coord_pos >= 3){
    //     move_avail = true;
    //     coord_pos = 0;
    //   }

    //   //Reset for the next message
    //   message_pos = 0;
    // } 
    // else {
    //   //Add the incoming byte to coord string
    //   message[message_pos] = inByte;
    //   message_pos++;
    // }
 }

 // move to entered coords
//  if (move_avail) {
//    Serial.print("Serial Move xyz: "); 
//    Serial.print(coords[0]); Serial.print(' ');
//    Serial.print(coords[1]); Serial.print(' ');
//    Serial.print(coords[2]); Serial.print('\n');
//    move_to(coords[0], coords[1], coords[2]);
//    move_avail = false;
//  }
 
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
  // InverseK.solve(x, y, z, a0, a1, a2, a3)

  if(InverseK.solve(x, y, z, a0, a1, a2, a3)) {  //-PI/4

    debug_angles(a0, a1, a2, a3);

    theta6 = a2l(a0);
    theta5 = 3000 - a2l(a1);   // this servo is backwards, lol
    theta4 = a2l(a2);
    theta3 = a2l(a3);

    
  } else {
    Serial.println("No solution found!");
  }

  // Gripper
  // myse.moveServo(1,1500,1000); //Open Gripper
  // delay(5000);
  // myse.moveServo(1,2400,1000); //Close Gripper
  // //delay(10000);
  // delay(3000);
  // myse.unloadServos(1, 1);


  // myse.moveServo(2,800,1000); //move No.2 servo in 1000ms to 800 position
  // delay(2000);
  // myse.moveServos(5,1000,0,1300,2,700,4,600,6,900,8,790); 
  // //Control 5 servos, action time is 1000ms, move No.0 servo to 1300 position, move No.2 servo to 700 position, move No.4 servo to 600 position
  // //Move No.6 servo to 900 position, move No.8 servo to 790 position
  // delay(2000);

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


// // Define the orders that can be sent and received
// enum Order {
//   HELLO = 0,
//   MOVE_TO = 1,
//   MOVE_PIECE = 2,
//   ALREADY_CONNECTED = 3,
//   ERROR = 4,
//   RECEIVED = 5,
//   STOP = 6,
// };

// typedef enum Order Order;

// void get_messages_from_serial()
// {
//   if(Serial.available() > 0)
//   {
//     // The first byte received is the instruction
//     Order order_received = read_order();

//     if(order_received == HELLO)
//     {
//       // If the cards haven't say hello, check the connection
//       if(!is_connected)
//       {
//         is_connected = true;
//         write_order(HELLO);
//       }
//       else
//       {
//         // If we are already connected do not send "hello" to avoid infinite loop
//         write_order(ALREADY_CONNECTED);
//       }
//     }
//     else if(order_received == ALREADY_CONNECTED)
//     {
//       is_connected = true;
//     }
//     else
//     {
//       switch(order_received)
//       {
//         case STOP:
//         {
//           motor_speed = 0;
//           stop();
//           if(DEBUG)
//           {
//             write_order(STOP);
//           }
//           break;
//         }
//         case MOVE:
//         {
//           servo_angle = read_i16();
//           if(DEBUG)
//           {
//             write_order(SERVO);
//             write_i16(servo_angle);
//           }
//           break;
//         }
//         case SPARE:
//         {
//           break;
//         }
//   			// Unknown order
//   			default:
//           write_order(ERROR);
//           write_i16(404);
//   				return;
//       }
//     }
//     write_order(RECEIVED); // Confirm the reception
//   }
// }

// Order read_order()
// {
// 	return (Order) Serial.read();
// }

void wait_for_bytes(int num_bytes, unsigned long timeout)
{
	unsigned long startTime = millis();
	//Wait for incoming bytes or exit if timeout
	while ((Serial.available() < num_bytes) && (millis() - startTime < timeout)){}
}

// serial read functions from https://github.com/araffin/arduino-robust-serial/blob/master/arduino-board/slave.cpp
// NOTE : Serial.readBytes is SLOW
// this one is much faster, but has no timeout
void read_signed_bytes(int8_t* buffer, size_t n)
{
	size_t i = 0;
	int c;
	while (i < n)
	{
		c = Serial.read();
		if (c < 0) break;
		*buffer++ = (int8_t) c; // buffer[i] = (int8_t)c;
		i++;
	}
}

float read_float()
{
  return (float) read_i32();
}

int8_t read_i8()
{
	wait_for_bytes(1, 100); // Wait for 1 byte with a timeout of 100 ms
  return (int8_t) Serial.read();
}

int16_t read_i16()
{
  int8_t buffer[2];
	wait_for_bytes(2, 100); // Wait for 2 bytes with a timeout of 100 ms
	read_signed_bytes(buffer, 2);
  return (((int16_t) buffer[0]) & 0xff) | (((int16_t) buffer[1]) << 8 & 0xff00);
}

int32_t read_i32()
{
  int8_t buffer[4];
	wait_for_bytes(4, 200); // Wait for 4 bytes with a timeout of 200 ms
	read_signed_bytes(buffer, 4);
  return (((int32_t) buffer[0]) & 0xff) | (((int32_t) buffer[1]) << 8 & 0xff00) | (((int32_t) buffer[2]) << 16 & 0xff0000) | (((int32_t) buffer[3]) << 24 & 0xff000000);
}

// void write_order(enum Order myOrder)
// {
// 	uint8_t* Order = (uint8_t*) &myOrder;
//   Serial.write(Order, sizeof(uint8_t));
// }

void write_i8(int8_t num)
{
  Serial.write(num);
}

void write_i16(int16_t num)
{
	int8_t buffer[2] = {(int8_t) (num & 0xff), (int8_t) (num >> 8)};
  Serial.write((uint8_t*)&buffer, 2*sizeof(int8_t));
}

void write_i32(int32_t num)
{
	int8_t buffer[4] = {(int8_t) (num & 0xff), (int8_t) (num >> 8 & 0xff), (int8_t) (num >> 16 & 0xff), (int8_t) (num >> 24 & 0xff)};
  Serial.write((uint8_t*)&buffer, 4*sizeof(int8_t));
}
