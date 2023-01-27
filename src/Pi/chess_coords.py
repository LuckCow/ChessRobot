import serial
import time

# Measurements
# robot
height_robot = 82  # ~1mm
ROBOT_TO_BOARD = 83 + 15

# Board
SQUARE_SIZE = 30.4  # 27 measured
board = 220  # has 1mm boarder and 1mm fold in middle
height_board = 22  # from table

# piece height
height_pawn = 22
height_queen = 42

HEIGHT_OFFSET = height_board - height_robot

# Height offsets
GRAB_OFFSET = 10  # general purpose low grab height - can be adapted per piece for refining
APPROACH_OFFSET = 70

# Capture Area

def get_coord(square, side=True):
    """
    returns coordinate of the middle of the square given
    the origin of the coordinate system is the middle of the robot's shoulder base, with the robot being in the middle of the board for y, and sitting on one side of the board offset in the x direction, with z being the height

    """
    # for white, A1 is bottom left. for black, A1 is top right
    square = square.upper()
    col = ord(square[0]) - ord('A')  # Column is the letter of square
    row = int(square[1]) - 1  # row is number

    # flip rows+cols if on black side
    if not side:
        col = 7 - col
        row = 7 - row

    x = -(SQUARE_SIZE / 2.0 + row * SQUARE_SIZE + ROBOT_TO_BOARD)
    y = -SQUARE_SIZE*4 + (SQUARE_SIZE / 2.0 + col * SQUARE_SIZE)

    # For some reason, the IK height is sloped so that it is lower with higher X values.
    # Slope is x -98.2, z +65 to x -311.0, z +10
    # 55 / (-98.2 + 311) = 0.25846

    z = HEIGHT_OFFSET - 0.25846 * (x + 98.2)

    return x, y, z


class RobotArm:

    def __init__(self, serial_port='COM4', timeout=0, chess_side=True):
        self.serial_port = serial_port
        self.timeout = timeout
        self.chess_side = chess_side

    def connect(self):
        """ Initiate serial communication with arduino """
        self.arduino = serial.Serial(port='COM4', timeout=0)
        time.sleep(2)

    def move_piece(self, square1, square2):
        """
        TODO: 'XX' means capture and the piece will be moved to capture zone
        `move x1 y1 z1 x2 y2 z2` moves chess piece from point 1 to point 2
        """
        x1, y1, z1 = get_coord(square1, self.chess_side)
        x2, y2, z2 = get_coord(square2, self.chess_side)

        serial_command = f'c{x1:.1f} {y1:.1f} {z1 + GRAB_OFFSET:.1f} {x2:.1f} {y2:.1f} {z2 + GRAB_OFFSET:.1f} '
        serial_command += '|' * (43 - len(serial_command))  # pad message so length is known for reading

        print(f"{square1} to {square2} {serial_command}")
        self.arduino.write(str.encode(serial_command))

        # TODO: wait for response
        #self.arduino.read()

        time.sleep(30)

    def move_to(self, square):
        x1, y1, z1 = get_coord(square, self.chess_side)
        serial_command = f'b{x1:.1f} {y1:.1f} {z1 + APPROACH_OFFSET:.1f} '
        serial_command += '|' * (22 - len(serial_command))

        print(square, serial_command)
        self.arduino.write(str.encode(serial_command))
        time.sleep(2.5)

    def close_gripper(self):
        self.arduino.write(str.encode('e'))

    def open_gripper(self):
        self.arduino.write(str.encode('d'))


if __name__ == '__main__':
    get_coord('A1')
    get_coord('H8')

    robot = RobotArm(serial_port='COM4', timeout=0, chess_side=False)

    robot.connect()

    robot.move_to('E5')
    robot.move_to('A8')
    robot.move_to('A1')
    robot.move_to('H8')
    robot.move_to('H1')

    robot.move_to('D4')
    robot.move_to('C3')
    robot.move_to('B2')

    robot.move_to('D1')
    robot.move_to('D2')
    robot.move_to('D3')
    robot.move_to('D4')

    # 4 corner calibration
    # robot.close_gripper()
    # robot.move_to('A1')
    # robot.move_to('A8')
    # robot.move_to('H1')
    # robot.move_to('H8')

    #robot.move_piece('D5', 'D4')
    #robot.move_piece('H1', 'H2')
    #robot.move_piece('A1', 'A2')
    #robot.move_piece('A8', 'A7')
