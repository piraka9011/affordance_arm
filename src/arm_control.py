#!/usr/bin/env python

import sys
import serial
import thread

import rospy
from dynamixel_constants import *
from dynamixel_workbench_msgs.msg import DynamixelState, DynamixelStateList
from sensor_msgs.msg import  JointState


class ArmControl:
    def __init__(self):
        # Variables
        self.port = rospy.get_param('/robotis_arm/port', '/dev/ttyUSB0')
        self.baud = rospy.get_param('/robotis_arm/baud', 115200)
        self.error = 0
        self.servo_count = 0
        self._mutex = thread.allocate_lock()
        self._baud_codes = {1000000: 1,
                            500000: 3,
                            400000: 4,
                            250000: 7,
                            200000: 9,
                            115200: 16,
                            57600: 34,
                            19200: 103,
                            9600: 207}
        rospy.logdebug("Params:\n"
                       "Port: {}\nBaud: {}\n".format(self.port, self.baud))
        # Serial setup
        try:
            self._ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.1)
            rospy.loginfo("Arbotix Connected! Connection is {}".format(self._ser.is_open))
        except serial.SerialException as e:
            self._ser.close()
            rospy.logwarn("Got a Serial exception!\n{}".format(e))

        # ROS
        self._dyna_state_pub = rospy.Publisher('/turtlebot_arm/dynamixel_state', DynamixelStateList, queue_size=10)
        self._joint_state_pub = rospy.Publisher('/turtlebot_arm/joint_state', JointState, queue_size=10)
        rospy.Subscriber('/turtlebot_arm/position_cmd', JointState, self._position_cmd_cb)
        self._arm_joint_state = JointState()
        self._arm_joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'gripper']

    def __del__(self):
        rospy.loginfo("Closing Arbotix serial connection...")
        self._ser.close()


    # ======================================================= #
    #           Serial Connection Utility Functions           #
    # ======================================================= #
    def __write__(self, msg):
        try:
            self._ser.write(msg)
        except serial.SerialException as e:
            self._mutex.release()
            raise serial.SerialException(e)

    def openPort(self):
        self._ser.close()
        try:
            self._ser.open()
        except serial.SerialException as e:
            raise serial.SerialException(e)

    def closePort(self):
        self._ser.close()

    def getPacket(self, mode, id=-1, leng=-1, error=-1, params=None):
        """ Read a dynamixel return packet in an iterative attempt.
            Parameters
            ----------
            mode : int
                This should be 0 to start reading packet.
            Returns
            -------
            packet : int
                The error level returned by the device.
        """
        try:
            d = self._ser.read()
        except Exception as e:
            print e
            return None
        # need a positive byte
        if d == '':
            return None

        # now process our byte
        if mode == 0:  # get our first 0xFF
            if ord(d) == 0xff:
                return self.getPacket(1)
            else:
                return self.getPacket(0)
        elif mode == 1:  # get our second 0xFF
            if ord(d) == 0xff:
                return self.getPacket(2)
            else:
                return self.getPacket(0)
        elif mode == 2:  # get id
            if d != 0xff:
                return self.getPacket(3, ord(d))
            else:
                return self.getPacket(0)
        elif mode == 3:  # get length
            return self.getPacket(4, id, ord(d))
        elif mode == 4:  # read error
            self.error = ord(d)
            if leng == 2:
                return self.getPacket(6, id, leng, ord(d), list())
            else:
                return self.getPacket(5, id, leng, ord(d), list())
        elif mode == 5:  # read params
            params.append(ord(d))
            if len(params) + 2 == leng:
                return self.getPacket(6, id, leng, error, params)
            else:
                return self.getPacket(5, id, leng, error, params)
        elif mode == 6:  # read checksum
            checksum = id + leng + error + sum(params) + ord(d)
            if checksum % 256 != 255:
                return None
            return params
        # fail
        return None

    def execute(self, index, ins, params, ret=True):
        """Send an instruction to the device.
            Parameters
            ----------
            index : int
                The ID of the servo to write.
            ins : int
                The instruction to send.
            params : list
                A list of the params to send.
            ret : bool
                Whether to read a return packet.
            Returns
            -------
            packet : int
                The return packet, if read.
            """
        values = None
        self._mutex.acquire()
        try:
            self._ser.flushInput()
        except Exception as e:
            print e
        length = 2 + len(params)
        checksum = 255 - ((index + length + ins + sum(params)) % 256)
        self.__write__(chr(0xFF) + chr(0xFF) + chr(index) + chr(length) + chr(ins))
        for val in params:
            self.__write__(chr(val))
        self.__write__(chr(checksum))
        if ret:
            values = self.getPacket(0)
        self._mutex.release()
        return values

    def read(self, index, start, length):
        """Read values of registers.
            @param index The ID of the servo.
            @param start The starting register address to begin the read at.
            @param length The number of bytes to read.
            @return A list of the bytes read, or -1 if failure.
        """
        values = self.execute(index, AX_READ_DATA, [start, length])
        if values is None:
            return -1
        else:
            return values

    def write(self, index, start, values):
        """Write values to registers.
            @param index The ID of the servo.
            @param start The starting register address to begin writing to.
            @param values The data to write, in a list.
            @return The error level.
            """
        self.execute(index, AX_WRITE_DATA, [start] + values)
        return self.error

    def syncWrite(self, start, values):
        """Write values to registers on many servos.
            @param start The starting register address to begin writing to.
            @param values The data to write, in a list of lists. Format should be
                   [(id1, val1, val2), (id2, val1, val2)]
        """
        output = list()
        for i in values:
            output = output + i
        length = len(output) + 4  # length of overall packet
        lbytes = len(values[0]) - 1  # length of bytes to write to a servo
        self._mutex.acquire()
        try:
            self._ser.flushInput()
        except:
            pass
        self.__write__(chr(0xFF) + chr(0xFF) + chr(254) + chr(length) + chr(AX_SYNC_WRITE))
        self.__write__(chr(start))  # start address
        self.__write__(chr(lbytes))  # bytes to write each servo
        for i in output:
            self.__write__(chr(i))
        checksum = 255 - ((254 + length + AX_SYNC_WRITE + start + lbytes + sum(output)) % 256)
        self.__write__(chr(checksum))
        self._mutex.release()

    def syncRead(self, servos, start, length):
        """Read values of registers on many servos.
            @param servos A list of the servo IDs to read from.
            @param start The starting register address to begin reading at.
            @param length The number of bytes to read from each servo.
            @return A list of bytes read.
        """
        return self.execute(0xFE, AX_SYNC_READ, [start, length] + servos)

    # ===================================== #
    #           Utility Functions           #
    # ===================================== #

    def _val2rad(self, value):
        return (value - AX_ZERO_RAD_POS) * AX_MAX_RAD / (AX_MAX_RAD_POS - AX_ZERO_RAD_POS)

    def _

    # ========================================= #
    #           ROS Utility Functions           #
    # ========================================= #
    def _position_cmd_cb(self, msg):
        pass

    # ========================================== #
    #           Controller set/getters           #
    # ========================================== #
    def setBaudRate(self, servo_id=1, baud=1000000):
        try:
            self.write(servo_id, P_BAUD_RATE, [self._baud_codes[baud]])
        except KeyError:
            rospy.logerr("Please enter a valid baud rate! Setting to 1Mbps.")
            self.write(servo_id, P_BAUD_RATE, [1])

    def setPosition(self, servo_id=1, value=510):
        # Set limits 60 - 820
        if value < 60:
            value = 60
        elif value > 820:
            value = 820

        return self.write(servo_id, P_GOAL_POSITION_L, [value % 256, value >> 8])

    def getModelName(self, servo_id=1):
        try:
            model_num = int(self.read(servo_id, P_MODEL_NUMBER_L, 1)[0])
            return MODEL_NAME[model_num]
        except KeyError:
            rospy.logerr("Model {} does not exist!".format(model_num))
            return -1

    def getTorqueEnable(self, servo_id=1):
        return self.read(servo_id, P_TORQUE_ENABLE, 1)[0]

    def getGoalVelocity(self, servo_id=1):
        """Get the goal velocity of a servo.
        :param servo_id: The ID of the device to read.
        :return: The servo's goal velocity.
        """
        values = self.read(servo_id, P_PRESENT_SPEED_L, 2)
        try:
            return int(values[0]) + (int(values[1]) << 8)
        except:
            return -1

    def getGoalPosition(self, servo_id=1):
        """Get the goal position of a servo.
        :param servo_id: The ID of the device to read.
        :return: The servo's goal position.
        """
        values = self.read(servo_id, P_GOAL_POSITION_L, 2)
        try:
            return int(values[0]) + (int(values[1]) << 8)
        except:
            return -1

    def getPresentVelocity(self, servo_id):
        """Get the current velocity of a servo.
        :param servo_id: The ID of the device to read.
        :return: The servo's current velocity.
        """
        values = self.read(servo_id, P_GOAL_SPEED_L, 2)
        try:
            return int(values[0]) + (int(values[1]) << 8)
        except:
            return -1

    def getPresentPosition(self, servo_id):
        """Get the current position of a servo.
        :param servo_id: The ID of the device to read.
        :return: The servo's current position.
        """
        values = self.read(servo_id, P_PRESENT_POSITION_L, 2)
        try:
            return int(values[0]) + (int(values[1]) << 8)
        except:
            return -1

    def getTemperature(self, servo_id=1):
        """Get the temperature of a device.
        :param servo_id: The ID of the device to read.
        :return: The temperature, in degrees C.
        """
        try:
            return int(self.read(servo_id, P_PRESENT_TEMPERATURE, 1)[0])
        except:
            return -1

    def getServosCount(self, max_id=10):
        id_count = 0
        self._ser.timeout = 0.25
        for i in range(max_id):
            if self.getPosition(i+1) != -1:
                id_count += 1

        self.servo_count = id_count
        rospy.loginfo("Got {} servos".format(self.servo_count))
        return self.servo_count

    def publishDynamixelState(self):
        dyn_state_list = DynamixelStateList()

        for servo_id in range(1, self.servo_count+1):
            dyn_state = DynamixelState()
            dyn_state.model_name = self.getModelName(servo_id)
            dyn_state.id = servo_id
            dyn_state.torque_enable = self.getTorqueEnable(servo_id)
            dyn_state.present_position = self.getPresentPosition(servo_id)
            dyn_state.present_velocity = self.getPresentVelocity(servo_id)
            dyn_state.goal_position = self.getGoalPosition(servo_id)
            dyn_state.goal_velocity = self.getGoalVelocity(servo_id)
            dyn_state_list.append(dyn_state)

        self._dyna_state_pub.publish(dyn_state_list)

    def publishJointState(self):
        for servo_id in range(1, self.servo_count + 1):
            curr_pos = self.getPresentPosition(servo_id)
            curr_vel = self.getPresentVelocity(servo_id)
            self._arm_joint_state.position[servo_id] = self._val2rad(curr_pos)
            self._arm_joint_state.velocity[servo_id] = self._val2vel(curr_vel)


if __name__ == '__main__':
    rospy.init_node("arbotix_test", log_level=rospy.DEBUG)
    arb = ArmControl()
    arb.getServosCount()
    # arb.setPosition()


