#!/usr/bin/env python

from __future__ import print_function

import argparse
import rospy
import serial

from kb_utils.msg import Command
from kb_utils.msg import Encoder
from kb_utils.msg import Sonar
from kb_utils.msg import Servo_Command
from kb_utils.srv import ResetEncoder

from std_msgs.msg import Bool

class KB_Driver(object):

    def __init__(self, port):
        """
        Interfaces to the Teensy on the KB car
        """
        self.ser = serial.Serial(port, 115200)
        self.enc_pub = rospy.Publisher("encoder", Encoder, queue_size=1)
        self.son_pub = rospy.Publisher("sonar", Sonar, queue_size=1)
        self.servo_pub = rospy.Publisher("safety_pilot", Servo_Command, queue_size=1)
        self.override_pub = rospy.Publisher("safety_pilot_override", Bool, queue_size=1)
        self.cmd_sub = rospy.Subscriber("command", Command, self.send_command)
        self.enc_srv = rospy.Service('reset_encoder', ResetEncoder, self.reset_encoder)

    def reset_encoder(self, req):
        self.ser.write("<r>")
        return 0

    def send_command(self, cmd_msg):
        self.ser.write("<c:{},{}>".format(cmd_msg.steer, cmd_msg.throttle))
        # print("<c:{},{}>".format(cmd_msg.steer, cmd_msg.throttle))

    def pub_data(self, data):
        now = rospy.Time().now()
        enc_msg = Encoder()
        enc_msg.header.stamp = now
        enc_msg.header.frame_id = 'global'
        enc_msg.dist = float(data[0])
        enc_msg.vel = float(data[1])
        self.enc_pub.publish(enc_msg)

        son_msg = Sonar()

        son_msg.header.stamp = now
        son_msg.header.frame_id = 'global'
        son_msg.dist_back = float(data[2])
        self.son_pub.publish(son_msg)

        servo_msg = Servo_Command()
        servo_msg.header.stamp = now
        servo_msg.header.frame_id = 'global'
        servo_msg.steer = int(data[3])
        servo_msg.throttle = int(data[4])
        self.servo_pub.publish(servo_msg)

        self.override_pub.publish(data=(data[5] != '0'))

    def clean_shutdown(self):
        print("\nExiting kb_driver...")
        return True

    def run(self):
        while not rospy.is_shutdown():
            line = self.ser.readline()
            # print(line)
            try:
                a = line.index('[')+1
                b = line.index(']')
                data = line[a:b].split(',')
                if len(data) != 7:
                    raise ValueError
                self.pub_data(data[:-1])
            except ValueError:
                rospy.logwarn("Received malformed packet.")
                pass


def main():
    """driver to interface to the Teensy on the KB_Car

    Command Line Arguments
    port -- the serial port to talk to the Teensy
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                    description=main.__doc__)
    parser.add_argument('port', type=str, default='')
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("kb_driver", log_level=rospy.DEBUG)

    kb_driver = KB_Driver(args.port)
    rospy.on_shutdown(kb_driver.clean_shutdown)
    kb_driver.run()

    print("Done.")

if __name__ == '__main__':
    main()
