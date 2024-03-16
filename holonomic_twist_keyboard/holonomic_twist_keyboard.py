# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys

import geometry_msgs.msg
import rclpy
import threading

import keyboard



msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages.
---------------------------
Moving around:
        z    
   q    s    d

   rotate left h
   rotate right j

p/m : increase/decrease max speeds by 10%
o/l : increase/decrease only linear speed by 10%
i/k : increase/decrease only angular speed by 10%

esc : stop registering keys
enter : begin registering keys


CTRL-C to quit
"""

moveBindings = {
    'z': (1, 0, 0),
    'q': (0, 1, 0),
    's': (-1, 0, 0),
    'd': (0, -1, 0),
    'h': (0, 0, 1),
    'j': (0, 0, -1),
}

speedBindings = {
    'p': (1.1, 1.1),
    'm': (.9, .9),
    'o': (1.1, 1),
    'l': (.9, 1),
    'i': (1, 1.1),
    'k': (1, .9),
}


def get_key():
    return keyboard.get_hotkey_name()

def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    keyboard.start_recording()

    rclpy.init()

    node = rclpy.create_node('holonomic_twist_keyboard')

    # parameters
    stamped = True
    frame_id = node.declare_parameter('frame_id', '').value
    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print(msg)
        print(vels(speed, turn))
        last_keys = get_key()
        register = False
        while True:
            x = 0
            y = 0
            th = 0
            keys = get_key()
            if register == False:
                if 'enter' in keys and 'ctrl' in keys and 'a' in keys:
                    register = True
                    print("ok")
            elif register == True:
                if 'esc' in keys:
                    register = False
                    print("bye")
            if (not keys == None) and register == True:
                for key in keys:
                    if key in moveBindings.keys():
                        x += moveBindings[key][0]
                        y += moveBindings[key][1]
                        th += moveBindings[key][2]
                    elif key in speedBindings.keys() and key not in last_keys:
                        speed = speed * speedBindings[key][0]
                        turn = turn * speedBindings[key][1]
                        print(vels(speed, turn))
                        if (status == 14):
                            print(msg)
                        status = (status + 1) % 15
                    else:
                        if (key == '\x03'):
                            break
            last_keys = keys

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist_msg)

    except Exception as e:
        print(e)

    finally:
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()


if __name__ == '__main__':
    main()
