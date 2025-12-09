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

import socket
import threading
import ssl
import geometry_msgs.msg
import rcl_interfaces.msg
import rclpy
import asyncio
from oslo_log import log as logging
from iotronic_lightningrod.modules.plugins import Plugin
from std_msgs.msg import String
from autobahn.asyncio.component import Component

LOG = logging.getLogger(__name__)
board_name = socket.gethostname()
crossbar_ip="192.168.44.135"

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

def vels(speed, turn):
    return 'currently:\tspeed %.2f\tturn %.2f ' % (speed, turn)

class Worker(Plugin.Plugin):
    def __init__(self, uuid, name, q_result=None, params=None):
        super(Worker, self).__init__(uuid, name, q_result, params)

    def Set_cmd_vel_Node(self):
        rclpy.init()

        node = rclpy.create_node('teleop_twist_keyboard')
        read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
        stamped = node.declare_parameter('stamped', False, read_only_descriptor).value
        frame_id = node.declare_parameter('frame_id', '', read_only_descriptor).value
        speed = node.declare_parameter('speed', 0.5, read_only_descriptor).value
        turn = node.declare_parameter('turn', 1.0, read_only_descriptor).value


        if not stamped and frame_id:
            raise Exception("'frame_id' can only be set when 'stamped' is True")

        if stamped:
            TwistMsg = geometry_msgs.msg.TwistStamped
        else:
            TwistMsg = geometry_msgs.msg.Twist

        pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)
        spinner = threading.Thread(target=rclpy.spin, args=(node,))


        self.node = node
        self.pub = pub
        self.TwistMsg = TwistMsg
        self.twist_msg = TwistMsg()
        self.speed = speed
        self.turn = turn
        self.spinner= spinner
        self.stamped = stamped
        self.frame_id = frame_id

    def run(self):
        self.Set_cmd_vel_Node()
        twist_msg = self.twist_msg
        if self.stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = self.node.get_clock().now().to_msg()
            twist_msg.header.frame_id = self.frame_id
        else:
            twist = twist_msg
        
        def start_wamp():
            ssl_ctx = ssl._create_unverified_context()

            component = Component(
                transports=[
                    {
                            "type": "websocket",
                            "url": "wss://"+crossbar_ip+":8181/ws",
                            "endpoint": {
                                "type": "tcp",
                                "host": crossbar_ip,
                                "port": 8181,
                                "tls": ssl_ctx
                            },
                            "serializers": ["json", "msgpack"]
                        }
                    ],
                    realm="s4t"
                )
            @component.on_join
            async def onJoin(session, details):
                LOG.info(f"[WAMP] Session joined as {board_name}")
                LOG.info("[WAMP] RPCs registered: teleop_drive")
                session.drive_enabled=True

                async def teleop_drive(*args, **kwargs):
                    LOG.info(f"[WAMP] teleop_drive started")

                    if not session.drive_enabled:
                        return {"result": "Teleop not enabled"}

                    try:
                        LOG.info(vels(self.speed, self.turn))
                        x = 0.0
                        y = 0.0
                        z = 0.0
                        th = 0.0

                        key = args[0]['key']

                        if key in moveBindings:
                            x, y, z, th = moveBindings[key]
                            LOG.info(f"[WAMP] Moving: x={x}, y={y}, z={z}, th={th}")

                        elif key in speedBindings:
                            self.speed *= speedBindings[key][0]
                            self.turn *= speedBindings[key][1]
                            LOG.info(vels(self.speed, self.turn))
                            return {"result": vels(self.speed, self.turn)}

                        else:
                            x = y = z = th = 0.0
                            if key == 'v':
                                session.drive_enabled = False
                                LOG.info("[WAMP] teleop_drive stopped")
                                return {"result": "Teleop disabled"}
                        if self.stamped:
                            twist_msg.header.stamp = self.node.get_clock().now().to_msg()

                        twist.linear.x = x * self.speed
                        twist.linear.y = y * self.speed
                        twist.linear.z = z * self.speed
                        twist.angular.x = 0.0
                        twist.angular.y = 0.0
                        twist.angular.z = th * self.turn

                        LOG.info(twist_msg)
                        self.pub.publish(twist_msg)

                        return {"result": f"cmd_vel sent: x={x}, y={y}, z={z}, th={th}, speed={self.speed:.2f}, turn={self.turn:.2f}"}

                    except Exception as e:
                        LOG.error(e)
                        return {"result": f"Error: {e}"}
                    
                LINEAR_MAX = 0.7 
                ANGULAR_MAX = 0.6

                async def teleop_analog(*args, **kwargs):
                    try:
                        if not session.drive_enabled:
                            return {"result": "Teleop not enabled"}

                        data = args[0] if args else kwargs

                        lx = float(data.get("lx", 0.0))  
                        rx = float(data.get("rx", 0.0))
                        lx = max(-1.0, min(1.0, lx))
                        rx = max(-1.0, min(1.0, rx))

                        DEADZONE = 0.05
                        if abs(lx) < DEADZONE:
                            lx = 0.0
                        if abs(rx) < DEADZONE:
                            rx = 0.0

                        v_max = LINEAR_MAX
                        w_max = ANGULAR_MAX

                        v = lx * v_max
                        w = rx * w_max

                        twist_msg = self.twist_msg
                        if self.stamped:
                            twist_msg.header.stamp = self.node.get_clock().now().to_msg()
                            twist = twist_msg.twist
                        else:
                            twist = twist_msg

                        twist.linear.x = v
                        twist.linear.y = 0.0
                        twist.linear.z = 0.0
                        twist.angular.x = 0.0
                        twist.angular.y = 0.0
                        twist.angular.z = w

                        self.pub.publish(twist_msg)

                        LOG.info(f"[WAMP] teleop_analog v={v:.2f}, w={w:.2f}")
                        return {
                            "result": "ok",
                            "v": v,
                            "w": w
                        }

                    except Exception as e:
                        LOG.error(f"[WAMP] teleop_analog error: {e}")
                        return {"result": f"error: {e}"}
                    
                await session.register(teleop_drive, f"iotronic.{board_name}.teleop_drive")
                await session.register(teleop_analog, f"iotronic.{board_name}.teleop_analog")

            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                component.start(loop=loop)
                self.spinner.start()
                loop.run_forever()
            except Exception as e:
                LOG.error(f"[WAMP] Error in WAMP loop: {e}")

        threading.Thread(target=start_wamp, name="WAMP_ROSbotXL_teleop", daemon=True).start()
        LOG.info("[WAMP] ROSbotXL set, waiting for RPC...")
        self.q_result.put("WAMP_ROSbotXL_teleop plugin correctly started")
