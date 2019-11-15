# Copyright 2016 Mycroft AI, Inc.
#
# This file is part of Mycroft Core.
#
# Mycroft Core is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Mycroft Core is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Mycroft Core.  If not, see <http://www.gnu.org/licenses/>.

import json

from adapt.intent import IntentBuilder

from mycroft.skills.core import MycroftSkill
from mycroft.util.log import getLogger

import rclpy
from std_msgs.msg import String

__author__ = 'machinekoder'

LOGGER = getLogger(__name__)


class RobotControlSkill(MycroftSkill):
    def __init__(self):
        super(RobotControlSkill, self).__init__(name="RobotControlSkill")
        try:
            rclpy.init()
        except:
            pass

        self._node = rclpy.create_node('robot_control_publisher')
        self._pub = self._node.create_publisher(String, 'mycroft/request', 10)
        self._sub = self._node.create_subscription(
            String, 'mycroft/speak', self.handle_speak_request, 10
        )
        self._order_drink_pub = self._node.create_publisher(
            String, 'move_bottle/order_drink', 1
        )
        self._scan_pub = self._node.create_publisher(
            String, 'move_bottle/scan', 1
        )

    def spin_ros(self, _message):
        rclpy.spin_once(self._node, timeout_sec=0.0)

    def initialize(self):
        how_are_you_intent = (
            IntentBuilder("HowAreYouIntent").require("HowAreYouKeyword").build()
        )
        self.register_intent(how_are_you_intent, self.handle_how_are_you_intent)

        hello_world_intent = (
            IntentBuilder("HelloWorldIntent")
            .require("HelloWorldKeyword")
            .build()
        )
        self.register_intent(hello_world_intent, self.handle_hello_world_intent)

        look_at_me_intent = (
            IntentBuilder("LookAtMeIntent").require("LookAtMeKeyword").build()
        )
        self.register_intent(look_at_me_intent, self.handle_look_at_me_intent)

        scan_bottles_intent = (
            IntentBuilder("ScanBottlesIntent")
            .require("ScanBottlesKeyword")
            .build()
        )
        self.register_intent(
            scan_bottles_intent, self.handle_scan_bottles_intent
        )

        scan_tablet_intent = (
            IntentBuilder("ScanTabletIntent")
            .require("ScanTabletKeyword")
            .build()
        )
        self.register_intent(scan_tablet_intent, self.handle_scan_tablet_intent)

        self.register_intent_file('order.drink.intent', self.handle_order_drink_intent)

        # schedule a periodical event to spin the ROS node
        self.schedule_repeating_event(self.spin_ros, None, 0.1, name='SpinRos')

    def handle_how_are_you_intent(self, message):
        self.speak_dialog("how.are.you")
        msg = String()
        msg.data = "howdy"
        self._pub.publish(msg)

    def handle_hello_world_intent(self, message):
        self.speak_dialog("hello.world")
        msg = String()
        msg.data = "hello world"
        self._pub.publish(msg)

    def handle_look_at_me_intent(self, message):
        self.speak_dialog('look.at.me')
        msg = String()
        msg.data = "look at me"
        self._pub.publish(msg)

    def handle_scan_bottles_intent(self, _message):
        self.speak_dialog('executing.request')
        msg = String()
        msg.data = json.dumps({'type': 'bottles'})
        self._scan_pub.publish(msg)

    def handle_scan_tablet_intent(self, _message):
        self.speak_dialog('executing.request')
        msg = String()
        msg.data = json.dumps({'type': 'tablet'})
        self._scan_pub.publish(msg)

    def handle_order_drink_intent(self, message):
        self.speak_dialog('executing.request')
        msg = String()
        msg.data = json.dumps({'name': message.data.get('drink')})
        self._order_drink_pub.publish(msg)

    def handle_speak_request(self, msg):
        self.speak(msg.data, False)

    def stop(self):
        self._node.destroy_node()
        rclpy.shutdown()


def create_skill():
    return RobotControlSkill()
