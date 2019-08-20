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
        rclpy.init()

        self._node = rclpy.create_node('robot_control_publisher')
        self._pub = self._node.create_publisher(String, 'topic', 10)

    def initialize(self):
        thank_you_intent = IntentBuilder("ThankYouIntent"). \
            require("ThankYouKeyword").build()
        self.register_intent(thank_you_intent, self.handle_thank_you_intent)

        how_are_you_intent = IntentBuilder("HowAreYouIntent"). \
            require("HowAreYouKeyword").build()
        self.register_intent(how_are_you_intent,
                             self.handle_how_are_you_intent)

        hello_world_intent = IntentBuilder("HelloWorldIntent"). \
            require("HelloWorldKeyword").build()
        self.register_intent(hello_world_intent,
                             self.handle_hello_world_intent)

    def handle_thank_you_intent(self, message):
        self.speak_dialog("welcome")
        self._pub.publish(String("welcome"))

    def handle_how_are_you_intent(self, message):
        self.speak_dialog("how.are.you")
        #msg = String()
        #msg.data = "howdy"
        self._pub.publish(String("howdy"))

    def handle_hello_world_intent(self, message):
        self.speak_dialog("hello.world")
        self._pub.publish(String("hello world"))

    def stop(self):
        self._node.destroy_node()
        rclpy.shutdown()


def create_skill():
    return RobotControlSkill()
