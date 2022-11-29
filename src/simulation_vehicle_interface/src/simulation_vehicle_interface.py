#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from btc_msgs.msg import VehicleControl
from json import dumps

class SimulationRemap():

    def __init__(self):

        rospy.init_node('simulation_vehicle_interface', anonymous=True)
        self.pub = rospy.Publisher('/automobile/command', String, queue_size=10)
        rospy.Subscriber("/vehicle/command", VehicleControl, self._callback)

        rospy.spin()


    def _callback(self, msg):

        #generate dictionary from message
        if msg.command_type == VehicleControl.CONTROLLER_COMMAND_SPEED:
            command = {'action': str(msg.command_type), 'speed': msg.f_vel}
        elif msg.command_type == VehicleControl.CONTROLLER_COMMAND_STEER_ANGLE:
            command = {'action': str(msg.command_type), 'steerAngle': msg.f_angle_steer}
        elif msg.command_type == VehicleControl.CONTROLLER_COMMAND_BRAKE:
            command = {'action': str(msg.command_type), 'brake (steerAngle)': msg.f_angle_brake}
        elif msg.command_type == VehicleControl.CONTROLLER_COMMAND_ACTIVATE_PID:
            command = {'action': str(msg.command_type), 'activate': msg.activate}
        elif msg.command_type == VehicleControl.CONTROLLER_COMMAND_ACTIVATE_ENCODER_PUB:
            command = {'action': str(msg.command_type), 'activate': msg.activate}
        elif msg.command_type == VehicleControl.CONTROLLER_COMMAND_SET_PIDS:
            command = {'action': str(msg.command_type), 'kp': msg.kp, 'ki': msg.kp, 'kd': msg.kp, 'tf': msg.tf}
        elif msg.command_type == VehicleControl.CONTROLLER_COMMAND_MOVE:
            command = {'action': str(msg.command_type), 'distance': msg.distance, 'speed': msg.speed}
        
        converted = dumps(command)

        #republish command
        self.pub.publish(converted)


if __name__ == '__main__':
    try:
        sr = SimulationRemap()
    except rospy.ROSInterruptException:
        pass
