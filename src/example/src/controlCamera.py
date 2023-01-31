#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import json
from pynput import keyboard
import cv2
import os
from datetime import datetime

from RcBrainThread import RcBrainThread
from std_msgs.msg import String
from sensor_msgs.msg import Image
from time import sleep

import rospy

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class RemoteControlTransmitterProcess():
    # ===================================== INIT==========================================
    def __init__(self):
        """Run on the PC. It forwards the commans from the user via KeboardListenerThread to the RcBrainThread. 
        The RcBrainThread converts them into actual commands and sends them to the remote via a socket connection.
        
        """
        self.dirKeys   = ['w', 'a', 's', 'd']
        self.paramKeys = ['t','g','y','h','u','j','i','k', 'r', 'p']
        self.pidKeys = ['z','x','v','b','n','m']
        self.direrction = 'straight'

        self.date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        os.makedirs('data/'+self.date_str)
        self.data_dir = 'data/'+self.date_str+'/'
        self.index = 0

        self.allKeys = self.dirKeys + self.paramKeys + self.pidKeys
        
        self.rcBrain   =  RcBrainThread()   
        
        rospy.init_node('EXAMPLEnode', anonymous=False)     
        self.publisher = rospy.Publisher('/automobile/command', String, queue_size=1)

        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.callback)


        rospy.spin()
        

    def callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazsbo format
        :return: nothing but sets [cv_image] to the usefull image that can be use in opencv (numpy array)
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

         # save the image to the data folder
        cv2.imwrite(self.data_dir + str(self.index) + '.png', self.cv_image)
        self.index += 1

        # this will be a frame from the camera of the car - we can use it to detect the line
        # we then calculate whether we should go left or right and send the command to the car

        # for line detection we can use the following code:
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
        close = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel, iterations=1)
        cnts = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        for c in cnts:
            area = cv2.contourArea(c)
            if area > 500:
                cv2.drawContours(self.cv_image, [c], -1, (36, 255, 12), 2)
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (36, 255, 12), 2)
                x1 = x + w/2
                y1 = y + h/2
                
                cv2.circle(self.cv_image, (int(x1), int(y1)), 5, (36, 255, 12), -1)
                cv2.putText(self.cv_image, str(x1), (int(x1), int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (36, 255, 12), 2)

        # lets get two most centre-lines
        # we will use them to calculate the angle of the car
        # if the angle is too big we will send a command to the car to turn left or right
        left_line = None
        right_line = None
        angle = 0
        for c in cnts:
            area = cv2.contourArea(c)
            if area > 500:
                x, y, w, h = cv2.boundingRect(c)
                x1 = x + w/2
                y1 = y + h/2
                if left_line is None:
                    left_line = [x1, y1]
                elif right_line is None:
                    right_line = [x1, y1]
                elif x1 < left_line[0]:
                    left_line = [x1, y1]
                elif x1 > right_line[0]:
                    right_line = [x1, y1]
        angle = 0
        if left_line is not None and right_line is not None:
            angle = (left_line[0] - right_line[0]) / 2
            angle = angle / 100
            

        # show the image
        cv2.imshow("Image window", self.cv_image)
        cv2.waitKey(1)
        if not angle:
            angle = 0

        # always 1 decimal place even if it is 0
        str_angle = str(round(angle, 1))
        
        print('sending')
        print('{"action":"3", "steerAngle": ' + str_angle + '}')
        # self.publisher.publish('{"action":"3", "steerAngle: "' + str_angle + '}')
        # if abs(angle) > 0:
        self.rcBrain.steerAngle = angle*15
        self._send_command('p.w')
        print(angle)
            


        # self._send_command('p.a')
        # self._send_command('p.d')


    
   
    # ===================================== SEND COMMAND =================================
    def _send_command(self, key):
        """Transmite the command to the remotecontrol receiver. 
        
        Parameters
        ----------
        inP : Pipe
            Input pipe. 
        """
        command = self.rcBrain.getMessage(key)
        if command is not None:
	
            command = json.dumps(command)
            self.publisher.publish(command)  
            
if __name__ == '__main__':
    try:
        nod = RemoteControlTransmitterProcess()
    except rospy.ROSInterruptException:
        pass