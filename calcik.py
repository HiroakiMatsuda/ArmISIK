# This module provides a class that controls the serial servo motor manufactured by Futaba Corp.
# ver1.20831
# This module has been tested on python ver.2.6.6
# It need pySerial(http://pyserial.sourceforge.net/)
# (C) 2012 Matsuda Hiroaki

import ConfigParser as Conf
import math

class CalcIk(object):

        def __init__(self):
                self.conf = Conf.SafeConfigParser()
                self.conf.read('ini/ikconfig.ini')
                self.link_1 = float(self.conf.get('BODY', 'link_1'))
                self.link_2 = float(self.conf.get('BODY', 'link_2'))
                self.link_3 = float(self.conf.get('BODY', 'link_3'))
                self.link_4 = float(self.conf.get('BODY', 'link_4'))
                self.link_5 = float(self.conf.get('BODY', 'link_5'))
                # Distance to the center of the gripper from the third joint
                self.link_6 = self.link_4 + self.link_5

                self.calb_1 = float(self.conf.get('CALIBRATION', 'calb_1'))
                self.calb_2 = float(self.conf.get('CALIBRATION', 'calb_2'))
                self.calb_3 = float(self.conf.get('CALIBRATION', 'calb_3'))
                self.calb_4 = float(self.conf.get('CALIBRATION', 'calb_4'))

                self.servo1_min = int(self.conf.get('LIMIT', 'servo1_min'))
                self.servo1_max = int(self.conf.get('LIMIT', 'servo1_max'))
                self.servo2_min = int(self.conf.get('LIMIT', 'servo2_min'))
                self.servo2_max = int(self.conf.get('LIMIT', 'servo2_max'))
                self.servo3_min = int(self.conf.get('LIMIT', 'servo3_min'))
                self.servo3_max = int(self.conf.get('LIMIT', 'servo3_max'))
                self.servo4_min = int(self.conf.get('LIMIT', 'servo4_min'))
                self.servo4_max = int(self.conf.get('LIMIT', 'servo4_max'))

        def calc_ik(self, pos_x, pos_y, pos_z):
                theta_1 = math.atan2(pos_x, pos_y) * 180 / math.pi
                
                #Converted into the coordinate system of the first joint
                fj_pos_z = pos_z - (self.link_1 + self.link_2)

                
                length = math.sqrt(pos_x ** 2 + pos_y ** 2 + fj_pos_z ** 2)

                try:

                        th_2 = math.acos((length ** 2 + self.link_3 ** 2 - self.link_6 ** 2)
                                                / (2 * length * self.link_3))
                        th_3 = math.acos((self.link_3 ** 2 + self.link_6 ** 2 - length ** 2)
                                                / (2 * self.link_3 * self.link_6))

                        if fj_pos_z > 0 or fj_pos_z < 0:
                                theta_2 = math.pi / 2 - (math.atan(fj_pos_z / length) + th_2)

                        elif fj_pos_z == 0:
                                theta_2 = math.pi / 2 - th_2

                        theta_3 = math.pi - th_3

                except:
                        return [int(self.calb_1), int(self.calb_2), int(self.calb_3), int(self.calb_4)]

                theta_2 = theta_2 * 180 / math.pi                       
                theta_3 = theta_3 * 180 / math.pi

                theta_1 = int(theta_1 * 10 + self.calb_1) 
                theta_2 = int(theta_2 * 10 + self.calb_2)
                theta_3 = int(theta_3 * 10 + self.calb_3)

                if theta_1 < self.servo1_min:
                        theta_1 = self.servo1_min
                        print('Theta_1: Limit min')
                elif theta_1 > self.servo1_max:
                        theta_1 = self.servo1_max
                        print('Theta_1: Limit max')
                if theta_2 < self.servo2_min:
                        theta_2 = self.servo2_min
                        print('Theta_2: Limit min')
                elif theta_2 > self.servo2_max:
                        theta_2 = self.servo2_max
                        print('Theta_2: Limit max')
                if theta_3 < self.servo3_min:
                        theta_3 = self.servo3_min
                        print('Theta_3: Limit min')
                elif theta_3 > self.servo3_max:
                        theta_3 = self.servo3_max
                        print('Theta_3: Limit max')
                        
                return theta_1, theta_2, theta_3
                
