#!/usr/bin/env python
# -*- Python -*-

"""
 \file ArmISIK.py
 \brief This module controls the robot arm for education:ArmIS type0
 \date $Date$


"""
import sys
import time
sys.path.append(".")

import calcik
import ConfigParser as Conf
import time

# Import RTM module
import RTC
import OpenRTM_aist

# Import Service implementation class
# <rtc-template block="service_impl">

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
armisik_spec = ["implementation_id", "ArmISIK", 
		 "type_name",         "ArmISIK", 
		 "description",       "This module controls the robot arm for education:ArmIS type0", 
		 "version",           "1.0.0", 
		 "vendor",            "Matsuda Hiroaki", 
		 "category",          "ARM", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "0", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 ""]
# </rtc-template>

class ArmISIK(OpenRTM_aist.DataFlowComponentBase):
	
	"""
	\class ArmISIK
	\brief This module controls the robot arm for education:ArmIS type0
	
	"""
	def __init__(self, manager):
		"""
		\brief constructor
		\param manager Maneger Object
		"""
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		self._d_pos = RTC.TimedLongSeq(RTC.Time(0,0),[])
		"""
		"""
		self._posIn = OpenRTM_aist.InPort("pos", self._d_pos)
		self._d_sens = RTC.TimedLongSeq(RTC.Time(0,0),[])
		"""
		"""
		self._sensIn = OpenRTM_aist.InPort("sens", self._d_sens)
		self._d_motion = RTC.TimedLongSeq(RTC.Time(0,0),[])
		"""
		"""
		self._motionOut = OpenRTM_aist.OutPort("motion", self._d_motion)
		self._d_on_off = RTC.TimedLongSeq(RTC.Time(0,0),[])
		"""
		"""
		self._on_offOut = OpenRTM_aist.OutPort("on_off", self._d_on_off)
		


		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
		
		# </rtc-template>


		 
	def onInitialize(self):
		"""
		
		The initialize action (on CREATED->ALIVE transition)
		formaer rtc_init_entry() 
		
		\return RTC::ReturnCode_t
		
		"""
		# Bind variables and configuration variable
		
		# Set InPort buffers
		self.addInPort("pos",self._posIn)
		self.addInPort("sens",self._sensIn)
		
		# Set OutPort buffers
		self.addOutPort("motion",self._motionOut)
		self.addOutPort("on_off",self._on_offOut)
		
		# Set service provider to Ports
		
		# Set service consumers to Ports
		
		# Set CORBA Service Ports

		self.ik = calcik.CalcIk()
		print('onInitialize')
		
		return RTC.RTC_OK
	
	def onActivated(self, ec_id):
		"""
	
		The activated action (Active state entry action)
		former rtc_active_entry()
	
		\param ec_id target ExecutionContext Id
	
		\return RTC::ReturnCode_t
	
		"""
		self.conf = Conf.SafeConfigParser()
                self.conf.read('ini/ikconfig.ini')
                self.move_time = int(self.conf.get('SERVO', 'move_time'))

                print('onActivated')
	
		return RTC.RTC_OK
	
	def onDeactivated(self, ec_id):
		"""
	
		The deactivated action (Active state exit action)
		former rtc_active_exit()
	
		\param ec_id target ExecutionContext Id
	
		\return RTC::ReturnCode_t
	
		"""

		print('onDeactivated')
	
		return RTC.RTC_OK
	
	def onExecute(self, ec_id):
		"""
	
		The execution action that is invoked periodically
		former rtc_active_do()
	
		\param ec_id target ExecutionContext Id
	
		\return RTC::ReturnCode_t
	
		"""
		if self._posIn.isNew():
			# _d_pos.data:[command, pos_x, pos_y, pos_z, grab]
                        self._d_pos = self._posIn.read()
                        
                        command = self._d_pos.data[0]

                        if command == 0:
                                self.move_initial(-200)

                                print('Initial Position')

                        elif command == 1:
                                self.move_initial(0)
                                
                                pos_x = self._d_pos.data[1]
                                pos_y = self._d_pos.data[2]
                                pos_z = self._d_pos.data[3]
                                grab = self._d_pos.data[4]

                                theta = self.ik.calc_ik(pos_x, pos_y, pos_z)

                                self.write_motion([0, 1, theta[0], self.move_time])
                                self.write_motion([0, 3, theta[2], self.move_time])
                                self.write_motion([0, 2, theta[1], self.move_time])

                                print('Move to Color Ball')

                        elif command == 2:
                                self.write_motion([0, 4, -700 + int(self.ik.calb_4), self.move_time])
                                time.sleep(1)

                                self.move_initial(-700, [2, 1, 3])

                                time.sleep(1)
                                self.write_motion([0, 4, -100 + int(self.ik.calb_4), self.move_time])

                                print('Housed in a tray')

                        elif command == 1000:
                                for i in range(4):
                                        data = [1, i + 1, 1]
                                        self._d_on_off.data = data
                                        OpenRTM_aist.setTimestamp(self._d_on_off)
                                        self._on_offOut.write()

                                print('Toruque ON')

                        elif command == 1001:
                                for i in range(4):
                                        data = [1, i + 1, 0]
                                        self._d_on_off.data = data
                                        OpenRTM_aist.setTimestamp(self._d_on_off)
                                        self._on_offOut.write()

                                print('Toruque OFF')
	
		return RTC.RTC_OK

	def move_initial(self, grip, order = [3, 2, 1]):
                data = []
                pos = self.ik.calc_ik(140, 0, 50)
                data.append([0, order[0], pos[order[0] - 1], self.move_time])
                data.append([0, order[1], pos[order[1] - 1], self.move_time])
                data.append([0, order[2], pos[order[2] - 1], self.move_time])
                data.append([0, 4, grip + int(self.ik.calb_4), self.move_time])
                for temp in data:
                        self._d_motion.data = temp
                        OpenRTM_aist.setTimestamp(self._d_motion)
                        self._motionOut.write()
                        time.sleep(self.move_time / 100.0)

        def write_motion(self, data):
                self._d_motion.data = data
                OpenRTM_aist.setTimestamp(self._d_motion)
                self._motionOut.write()
                time.sleep(self.move_time / 100.0)

def ArmISIKInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=armisik_spec)
    manager.registerFactory(profile,
                            ArmISIK,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    ArmISIKInit(manager)

    # Create a component
    comp = manager.createComponent("ArmISIK")

def main():
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager()

if __name__ == "__main__":
	main()

