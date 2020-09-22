#!/usr/bin/env python
from math import pi
#from math import cos, sin
import os
import diagnostic_msgs
import diagnostic_updater
import roboclaw_driver.roboclaw_driver as roboclaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_msgs.msg import Float64

class Node:
    def __init__(self):
        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}
        self.ref_WR = 0
        self.ref_WL = 0
        self.UiR_1 = 0
        self.errorL_1 = 0
        self.UiL_1 = 0
        self.UiR_1 = 0
        self.Kp = 7.0 #7.0
        self.Ki = 0.5 #2.0
        self.Kd = 3.0 #3.0
        self.errorR_1 = 0
        self.errorR_2 = 0
        self.errorL_2 = 0
        self.errorL_1 = 0
        self.UR_2 = 0
        self.UR_1 = 0
        self.UL_2 = 0
        self.UL_1 = 0
        self.dt = 0.1
        self.EN_ctr = False
        self.last_UL = 0
        self.last_UR = 0
        self.ticks_L = 0.0
        self.ticks_R = 0.0

        rospy.init_node("roboclaw_node")
        rospy.on_shutdown(self.shutdown)
        #rospy.loginfo("Connecting to roboclaw")
        dev_name = rospy.get_param("~dev", "/dev/tty_roboclaw")
        baud_rate = int(rospy.get_param("~baud", "115200"))
        
        self.address = int(rospy.get_param("~address", "128"))
        if self.address > 0x87 or self.address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")
        
        ######################################## TOPICS #############################################
        self.alpha_pub = rospy.Publisher('/alpha_odom', Odometry, queue_size=10)
        # TODO need someway to check if address is correct
        try:
            roboclaw.Open(dev_name, baud_rate)
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")
        
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.
        FunctionDiagnosticTask("Vitals", self.check_vitals))
        
        try:
            version = roboclaw.ReadVersion(self.address)
        except Exception as e:
            rospy.logwarn("Problem getting roboclaw version")
            rospy.logdebug(e)
            pass
        
        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
        else:
            rospy.logdebug(repr(version[1]))
        
        roboclaw.SpeedM1M2(self.address, 0, 0)
        roboclaw.ResetEncoders(self.address)
        
        self.MAX_SPEED = float(rospy.get_param("~max_speed", "2.0"))
        self.TICKS_PER_METER = float(rospy.get_param("~tick_per_meter", "14853.7362"))
        self.BASE_WIDTH = float(rospy.get_param("~base_width", "0.38"))
        
        self.last_set_speed_time = rospy.get_rostime()
        
        ################################ SUBSCRIBER ####################################
        rospy.Subscriber("/joy", Joy, self.joy_event)
#        rospy.sleep(1)
        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("address %d", self.address)
        rospy.logdebug("max_speed %f", self.MAX_SPEED)
        rospy.logdebug("ticks_per_meter %f", self.TICKS_PER_METER)
        rospy.logdebug("base_width %f", self.BASE_WIDTH)

    def joy_event(self,data):
        # Charectization GamePad Logitech F710
        # READ BUTTONS
        A = data.buttons[0]
        B = data.buttons[1]
        X = data.buttons[2]
        Y = data.buttons[3]
        LB = data.buttons[4]
        RB = data.buttons[5]
        BACK = data.buttons[6]
        START = data.buttons[7]
        LOGITECH = data.buttons[8]
        ANALOG_L = data.buttons[9]
        ANALOG_R = data.buttons[10]
        # READ Axes
        LEFT_ANALOG_HOR = data.axes[0] # <<(+)
        LEFT_ANALOG_VER = data.axes[1] # ^^(+)
        LT = data.axes[2] #[1 -1]
        RIGHT_ANALOG_HOR = data.axes[3] # <<(+)
        RIGHT_ANALOG_VER = data.axes[4] # ^^(+)
        RT = data.axes[5] #[1 -1]
        LEFT_RIGHT = data.axes[6] # left=1, right=-1
        FRONT_BACK = data.axes[7] # front=1, back=-1
        if(LB==1):
            G=9.5 # max=9.5
        else:
            G=0.0
        self.ref_WR = G*(RIGHT_ANALOG_VER + LEFT_ANALOG_HOR)
        self.ref_WL = G*(RIGHT_ANALOG_VER - LEFT_ANALOG_HOR)
        
        if LB==1 and A==1: #Laser ON
            os.system("sudo python /home/ubuntu/Desktop/Mercury/laser_on.py")
        elif LB==1 and RB==1: #Leds ON
            os.system("sudo python /home/ubuntu/Desktop/Mercury/leds_on.py")
        elif LB==1 and B==1: #Lights OFF
                os.system("sudo python /home/ubuntu/Desktop/Mercury/lights_off.py")
        elif LEFT_ANALOG_HOR==-1 and RIGHT_ANALOG_VER==-1 and LT==-1 and RT==-1:
                print("Restarting System")
                rospy.sleep(3)
                os.system("sudo reboot")
        elif LOGITECH==1 and B==1:
                print("Shutting down system")
                rospy.sleep(3)
                os.system("sudo poweroff")
        elif LEFT_RIGHT==-1 and START==1 and not self.EN_ctr: #Control Mode
            os.system("sudo python /home/ubuntu/Desktop/Mercury/control_mode.py")
            self.EN_ctr = True
        elif LEFT_RIGHT==1 and START==1 and self.EN_ctr: #Manual Mode
            os.system("sudo python /home/ubuntu/Desktop/Mercury/manual_mode.py")
            self.EN_ctr = False
        self.move(LB,RIGHT_ANALOG_VER,LEFT_ANALOG_HOR)

    def move(self,tumbler,F_B,L_R):
        ML = L_R
        MR = F_B
        if(tumbler==1):
            G=63.0 # max=63
        else:
            G=0.0
        
        U_Trac =int(MR*G)
        U_Dir = int(ML*G)
        Motor2 = U_Trac+U_Dir
        Motor1 = U_Trac-U_Dir
        print(Motor2,Motor1)
        
        if(Motor2>63):
            Motor2=63
        elif(Motor2<-63):
            Motor2=-63
        if(Motor1>63):
            Motor1=63
        elif(Motor1<-63):
            Motor1=-63
        
        self.last_UR = Motor1
        self.last_UL = Motor2

    def encoders(self):
        
        WL_pub = rospy.Publisher('/WL', Float64, queue_size=10)    #
        WR_pub = rospy.Publisher('/WR', Float64, queue_size=10)
        
        Enc1 = roboclaw.ReadEncM1(self.address)
        Enc2 = roboclaw.ReadEncM2(self.address)
        
        self.ticks_L = float(Enc2[1])
        self.ticks_R = float(Enc1[1])
        
#        encoder = "LEFT ENC: "+str(enc_L)+", RIGHT ENC: "+str(enc_R)
#        self.encoder_pub.publish(encoder)	
        
        speed1 = roboclaw.ReadSpeedM1(self.address)
        speed1 = float(speed1[1])
        
        speed2 = roboclaw.ReadSpeedM2(self.address)
        speed2 = float(speed2[1])
        
        Wr = (0.0092*speed1)-0.1514  #RPM_R
        Wl = (0.0092*speed2)+0.0126  #RPM_L
        WR = ((2*pi)/60)*Wr     #rad/s R
        WL = ((2*pi)/60)*Wl     #rad/s L
        
        WL_pub.publish(WL)
        WR_pub.publish(WR)
        
        voltage_pub = rospy.Publisher('/voltage', BatteryState, queue_size=10)
        Battery = BatteryState()
        Battery.header.stamp = rospy.Time.now()
        Battery.header.frame_id = 'PowerBatery'
        
        iL_pub = rospy.Publisher('/iL', Float64, queue_size=10)    #
        iR_pub = rospy.Publisher('/iR', Float64, queue_size=10)
        
        i = roboclaw.ReadCurrents(self.address)[0]
        i_L = roboclaw.ReadCurrents(self.address)[1]
        i_R = roboclaw.ReadCurrents(self.address)[2]
        
        i = 10*float(i)
        i_L = 10*float(i_L)
        i_R = 10*float(i_R)
        
        iL_pub.publish(i_L/1000.0)
        iR_pub.publish(i_R/1000.0)
        
        volts = float(roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10.0)
        volts = volts + 0.5
        
        Battery.voltage = volts
        Battery.current = i/1000.0
        Battery.capacity = 6.0
        Battery.percentage = volts/12.6
        Battery.location = "Power connection - inside of robot"
        if (volts<=11.2):
            Battery.power_supply_health = 3
        else:
            Battery.power_supply_health = 1
        Battery.power_supply_status = 2
        Battery.power_supply_technology = 3
        voltage_pub.publish(Battery)

    def run(self):
        #rospy.loginfo("Starting motor drive")
#        r_time = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 1:
                rospy.loginfo("Did not get command for 1 second, stopping")
            self.encoders()
            if self.EN_ctr:
                try:
#                        UR = self.WR_Control()
#                        UL = self.WL_Control()
                    UR = self.last_UR
                    UL = self.last_UL
#                        pub_pid = "Left ref: "+str(self.ref_WL)+", Speed_L: "+str(self.WL)+", UL: "+str(UL)+",	Right ref: "+str(self.ref_WR)+", Speed_R: "+str(self.WR)+", UR: "+str(UR)
#                        self.pid_pub.publish(pub_pid)
                    roboclaw.ForwardBackwardM1(self.address, int(63+UL)) # Left
                    roboclaw.ForwardBackwardM2(self.address, int(63-UR))
                except OSError as e:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(e)
            else:
                try:
                    roboclaw.ForwardBackwardM1(self.address, 63+self.last_UR) # Left
                    roboclaw.ForwardBackwardM2(self.address, 63-self.last_UL)
                except OSError as e:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(e)
            
            # TODO need find solution to the OSError11 looks like sync problem with serial
            status1, enc1, crc1 = None, None, None
            status2, enc2, crc2 = None, None, None
            
            try:
                status1, enc1, crc1 = roboclaw.ReadEncM1(self.address)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
                rospy.logdebug(e)
            
            try:
                status2, enc2, crc2 = roboclaw.ReadEncM2(self.address)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM2 OSError: %d", e.errno)
                rospy.logdebug(e)

#    def model_implementation(self, twist):
#        self.last_set_speed_time = rospy.get_rostime()
#        
#        linear_x = twist.linear.x
#        if linear_x > self.MAX_SPEED:
#            linear_x = self.MAX_SPEED
#        if linear_x < -self.MAX_SPEED:
#            linear_x = -self.MAX_SPEED
#            
#        vr = linear_x + twist.angular.z * self.BASE_WIDTH / 2.0  # m/s
#        vl = linear_x - twist.angular.z * self.BASE_WIDTH / 2.0
#        
#        vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
#        vl_ticks = int(vl * self.TICKS_PER_METER)
#        
#        rospy.logdebug("vr_ticks:%d vl_ticks: %d", vr_ticks, vl_ticks)
#        
#        try:
#            # This is a hack way to keep a poorly tuned PID from making noise at speed 0
#            if vr_ticks is 0 and vl_ticks is 0:
#                roboclaw.ForwardM1(self.address, 0)
#                roboclaw.ForwardM2(self.address, 0)
#            else:
#                roboclaw.SpeedM1M2(self.address, vr_ticks, vl_ticks)
#        except OSError as e:
#            rospy.logwarn("SpeedM1M2 OSError: %d", e.errno)
#            rospy.logdebug(e)

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = roboclaw.ReadError(self.address)[1]
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return
        
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Main Batt V:", float(roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10))
            stat.add("Logic Batt V:", float(roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10))
            stat.add("Temp1 C:", float(roboclaw.ReadTemp(self.address)[1] / 10))
            stat.add("Temp2 C:", float(roboclaw.ReadTemp2(self.address)[1] / 10))
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            roboclaw.ForwardM1(self.address, 0)
            roboclaw.ForwardM2(self.address, 0)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")