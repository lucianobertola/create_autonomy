#! /usr/bin/env python

import math
import rospy
import threading
import PyKDL

import numpy as np

from geometry_msgs.msg import Twist, Pose, Pose2D, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties
from sensor_msgs.msg import Imu, JointState


CMD_VEL_TOPIC = "/cmd_vel"
STASIS_VEL_TOPIC = "/stasis_vel"
GT_TOPIC = "/odom"
JOINT_STATES_TOPIC = "/joint_states"
IMU_TOPIC = "/imu/data"

PENDIENTE_LIN = 0.15   # piso de ruido en la deteccion la pendiente positiva en la aceleracion
PENDIENTE_ANG = 0.15
VEL_BASE = 0.05

TIEMPO_INERCIA=1

TAM = 10
class nodo_stasis(object):
    """docstring for nodo_stasis."""
    def __init__(self):
        self.pose = Pose2D()
        self.last_pose = None
        self.pose_lock = threading.Lock()   # esto es una clase
        
        self.ruedas_lock = threading.Lock()   # esto es una clase        
        self.pos_rueda_iz = 0
        self.pos_rueda_der = 0
        
        self.vel_Twist = Twist()
        self.vel_lock = threading.Lock()   # esto es una clase
        
        self.ace_Imu = Imu()
        self.imu_lock = threading.Lock()   # esto es una clase
        self.ace_Imu_ant=None
        self.ace_Imu=None
        
        self.esta_parado=0
        self.quiere_arrancar=0        
        self.esta_arrancando=0
        self.stasis=0        
        
        self.vec_ace_lin = np.zeros(TAM)
        self.vec_ace_ang = np.zeros(TAM)
        self.i = 0
                
        rospy.init_node("nodo_stasis")
        self.vel_pub = rospy.Publisher(STASIS_VEL_TOPIC, Twist, queue_size=1)
        self.vel_sub = rospy.Subscriber(CMD_VEL_TOPIC, Twist, self._cmd_cb)  #no se porque ese nombre
        #self.gt_sub = rospy.Subscriber(GT_TOPIC, Odometry, self._gt_cb)
        self.gt_sub = rospy.Subscriber(JOINT_STATES_TOPIC, JointState, self._js_cb)
        self.imu_sub = rospy.Subscriber(IMU_TOPIC, Imu, self.__imu_cb__)
        
        # This sleep avoids issues with the first case
        rospy.sleep(rospy.Duration(secs=1))



    def __imu_cb__(self, msg):
        
        self.imu_lock.acquire()          
        self.ace_Imu = msg    

        self.vec_ace_lin=np.roll(self.vec_ace_lin,1)
        self.vec_ace_lin[0]=abs(round(msg.linear_acceleration.x,2))
        self.vec_ace_ang=np.roll(self.vec_ace_ang,1)
        self.vec_ace_ang[0]=abs(round(msg.angular_velocity.z,2))   
        
        if self.i > 0:
            self.i -= 1
     
        self.imu_lock.release()
        


    def _cmd_cb(self, msg):     # en este nos llegan las comandos de velocidad entra cada vez que alguien publica vel 
        
        self.vel_lock.acquire()
        self.vel_Twist = msg
        self.vel_lock.release()



    def _gt_cb(self, msg):     # en este nos llegan las posiciones de odometria

        self.pose_lock.acquire()
        self.vel_odom = msg
        self.pose_lock.release()

    def _js_cb(self, msg):     # en este nos llegan posiciones de las ruedas
       
        self.ruedas_lock.acquire()
        self.pos_ruedas_msg = msg
        
        self.pos_rueda_ant_iz = self.pos_rueda_iz
        self.pos_rueda_ant_der = self.pos_rueda_der  
              
        self.pos_rueda_iz = abs(self.pos_ruedas_msg.position[1])           
        self.pos_rueda_der = abs(self.pos_ruedas_msg.position[0])
        
        self.delta_pos_iz=round(abs(self.pos_rueda_iz-self.pos_rueda_ant_iz),3)    
        self.delta_pos_der=round(abs(self.pos_rueda_der-self.pos_rueda_ant_der),3)    
        
        self.delta_pos = self.delta_pos_iz + self.delta_pos_der
                        
 #       print "msg pos ruedas:", self.delta_pos, rospy.Time.now().secs               
        self.ruedas_lock.release()

        
           


    def miro_stasis(self):
       
        
        # comandos de velocidad              
        twist_vel_lin = self.vel_Twist.linear.x  #solo mueve sobre eje x del robot
        twist_vel_ang = self.vel_Twist.angular.z  # solo gira sobre eje z        
               
        # datos de la imu
        prom_act_lin=np.mean(abs(self.vec_ace_lin[0:4]))
        prom_ant_lin=np.mean(abs(self.vec_ace_lin[5:9]))
        prom_act_ang=np.mean(abs(self.vec_ace_ang[0:4]))
        prom_ant_ang=np.mean(abs(self.vec_ace_ang[5:9]))
    
        
        if (twist_vel_lin == 0 and twist_vel_ang == 0): 
            #print "esta parado"
            self.esta_parado=1
            self.quiere_arrancar=0
            self.esta_arrancando=0
            self.stasis=0
            
        if (self.esta_parado == 1 and (twist_vel_lin != 0 or twist_vel_ang != 0)):
            print "stasis: quiere arrancar"
            self.quiere_arrancar=1
            self.esta_arrancando=0
            self.esta_parado=0
            self.stasis=0
                 
            self.vec_ace_lin=np.zeros(TAM)
            self.vec_ace_ang=np.zeros(TAM)            
            #ahora que esta por arrancar toma TAM/2 valores nuevos para realizarle la media y compararlo con la media de los TAM/2 valores previos al arrancuque            
            self.i=TAM/2
            while (self.i > 0): # se duerme hasta que se llenen los TAM/2 valores necesarios
                #se duerme un ratito para darle tiempo de respuesta porque el bicho tiene inercia al arrancar
                rospy.sleep(rospy.Duration(secs=0.001)) # dependera del tiempo de respuesta del robot             

            prom_act_lin=np.mean(abs(self.vec_ace_lin[0:4]))
            prom_ant_lin=np.mean(abs(self.vec_ace_lin[5:9]))
            prom_act_ang=np.mean(abs(self.vec_ace_ang[0:4]))
            prom_ant_ang=np.mean(abs(self.vec_ace_ang[5:9]))


            
        if ((self.quiere_arrancar == 1) and (self.delta_pos > VEL_BASE) ): 
            if(( prom_act_lin - prom_ant_lin > PENDIENTE_LIN) or (prom_act_ang - prom_ant_ang > PENDIENTE_ANG)):
                print "stasis: esta arrancando "
                self.esta_arrancando=1
                self.quiere_arrancar=0            
                self.esta_parado=0
                self.stasis=0       
            else:
                print "stasis: esta patinando stasis"
                self.stasis=1
                self.esta_parado=0
                self.esta_arrancando=0
                self.quiere_arrancar=0              
                
                now = rospy.Time.now().secs
                while ((rospy.Time.now().secs - now) <= TIEMPO_INERCIA):
                    self.vel_pub.publish(self._twist((-twist_vel_lin),0))   #pega un tiron en sentido contrario a ver si destraba 
                self.vel_pub.publish(self._twist(0, 0))
 
        if ((self.esta_arrancando == 1) and (self.delta_pos > VEL_BASE) ): 
            if(( prom_act_lin - prom_ant_lin < -2) or (prom_act_ang - prom_ant_ang < -2)):
                print "stasis: topo con algo esta patinando stasis"
                self.stasis=1
                self.esta_parado=0
                self.esta_arrancando=0
                self.quiere_arrancar=0 
                
                
                now = rospy.Time.now().secs
                while ((rospy.Time.now().secs - now) <= TIEMPO_INERCIA):
                    self.vel_pub.publish(self._twist((-twist_vel_lin),0))   #pega un tiron en sentido contrario a ver si destraba 
                self.vel_pub.publish(self._twist(0, 0))
                             
                


    def _twist(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        return msg




def arranca():
    stasis = nodo_stasis()
    print "-----Stasis: listo yo te vigilo, si patinas te aviso -------"
    while(1):
        stasis.miro_stasis()
        rospy.sleep(rospy.Duration(secs=1e-4))
 
   
    


if __name__ == '__main__':
   arranca()
