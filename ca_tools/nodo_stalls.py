#! /usr/bin/env python

import math
import rospy
import threading
import PyKDL

#import numpy as np

from geometry_msgs.msg import Twist, Pose, Pose2D, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState


INTERVALO_ODOM=100 #mira cada n intervalos de numeros de odometria cargados en _gt_cb
TIEMPO_gt_cb=10000 # en realidad no se cada cuanto consulta _gt_cb

TIEMPO_INERCIA=0.2
TIEMPO_INERCIA2=1
TIEMPO_INERCIA3=2
VEL_BASE = 0.05

CMD_VEL_TOPIC = "/cmd_vel"
STALLS_VEL_TOPIC = "/stalls_vel"
STOP_TOPIC = "/stop_vel"
GT_TOPIC = "/odom"
JOINT_STATES_TOPIC= "/joint_states"

TAM = 100

class nodo_stalls(object):
    """docstring for nodo_stalls."""
    def __init__(self):
        self.pose = Pose2D()
        self.last_pose = None
        self.pose_lock = threading.Lock()   # esto es una clase
        
        self.ruedas_lock = threading.Lock()   # esto es una clase        
        self.pos_rueda_iz= 0
        self.pos_rueda_der= 0
                
        self.vel_Twist = Twist()
        self.vel_lock = threading.Lock()   # esto es una clase
        
        
        rospy.init_node("nodo_stalls")
        self.vel_pub = rospy.Publisher(STALLS_VEL_TOPIC, Twist, queue_size=1)
        self.stop_pub = rospy.Publisher(STOP_TOPIC, Bool, queue_size=1)
        self.pos_pub = rospy.Publisher("/dif_pos", Float64 , queue_size=1)
        self.vel_sub = rospy.Subscriber(CMD_VEL_TOPIC, Twist, self._cmd_cb)  #no se porque ese nombre
        self.gt_sub = rospy.Subscriber(JOINT_STATES_TOPIC, JointState, self._js_cb)
        #self.gt_sub = rospy.Subscriber(GT_TOPIC, Odometry, self._gt_cb)
        # Increase RTF of the simulation to speed it up
        #self._set_rtf(2000.)
        # This sleep avoids issues with the first case
        rospy.sleep(rospy.Duration(secs=1))



    def _cmd_cb(self, msg):     # en este nos llegan las comandos de velocidad entra cada vez que alguien publica vel
        
        self.vel_lock.acquire()
        self.vel_Twist = msg
        self.vel_lock.release()


#    def _gt_cb(self, msg):     # en este nos llegan las posiciones de odometria#

#        self.pose_lock.acquire()
#        self.vel_odom = msg
#        self.pose_lock.release()


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
        self.pos_pub.publish(self.delta_pos)                
        self.ruedas_lock.release()











    def miro_stalls(self):
        #self.stop_pub.publish(False)
        
        twist_vel_lin = self.vel_Twist.linear.x  #solo mueve sobre eje x del robot
        twist_vel_ang = self.vel_Twist.angular.z  # solo gira sobre eje z
        
        #print "velocidad lineal y angular odom twist: ", odom_vel_lin, odom_vel_ang, twist_vel_lin, twist_vel_ang 
        
        if ((twist_vel_lin != 0) or (twist_vel_ang != 0)):

            #se duerme un ratito para darle tiempo de respuesta porque el bicho tiene inercia al arrancar
            rospy.sleep(rospy.Duration(secs=TIEMPO_INERCIA)) # dependera del tiempo de respuesta del robot 
            
            if(  self.delta_pos < VEL_BASE and twist_vel_lin != 0 ):
               print "Stalls: esta trabado al avanzar",self.delta_pos,twist_vel_lin
               self.vel_pub.publish(self._twist(0 , 0))   #pega un tiron a ver si destraba               
               rospy.sleep(rospy.Duration(secs=0.1))                     
               now = rospy.Time.now().secs
               while ((rospy.Time.now().secs - now) <= TIEMPO_INERCIA2) and self.delta_pos < VEL_BASE : 
                   self.vel_pub.publish(self._twist((twist_vel_lin) * 2, 0))   #pega un tiron a ver si destraba
                                      
               if( self.delta_pos < VEL_BASE ): # si sigue trabado
                  print "Stalls: esta trabado al avanzar 2do intento "
                  self.vel_pub.publish(self._twist(0 , 0))   #pega un tiron a ver si destraba
                  rospy.sleep(rospy.Duration(secs=1))                     
                  now1 = rospy.Time.now().secs
                  while ((rospy.Time.now().secs - now1) <= TIEMPO_INERCIA3) and self.delta_pos < VEL_BASE :
                     self.vel_pub.publish(self._twist((twist_vel_lin) * -2, 0))   #pega un tiron en sentido contrario a ver si destraba

                                                                                                         
                  if( self.delta_pos < VEL_BASE  ): # si sigue trabado               
                     #se quedo trabado stop robot      
                     self.vel_pub.publish(self._twist(0, 0))
                     self.stop_pub.publish(True)
                     print "Stalls: esta trabado al avanzar STALL"
               
               print "lo pone en cero             "
               self.vel_pub.publish(self._twist(0, 0))
 

            if( self.delta_pos < VEL_BASE == 0 and twist_vel_ang != 0 ): 
               self.vel_pub.publish(self._twist(0 , 0))   #pega un tiron a ver si destraba               
               rospy.sleep(rospy.Duration(secs=0.1))       
               now = rospy.Time.now().secs
               while ((rospy.Time.now().secs - now) <= TIEMPO_INERCIA2) and self.delta_pos < VEL_BASE :
                   self.vel_pub.publish(self._twist(0, (twist_vel_ang) * 2))   #pega un tiron a ver si destraba
                   

               if( self.delta_pos < VEL_BASE ): #si sigue trabado
                  print "Stalls: esta trabado al girar 2do intento"
                  self.vel_pub.publish(self._twist(0 , 0))   #pega un tiron a ver si destraba
                  rospy.sleep(rospy.Duration(secs=1))                     
                  now = rospy.Time.now().secs
                  while ((rospy.Time.now().secs - now) <= TIEMPO_INERCIA2) and self.delta_pos < VEL_BASE:
                      self.vel_pub.publish(self._twist(0, (twist_vel_ang) * -1))   #pega un tiron en sentido contrario a ver si destraba 
                    
                  
                  if( self.delta_pos < VEL_BASE ): #si sigue trabado
                     #se quedo trabado stop robot                         
                     self.vel_pub.publish(self._twist(0, 0))
                     self.stop_pub.publish(True)                 
                     print "Stalls: esta trabado al girar STALL"
        
               #lo pone en cero             
               self.vel_pub.publish(self._twist(0, 0))



    def _twist(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        return msg




def arranca():
    stalls = nodo_stalls()
    print "----- listo yo te vigilo, si se traban las ruedas es stalls y te doy un envion -------"
    while(1):
        stalls.miro_stalls()
        rospy.sleep(rospy.Duration(secs=1e-4))    
 
   
    


if __name__ == '__main__':
   arranca()
