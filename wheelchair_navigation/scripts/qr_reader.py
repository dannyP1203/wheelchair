#!/usr/bin/env python

# Implementazione della classe QR_Reader. 
# Stima la posa globale (wrt MAP frame) della carrozzina utilizzando le informazioni pubblicate dal nodo visp_auto_tracker.
# NB: I qr code devono essere orientati con l'angolo vuoto in basso a destra rispetto alla posizione iniziale della carrozzina.
#
# Modalita di funzionamento: 
#    -) gazebo: viene usato l'orientamento da gazebo per la stima della posa
#    -) odom: viene usato l'orientamento dall'odometria per la stima della posa
#    -) imu: viene usato l'orientamento dall'imu per la stima della posa
#    -) qr: l'orientamento viene stimato dall'inclinazione dell'immagine. in questa modalita, per la stima vengono usati solo dati ottenuti dall'immagine.
#
# Subscribed Topics:
#    -) /visp_auto_tracker/object_position_covariance
#    -) /visp_auto_tracker/code_message
#    -) /odometry/ekf_odom                               solo in modalita odom
#    -) /odometry/ground_truth_odom                      solo in modalita gazebo
#    -) /imu/data                                        solo in modalita imu
#    
# Published Topics:
#    -) /qr_pose
# 
# Attributes: 
#    -) pose: 4x4 matrix, posizione stimata
#
# Methods: 
#    -) get_pose(): ritorna True se e possibile calcolare la posa, mettendo il risultato in obj.pose. Viceversa, ritorna False.
#    -) publish(T): pubblica la posa (T 4x4) in formato PoseWithCovarianceStamped sull'output topic /qr_pose.





import rospy
import numpy as np
import tf
import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String



class QR_Reader ():
    def __init__(self, mode = 'gazebo'):
    
        #############      Public Attributes      #############
        
        self.pose = None                                                                # Tf finale MAP -> BASE_FOOTPRINT
        
        
        #############      Protected Attributes      #############
        
        self.t = tf.TransformListener()
        
        self._mode = mode      
        self._available_modes = ("imu", "odom", "qr", "gazebo")
        self._check_params()
        
        self.qr_pos = np.ones((4,1))                                                    # Posizione del qr dal topic object_position_covariance
        self.qr_orientation = [0,0,0,1]                                                 # Orientamento del qr dal topic object_position_covariance
        self.qr_covariance = None                                                       # Covarianza del qr dal topic object_position_covariance
        self.qr_map = np.zeros((3,1))                                                   # Posizione assoluta del qr dal topic code_message
        self.model_orientation = [0,0,0,1]                                              # Orientamento carrozzina dall'odometria
        self.imu_model_orientation = [0,0,0,1]                                          # Orientamento carrozzina dall'imu
        self.gazebo_model_position = [None,None,None]                                   # Posizione carrozzina da Gazebo
        self.gazebo_model_orientation = [0,0,0,1]                                       # Orientamento carrozzina da Gazebo
        
        
        self.room = 0                                                                   # ID dell'ambiente (attualmente inutilizzato)
        self.qr_presence = False                                                        # Presenza o no di un qr nell'inquadratura   
        
        self.T_l2r = np.array([[1,0,0,0],[0,0,1,0],[0,1,0,0],[0,0,0,1]])                # T da frame immagine sx a dx
        self.R_imm_QR = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])           # R da frame immagine dx a QR 
        self.T_cam_bf = None                                                            # Tf CAMERA_LINK -> BASE_FOOTPRINT
        self.R_cam = None                                                               # R CAM -> CAMERA_LINK
        
        self.qr_pos_sub = rospy.Subscriber("/visp_auto_tracker/object_position_covariance", PoseWithCovarianceStamped, self._qr_pos_callback)
        self.qr_msg_sub = rospy.Subscriber("/visp_auto_tracker/code_message", String, self._qr_msg_callback)
        if self._mode == "odom": 
            self.odom_sub = rospy.Subscriber("/odometry/ekf_odom", Odometry, self._odom_sub_callback)
        if self._mode == "gazebo": 
            self.gazebo_sub = rospy.Subscriber("/odometry/ground_truth_odom", Odometry, self._gazebo_sub_callback)
        if self._mode == "imu":     
            self.imu_sub = rospy.Subscriber("/imu/data", Imu, self._imu_sub_callback)
        
        self.pub = rospy.Publisher("qr_pose", PoseWithCovarianceStamped, queue_size = 10)
        
        self._exit_code = 0 
        rospy.on_shutdown(self._shutdown)
        self._init_transforms()
        
        rospy.loginfo("QR Reader started in %s mode." % self._mode)
        
        
        
    #############      Public Methods      #############
    
    def get_pose (self):
    
        if self.qr_presence:    
            # Trasformo posizione e orientamento del qr da frame immagine sx a dx
            new_pos, new_quat = self._left_hand_to_right_hand_frame(self.T_l2r)
            
            # Trovo la rotazione R_qr per compensare la rotazione del frame immagine dovuta al yaw della carrozzina, per poi trasformarlo in QR
            R_qr = self._get_qr_orientation(new_quat)
            
            # Trasformo il vettore qr_pos dal frame immagine al frame QR, che e orientato come MAP e centrato nel qr code.
            R_qr_cam = tf.transformations.concatenate_matrices(R_qr,self.R_imm_QR)
            p_qr_cam = np.dot(R_qr_cam,new_pos) 
            
            # Calcolo la tf MAP -> QR. La traslazione corrisponde alle coordinate scritte nel qr code, mentre l'orientamento e lo stesso.
            T_map_qr = self.t.fromTranslationRotation(self.qr_map.T,(0,0,0,1))
            
            # Calcolo la tf QR -> CAM. La traslazione e il vettore trasformato p_qr_cam, la rotazione e l'orientamento della carrozzina.
            T_qr_cam = R_qr
            T_qr_cam[:,3] = p_qr_cam.T
            
            # Concateno le tf MAP -> QR -> CAM -> CAMERA_LINK -> BASE_FOOTPRINT
            T_map_bf = tf.transformations.concatenate_matrices(T_map_qr,T_qr_cam,self.R_cam,self.T_cam_bf)
            self.pose = T_map_bf

            return True
        return False
    
    def publish (self, T):
        
        pos = tf.transformations.translation_from_matrix(T)
        quat = tf.transformations.quaternion_from_matrix(T)
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = pos[0]
        msg.pose.pose.position.y = pos[1]
        msg.pose.pose.position.z = pos[2]

        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        
        msg.pose.covariance = self.qr_covariance
        
        self.pub.publish(msg)
    
    
    #############      Protected Methods     #############
    
    def _exit_code_bindings (self, code):
        _errors = { 
            20: "Timeout occurred while waiting for transform.",
            21: "An error occurred while getting the transform.",
        } 
        
        _warnings = { 
            10: "", 
        } 
        
        _info = { 
            0: "QR Reader is shutting down.", 
        } 
        
        if code in _info.keys():
                code_type = 0
                exit_message = _info.get(code) 
                
        if code in _warnings.keys():
                code_type = 1
                exit_message = _warnings.get(code) 
                
        if code in _errors.keys():
                code_type = 2
                exit_message = _errors.get(code) 
                
        return (code_type, exit_message)
    
    def _shutdown(self):
        type, error_message = self._exit_code_bindings(self._exit_code)
        
        if type == 0:
            rospy.loginfo(error_message)
        if type == 1:
            rospy.logwarn(error_message)
        if type == 2:
            rospy.logerr(error_message)

    def _qr_pos_callback (self, msg):        
        self.qr_pos[0] = msg.pose.pose.position.x
        self.qr_pos[1] = msg.pose.pose.position.y
        self.qr_pos[2] = msg.pose.pose.position.z
        
        self.qr_orientation[0] = msg.pose.pose.orientation.x
        self.qr_orientation[1] = msg.pose.pose.orientation.y
        self.qr_orientation[2] = msg.pose.pose.orientation.z
        self.qr_orientation[3] = msg.pose.pose.orientation.w
        
        self.qr_covariance = msg.pose.covariance

    def _qr_msg_callback (self, msg):
        # Pattern del messaggio del QR: "#xx#yy#zz#rr"        
        if msg.data != "":
            _var = msg.data.split("#")
            self.qr_map[0] = _var[1]
            self.qr_map[1] = _var[2]
            self.qr_map[2] = _var[3]
            self.room = _var[4]
            self.qr_presence = True
        else:
            self.qr_presence = False
    
    def _odom_sub_callback (self, msg):
        
        self.model_orientation[0] = msg.pose.pose.orientation.x
        self.model_orientation[1] = msg.pose.pose.orientation.y
        self.model_orientation[2] = msg.pose.pose.orientation.z
        self.model_orientation[3] = msg.pose.pose.orientation.w
        
    def _gazebo_sub_callback (self, msg):
        self.gazebo_model_position[0] = msg.pose.pose.position.x
        self.gazebo_model_position[1] = msg.pose.pose.position.y
        self.gazebo_model_position[2] = msg.pose.pose.position.z
        
        self.gazebo_model_orientation[0] = msg.pose.pose.orientation.x
        self.gazebo_model_orientation[1] = msg.pose.pose.orientation.y
        self.gazebo_model_orientation[2] = msg.pose.pose.orientation.z
        self.gazebo_model_orientation[3] = msg.pose.pose.orientation.w
        
    def _imu_sub_callback (self, msg):
    
        self.imu_model_orientation[0] = msg.orientation.x
        self.imu_model_orientation[1] = msg.orientation.y
        self.imu_model_orientation[2] = msg.orientation.z
        self.imu_model_orientation[3] = msg.orientation.w
    
    def _init_transforms (self):
        
        try:
            self.t.waitForTransform("camera_link","base_footprint",rospy.Time(),rospy.Duration(4))
        except tf.Exception:
            self._exit_code = 20
            rospy.signal_shutdown("") 
        
        try:
            _T_cam_bf = self.t.lookupTransform("camera_link","base_footprint",rospy.Time(0))
            self.T_cam_bf = self.t.fromTranslationRotation(_T_cam_bf[0],_T_cam_bf[1])
            self.R_cam = tf.transformations.quaternion_matrix([ 0, -0.7071068, 0, 0.7071068 ])   # Rotazione tra BASE_FOOTPRINT (CAM) e CAMERA_LINK
        except tf.Exception:
            self._exit_code = 21
            rospy.signal_shutdown("")
    
    def _check_params (self):
        
        if not (self._mode in self._available_modes):
            rospy.logwarn("The param mode is not correct. The default gazebo mode will be used.")
            self._mode = "gazebo"

    def _get_qr_orientation (self, qr_orient):
        
        # La rotazione del frame immagine corrisponde all'angolo di yaw della carrozzina
        if self._mode == "imu":
            R_qr = self.t.fromTranslationRotation((0,0,0),self.imu_model_orientation)
            
        if self._mode == "odom":
            R_qr = self.t.fromTranslationRotation((0,0,0),self.model_orientation)
            
        if self._mode == "gazebo":
            R_qr = self.t.fromTranslationRotation((0,0,0),self.gazebo_model_orientation)
            
        # Ricostruisco la rotazione del frame immagine dall'orientamento del qr 
        if self._mode == "qr": 
        
            # Le matrici di rotazione R_1 e R_2 sono state trovate sperimentalmente.
            # Vale la relazione R_1 * R(qr_orientation) * R_2 = R(yaw)   
            # TODO: Inizialmente si puo stimare la R_2 da: R_1 * R(qr_orientation) * R_2 = I (il yaw iniziale e sempre 0). 
            #     In questo modo si rende dinamico l'algoritmo rispetto all'orientamento dei qr.
            
            R_1 = np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
            R_2 = np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]])           
            
            R_qr_orient = self.t.fromTranslationRotation((0,0,0),qr_orient)
            _R_qr = tf.transformations.concatenate_matrices(R_1, R_qr_orient, R_2)
            
            # Normalizzo la R_qr considerando una rotazione elementare sull'asse Z.
            R_qr = self._yaw_rotation_normalization(_R_qr)
            
        return R_qr
  
    def _left_hand_to_right_hand_frame (self, T):
        
        T_inv = np.linalg.inv(T)
        T_left = self.t.fromTranslationRotation(self.qr_pos[0:3].T,self.qr_orientation)
        T_right = tf.transformations.concatenate_matrices(T, T_left, T_inv)  
        
        pos = tf.transformations.translation_from_matrix(T_right)
        quat = tf.transformations.quaternion_from_matrix(T_right)
        return (np.append(pos,[1]), quat)
    
    def _yaw_rotation_normalization (self, R):
        
        roll, pitch, yaw = tf.transformations.euler_from_matrix(R)
        R_norm = tf.transformations.euler_matrix(0,0,yaw)
        
        return R_norm



  
#################################################################################################################################################


if __name__ == "__main__":

    rospy.init_node('qr_reader')
    
    # Check for mode param
    if rospy.has_param('~mode'):
        mode = rospy.get_param('~mode')
        obj = QR_Reader(mode) 
    else:
        obj = QR_Reader()
    
    rate = rospy.Rate(10)   
    rospy.sleep(0.5)
 
    try:
        while not rospy.is_shutdown():
            
            result = obj.get_pose()
            
            if result:
                T = obj.pose
                obj.publish(T)
                
            rate.sleep()  
            
    except rospy.exceptions.ROSInterruptException:
        pass
    