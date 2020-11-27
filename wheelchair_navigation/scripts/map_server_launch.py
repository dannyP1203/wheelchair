#!/usr/bin/env python

# Launcher del nodo map_server tramite Roslaunch Python API.
# In questo modo si puo gestire il caso in cui la mappa relativo a un certo world non sia presente.
# Se la mappa relativa al world selezionato esiste viene caricata, altrimenti viene usata quella rlativo all'empty_world.


import rospy
import roslaunch
import os.path
from rospkg import RosPack

class MS_Launcher(): 
    def __init__(self):     
        rospy.on_shutdown(self.shutdown)
        self.world_name = rospy.get_param('~world_name', 'default')
        self.package = 'map_server'
        self.executable = 'map_server'
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        self.process = None 
          
    def launch_node(self):
        arg_str = self.check_map_existance()        
        node = roslaunch.core.Node(self.package, self.executable, name="map_server", args=arg_str)
        self.process = self.launch.launch(node)
    
    def check_map_existance(self):
        rp = RosPack()
        pkg_path = rp.get_path('wheelchair_navigation')
        map_path = pkg_path + "/maps/" + self.world_name + "/" + self.world_name + ".yaml"
        
        if os.path.isfile(map_path):
            arguments = map_path
        else:
            arguments = pkg_path + "/maps/empty_world/empty_world.yaml"
            rospy.logwarn("Map Server Launcher: The map of %s.world does not exist. The empty_world one will be used instead." % self.world_name)    
        return arguments
     
    def shutdown(self):
        pass


if __name__=="__main__":

    rospy.init_node('ms_launcher')
    obj = MS_Launcher()
    obj.launch_node()
    
    rospy.spin()