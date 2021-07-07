"""
R2D2 Competiton API
"""

import rospy

class API(object):
    def __init__(self):
        pass
        #rospy.init_node("competition_api")

        # load yaml files

        # get gazebo service proxies for spawn and delete

    def load_scene(self,scene):
        """Load all personas and objects into the scene"""
        rospy.loginfo("Loading scene {}".format(scene))
        pass

    def destroy_scene(self):
        """Remove all objects specific to this scene"""
        pass



    def spawn_object(self,object_name,location,object_handle):
        """Load one specific object into the scene at a given location"""
        pass

    def remove_object(self,object_handle):
        """Remove one specific object given it's handle"""
        pass
        

    def get_scene_info(self):
        """Return an abstract description of the scene"""
        pass

    def set_arm_configuration(self,configuration_name):
        """Move the arm to a given configuration"""
        pass

    def set_torso_configuration(self, configuration_name):
        """Move the torso to a given configuration"""
        pass

    def drive_to(self, site_name):
        """Drive the robot to a given configuration"""
        pass

    def say(self, text):
        """Make the robot say something (write it into a text field)"""
        pass


    def grasp_object(self, object_handle):
        """Grasp an object with the object (faked)"""
        pass

    def drop_object(self, object_handle):
        """Drop a held object (faked)"""
        pass


