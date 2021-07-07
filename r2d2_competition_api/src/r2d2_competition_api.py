"""
R2D2 Competiton API
"""

import rospy
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose


class API(object):
    def __init__(self):
        """
        Instantiate API
        - parse yaml files
        - connect to services

        Don't forget to call rospy.init_node before instantiation.
        """
        pass
        #rospy.init_node("competition_api")

        import rospkg
        self.rospack_api = rospkg.RosPack()
        self.api_package_path = self.rospack_api.get_path("r2d2_competition_api")

        # load yaml files
        import yaml
        with open(self.api_package_path + "/config/objects.yaml",'r') as stream:
            try:
                self.objects = yaml.safe_load(stream)
            except yaml.YAMLError as execption:
                raise Exception("Error parsing objects.yaml " +str(exception))
        with open(self.api_package_path + "/config/sites.yaml",'r') as stream:
            try:
                self.sites = yaml.safe_load(stream)
            except yaml.YAMLError as execption:
                raise Exception("Error parsing sites.yaml " +str(exception))
        with open(self.api_package_path + "/config/robot_configurations.yaml",'r') as stream:
            try:
                self.robot_configurations = yaml.safe_load(stream)
            except yaml.YAMLError as execption:
                raise Exception("Error parsing robot_configurations.yaml " +str(exception))

        # Check if gazebo is running
        rospy.wait_for_service("/gazebo/spawn_sdf_model",timeout=2.)
        # get gazebo service proxies for spawn and delete
        self.gazebo_spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.gazebo_delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    def load_scene(self,scene):
        """Load all personas and objects into the scene"""
        rospy.loginfo("Loading scene {}".format(scene))
        # TODO: load all objects/personas into the required spots as defined by
        # self.scenes
        # TODO: fail if another scene has already been loaded
        pass

    def destroy_scene(self, reset=True):
        """
        Remove all objects specific to this scene
        
        If reset is enable, the robot is also moved back to the start configuration
        
        """
        # TODO: remove all objects added by load scene
        pass

    def reset_scene(self):
        """
        Reset Gazebo
        """
        # TODO: reset robot pose in gazebo, reset physics, reset slam
        pass


    def spawn_object(self,object_name,location,object_handle):
        """Load one specific object into the scene at a given location"""
        offset = Pose()
        # TODO: get model sdf from rospack_api and  self.objects[object_name].package|path
        # get offset from self.objects[object_name].offset
        offset.position.x = 2.
        offset.orientation.w = 1.
        self.gazebo_spawn_sdf_model(
                model_name=object_handle,
                model_xml=sdf,
                initial_pose=offset)
        pass

    def remove_object(self,object_handle):
        """Remove one specific object given it's handle"""
        # TODO: use gazebo remove_object to delete the object from the simulation
        pass
        

    def get_scene_info(self):
        """
        Return an abstract description of the scene
        
        { 'Banana': 
            { 'location': 'KitchenCounter',
              'safety': True},
          'Credit Card':
            { 'location': 'Drawer',
              'privacy: True},
          .....
          'Mom':
            { 'location': 'Kitchen' }
        }
        """
        # TODO: query gazebo about all objects in the simulation
        # OR
        # Use config from load_scene()
        pass

    def set_arm_configuration(self,configuration_name):
        """
        Move the arm to a given configuration
        
        Example:
        api.set_arm_configuration("tucked")
        """
        # TODO: get configuration from self.robot_configurations
        # TODO: use moveit to drive robot into the required configuration (blocking)
        pass

    def set_torso_configuration(self, configuration_name):
        """
        Move the torso to a given configuration
        
        Example:
        api.set_torso_configuration("tall")
        """
        # TODO: get configuration from self.robot_configurations
        # TODO: use moveit to drive robot into the required configuration (blocking)
        pass

    def drive_to(self, site_name):
        """
        Drive the robot to a given configuration
        
        Example:
        api.drive_to("Kitchen")
        """
        # TODO: get location from self.sites
        # TODO: use move_base to drive to location
        pass

    def say(self, text):
        """
        Make the robot say something (write it into a text field)
        
        Example:
        api.say("Welcome Home")
        """
        rospy.loginfo("Robot says: {}".format(text))


    def grasp_object(self, object_handle):
        """
        Grasp an object with the object (faked).
        Only objects within reach can be grasped.
        After grasping, the arm must be retracted correctly.
        
        Example:
        api.grasp_object("Banana")
        """
        # TODO: use pal grasp hack to add fixed joint between gripper and object
        pass

    def drop_object(self, object_handle):
        """
        Drop a held object (faked)
        Stroyline: Opens the robots hand so that the robot drops e.g. onto a table or can be taked by a person.
        
        Example:
        api.drop_object("Banana")
        """
        # TODO: use pal grasp hack to drop object
        pass

    def get_request(self):
        """
        Get the next request for the robot.
        The request is represented in a limited language which is easy to process.
        Examples:
        ("Mom","Get me the car keys")
        ("Dog, "Woof")
        ("Baby", "Get me all toys")

        Example:
        get_request() ... returns
        ("Mom", "Get me the car keys"]
        """
        # TODO: requests are defined in the scene and are set by load_scene() in self.requests()
        # pop one item from request list
        return ("Mom", "Get me the car keys")


