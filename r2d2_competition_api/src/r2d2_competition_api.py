"""
R2D2 Competiton API
"""

import rospy
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty


def parse_pose(info):
    from geometry_msgs.msg import Pose,Point,Quaternion
    position = Point()
    if info.has_key('position'):
        position = Point(**info['position'])
    orientation = Quaternion()
    if info.has_key('orientation'):
        orientation = Quaternion(**info['orientation'])
    pose = Pose(position=position,orientation=orientation)
    return pose


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
        self._rospack_api = rospkg.RosPack()
        self.api_package_path = self._rospack_api.get_path("r2d2_competition_api")

        # load yaml files
        import yaml
        with open(self.api_package_path + "/config/competition.yaml",'r') as stream:
            try:
                configuration = yaml.safe_load(stream)
            except yaml.YAMLError as execption:
                raise Exception("Error parsing objects.yaml " +str(exception))
        try:
            self._objects = configuration['Objects']
            self._locations = configuration['Locations']
            self._scenes = configuration['Scenes']
            self._arm_configurations = configuration['robot_configurations']['arm']
            self._torso_configurations = configuration['robot_configurations']['torso']
        except Exception as ex:
            raise Exception("Failed to load competition configuration ({})".format(ex))

        import moveit_commander
        # Check if gazebo is running
        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model",timeout=0.1)
            # get gazebo service proxies for spawn and delete
            self._gazebo_spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            self._gazebo_delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            self._gazebo_reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
            self._arm_move_group = moveit_commander.MoveGroupCommander("arm")
            #self._torso_move_group = moveit_commander.MoveGroupCommander("torso")

        except rospy.ROSException as ex:
            print("Some simulation services were not found, is the simulation running? ({})".format(ex))

        self._spawned_objects = []
        self._spawned_personas = []
        self._scene_request = None
            
            
    def list_scenes(self):
        return self._scenes.keys()

    def load_scene(self,scene):
        """Load all personas and objects into the scene"""
        rospy.loginfo("Loading scene {}".format(scene))
        # TODO: load all objects/personas into the required spots as defined by
        # self.scenes
        # TODO: fail if another scene has already been loaded
        try:
            my_scene = self._scenes[scene]
        except Exception as ex:
            raise Exception("Scene not found {}: {}".format(scene,ex))
        # spawn objects
        for obj in my_scene['Objects'].keys():
            object_type = my_scene['Objects'][obj]['type']
            object_location = my_scene['Objects'][obj]['location']
            self.spawn_object(object_type,object_location,obj)
        # TODO: spawn persones
        # set request
        self._scene_request = my_scene['Request']

    def destroy_scene(self, reset=True):
        """
        Remove all objects specific to this scene
        
        If reset is enable, the robot is also moved back to the start configuration
        
        """
        # remove all objects added by load scene
        for obj in self._spawned_objects:
            self._gazebo_delete_model(model_name=obj)
        self._spawned_objects = []
        self._scene_request = None
        if reset:
            self._gazebo_reset_world()

    def reset_scene(self):
        """
        Reset Gazebo
        """
        # TODO: reset slam?
        self._gazebo_reset_world()


    def spawn_object(self,object_name,location,object_handle):
        """Load one specific object into the scene at a given location"""
        # loadte SDF model
        try:
            my_object = self._objects[object_name]
        except Exception as ex:
            raise Exception("Object not defined ({})".format(ex))

        sdf_path = self._rospack_api.get_path(my_object['package']) + '/' + my_object['model_path']
        rospy.logdebug("Loading object {} with sdf_path = {}".format(object_name,sdf_path))
        sdf = open(sdf_path).read()

        offset = parse_pose(my_object['offset'])

        try:
            my_location_info = self._locations[location]
        except Exception as ex:
            raise Exception("Location not defined ({})".format(ex))
        my_location = parse_pose(my_location_info)

        # TODO: implement multiplication
        #spawn_location = site * offset
        spawn_location = my_location
        
        # TODO: get model sdf from rospack_api and  self.objects[object_name].package|path
        # get offset from self.objects[object_name].offset
        self._gazebo_spawn_sdf_model(
                model_name=object_handle,
                model_xml=sdf,
                initial_pose=spawn_location,
                reference_frame='world')
        self._spawned_objects.append(object_handle)
        rospy.logdebug("Spawning object type {} with object handle {} at {}".format(object_name,object_handle,spawn_location))

    def remove_object(self,object_handle):
        """Remove one specific object given it's handle"""
        # use gazebo remove_object to delete the object from the simulation
        self._gazebo_delete_model(
                model_name=object_handle)
        self._spawned_objects.remove(object_handle)

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
        try:
            my_configuration = self._arm_configurations[configuration_name]
        except Exception as ex:
            raise Exception("Configuration not defined {}: {}".format(configuration_name,ex))
        self._arm_move_group.go(my_configuration, wait=True)

    def get_arm_configuration(self):
        return self._arm_move_group.get_current_joint_values()

    def set_torso_configuration(self, configuration_name):
        """
        Move the torso to a given configuration
        
        Example:
        api.set_torso_configuration("tall")
        """
        # get configuration from self._torso_configurations
        try:
            my_configuration = self._torso_configurations[configuration_name]
        except Exception as ex:
            raise Exception("Configuration not defined {}: {}".format(configuration_name,ex))
        #joint_goal = self._arm_move_group.get_current_joint_values()
        # use moveit to drive robot into the required configuration (blocking)
        self._arm_move_group.go(my_configuration, wait=True)
    
    def get_torso_configuration(self):
        return self._torso_move_group.get_current_joint_values()

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
        return self._scene_request


