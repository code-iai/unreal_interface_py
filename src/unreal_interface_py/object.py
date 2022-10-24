import copy
from threading import Lock

import geometry_msgs.msg
import rospy
import datetime

import world_control_msgs.srv
import world_control_msgs.msg

import unreal_interface_py.types

UROSWORDCONTROL_DOMAIN = 'UnrealSim'

DEFAULT_SPAWN_TAG_TYPE = "UnrealInterface"
DEFAULT_SPAWN_TAG_KEY = "spawned"
ERROR_SPAWN_ID_NOT_UNIQUE = "1"
ERROR_SPAWN_OBSTRUCTED = "2"

TRANSPORT_CHECK_TIMEOUT_TIME = 0.3


class SpawnIdNotUnique(Exception):
    """The object that you want to spawn is requested to have an ID that is already present"""

    def __init__(self):
        self.message = "The object that you want to spawn is requested to have an ID that is already present"
        super().__init__(self.message)


class SpawnObstructed(Exception):
    """The object that you want to spawn is requested to be in a place where it would get into collision"""

    def __init__(self):
        self.message = "The object that you want to spawn is requested to be in a place where it would get into " \
                       "collision "
        super().__init__(self.message)


class SpawnFailed(Exception):
    """Generic spawn failed. No details available from UROSWorldControl about the cause"""

    def __init__(self):
        self.message = "Generic spawn failed. No details available from UROSWorldControl about the cause"
        super().__init__(self.message)


class RequestFailed(Exception):
    """Generic request failed for service calls. No details available from UROSWorldControl about the cause.
    Please note, that this should not be raised when you just can't reach a service. It should not be used when
    a rospy.ServiceException would be more approriate. This exception should only be used when the service calls
    returned a non-success state."""

    def __init__(self):
        self.message = "Generic request failed. No details available from UROSWorldControl about the cause"
        super().__init__(self.message)


class Object:
    def init(self):
        self.spawn_client = rospy.ServiceProxy(UROSWORDCONTROL_DOMAIN + "/spawn_model",
                                               world_control_msgs.srv.SpawnModel)
        self.delete_client = rospy.ServiceProxy(UROSWORDCONTROL_DOMAIN + "/delete_model",
                                                world_control_msgs.srv.DeleteModel)
        self.delete_all_client = rospy.ServiceProxy(UROSWORDCONTROL_DOMAIN + "/delete_all",
                                                    world_control_msgs.srv.DeleteAll)
        self.set_pose_client = rospy.ServiceProxy(UROSWORDCONTROL_DOMAIN + "/set_model_pose",
                                                  world_control_msgs.srv.SetModelPose)
        self.get_pose_client = rospy.ServiceProxy(UROSWORDCONTROL_DOMAIN + "/get_model_pose",
                                                  world_control_msgs.srv.GetModelPose)

        self.pose_update_subscriber = None
        self.state_update_subscriber = None
        self.touch_subscriber = None

        # Due to an unsolved bug in ROSWorldControl where objects can't be found in modification operations,
        # we might need to retry the same command
        self.retry_count = 5
        self.retry_delay = 0.4

        self.spawned_objects = dict()  # Object ID as Key and unreal_interface_py.types.ObjectInfo as value

    def transport_available(self):
        """
        Use this method to do a low-level functional check of the transport capabilities.
        In the current version of this library, this method is checking if the necessary
        services/topics are available.
        :return: True if necessary services are available and the corresponding ServiceProxys are up
        """
        try:
            rospy.wait_for_service(self.spawn_client.resolved_name, timeout=TRANSPORT_CHECK_TIMEOUT_TIME)
            rospy.wait_for_service(self.delete_client.resolved_name, timeout=TRANSPORT_CHECK_TIMEOUT_TIME)
            rospy.wait_for_service(self.delete_all_client.resolved_name, timeout=TRANSPORT_CHECK_TIMEOUT_TIME)
            rospy.wait_for_service(self.set_pose_client.resolved_name, timeout=TRANSPORT_CHECK_TIMEOUT_TIME)
            rospy.wait_for_service(self.get_pose_client.resolved_name, timeout=TRANSPORT_CHECK_TIMEOUT_TIME)

        except Exception as e:
            rospy.logdebug(f"Exception catched in transport_available(): {e}")
            return False

        return True

    def spawn_object(self,
                     req: world_control_msgs.srv.SpawnModelRequest) -> unreal_interface_py.types.ObjectInfo.IdType:
        # Set a tag so we can uniquely identify the spawned objects in UE4
        tag = world_control_msgs.msg.Tag()
        tag.type = DEFAULT_SPAWN_TAG_TYPE
        tag.key = DEFAULT_SPAWN_TAG_KEY
        tag.value = str(datetime.datetime.now())

        req.tags.append(tag)

        response: world_control_msgs.srv.SpawnModelResponse = self.spawn_client(req)

        if not response.success:
            if response.etype == ERROR_SPAWN_ID_NOT_UNIQUE:
                raise SpawnIdNotUnique()
            elif response.etype == ERROR_SPAWN_OBSTRUCTED:
                raise SpawnObstructed()
            else:
                raise SpawnFailed()

        object_info = unreal_interface_py.types.ObjectInfo()
        object_info.id = response.id
        object_info.actor_name = response.name

        self.spawned_objects[response.id] = object_info
        return unreal_interface_py.types.ObjectInfo.IdType(response.id)

    def delete_object(self, object_id: unreal_interface_py.types.ObjectInfo.IdType):
        print(f"Deleting object {object_id}")

        req = world_control_msgs.srv.DeleteModelRequest()
        req.id = str(object_id)

        if not self.delete_model(req):
            for i in range(0, self.retry_count):
                rospy.sleep(self.retry_delay)
                print(f"Retrying to delete object {object_id}")

                if not self.delete_model(req):
                    print(f"Deleted object {object_id} successfully in iteration f{i}")
                    return True

            print(f"Couldn't delete object {object_id}")
            return False

        return True

    def delete_model(self, req: world_control_msgs.srv.DeleteModelRequest) -> bool:
        """
        Low level delete func
        :param req:
        :return:
        """
        response = self.delete_client(req)

        # When is a good time to delete stuff? Only if the service call suceeds?
        # But on the other hand, it will also return false if actors are already gone...
        if not response.success:
            print("Warning: DeleteModel failed")
            return False

        if req.id not in self.spawned_objects:
            print(f"DeleteModel couldn't find {req.id} in spawned_objects mapping")
            self.print_all_object_info()
            return False

        del self.spawned_objects[req.id]

        return True

    def is_object_known(self, object_id: unreal_interface_py.types.ObjectInfo.IdType) -> bool:
        if not isinstance(object_id, unreal_interface_py.types.ObjectInfo.IdType):
            raise TypeError()

        return object_id in self.spawned_objects

    def get_object_info(self,
                        object_id: unreal_interface_py.types.ObjectInfo.IdType) -> unreal_interface_py.types.ObjectInfo:
        """
        Access the object info of spawned objects.

        :param object_id:
        :return: None, if object_id can't be found in the object representation. ObjectInfo for object_id otherwise.
        """

        # TODO: Object Infos can be changed asynchronously. Mutex/with:/__enter__/__exit__ needed?
        if self.is_object_known(object_id):
            print(f"Object with id ={object_id} not found in object representation")
            return None

        return self.spawned_objects[object_id]

    def add_object_info(self, object_info: unreal_interface_py.types.ObjectInfo) -> bool:
        if object_info.id == "":
            print("Error in add_object_info(): ID is empty")
            return False

        if self.is_object_known(object_info.id):
            print(f"Warning in add_object_info(): Object with id={object_info.id} is already known")

        self.spawned_objects[object_info.id] = object_info
        return True

    def print_all_object_info(self):
        for key, value in self.spawned_objects.items():
            print(value)

    def spawned_object_count(self):
        return len(self.spawned_objects)

    def set_model_pose(self, req: world_control_msgs.srv.SetModelPoseRequest) -> bool:
        response: world_control_msgs.srv.SetModelPoseResponse = self.set_pose_client(req)

        if not response.success:
            raise RequestFailed()

        return True

    def set_object_pose(self, object_id: unreal_interface_py.types.ObjectInfo.IdType,
                        pose: geometry_msgs.msg.Pose) -> bool:
        """
        Update the object pose of a known object.

        :rtype: bool
        :return: True if successfully set object pose, False otherwise
        """
        req = world_control_msgs.srv.SetModelPoseRequest()
        req.id = str(object_id)
        req.pose = pose

        if not self.is_object_known(object_id):
            print(f"Object {object_id} is not known. Can't set pose.")
            self.print_all_object_info()
            return False

        if not self.set_model_pose(req):
            for i in range(0, self.retry_count):
                rospy.sleep(self.retry_delay)
                print(f"Retrying to set pose for object {object_id}")

                if not self.set_model_pose(req):
                    print(f"Object {object_id} successfully set pose in iteration f{i}")
                    return True

            print(f"Couldn't update pose for object {object_id}")
            return False

        return True

    def get_object_pose(self, object_id: unreal_interface_py.types.ObjectInfo.IdType) -> geometry_msgs.msg.Pose:

        req = world_control_msgs.srv.GetModelPoseRequest()
        req.id = str(object_id)

        response: world_control_msgs.srv.GetModelPoseResponse = self.get_pose_client(req)

        if not response.success:
            raise RequestFailed()

        return response.pose

    def delete_all_spawned_objects(self):
        """
        This method will delete all spawned objects based on its own internal data representation.
        It basically goes one by one through the dict and issues a DeleteModel call for it.

        It is rather slow, so consider using self.delete_all_spawned_objects_by_tag()
        :return: True if all objects could be deleted, False otherwise
        """
        print("Deleting all previously spawned objects")
        spawned_objects = copy.deepcopy(self.spawned_objects)
        return_val = True

        for key, value in spawned_objects.items():
            if not self.delete_object(key):
                print(f"delete_all_spawned_objects: DeleteObject on id={key} failed")
                return_val = False

        return return_val

    def delete_all_spawned_objects_by_tag(self):
        """
        Delete all spawned objects at once by referring to the tag key and type we set in self.spawn_model.
        """
        print(f"Deleting all objects with key {DEFAULT_SPAWN_TAG_KEY} and type {DEFAULT_SPAWN_TAG_TYPE}")
        req = world_control_msgs.srv.DeleteAllRequest()
        req.key_to_delete = DEFAULT_SPAWN_TAG_KEY
        req.type_to_delete = DEFAULT_SPAWN_TAG_TYPE
        req.ignore_value = True

        response: world_control_msgs.srv.DeleteAllResponse = self.delete_all_client(req)

        if not response.success:
            raise RequestFailed()

        self.spawned_objects.clear()

        return True

    def __init__(self):
        self.init()
