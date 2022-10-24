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
        self.__init__(self.message)


class SpawnObstructed(Exception):
    """The object that you want to spawn is requested to be in a place where it would get into collision"""

    def __init__(self):
        self.message = "The object that you want to spawn is requested to be in a place where it would get into " \
                       "collision "
        self.__init__(self.message)


class SpawnFailed(Exception):
    """Generic spawn failed. No details available from UROSWorldControl about the cause"""

    def __init__(self):
        self.message = "Generic spawn failed. No details available from UROSWorldControl about the cause"
        self.__init__(self.message)


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

    def print_all_object_info(self):
        for key, value in self.spawned_objects.items():
            print(value)

    def spawned_object_count(self):
        return len(self.spawned_objects)

    def __init__(self):
        self.init()
