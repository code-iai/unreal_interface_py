import rospy

import world_control_msgs.srv

UROSWORDCONTROL_DOMAIN = 'UnrealSim'

DEFAULT_SPAWN_TAG_TYPE = "UnrealInterface"
DEFAULT_SPAWN_TAG_KEY = "spawned"
ERROR_SPAWN_ID_NOT_UNIQUE = "1"
ERROR_SPAWN_OBSTRUCTED = "2"


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

        self.spawned_objects = dict()  # Object ID as Key and Object Info as value

    def transport_available(self):
        try:
            rospy.wait_for_service(self.spawn_client.resolved_name)
            rospy.wait_for_service(self.delete_client.resolved_name)
            rospy.wait_for_service(self.delete_all_client.resolved_name)
            rospy.wait_for_service(self.set_pose_client.resolved_name)
            rospy.wait_for_service(self.get_pose_client.resolved_name)

        except Exception as e:
            rospy.logdebug(f"Exception catched in transport_available(): {e}")
            return False

        return True

    def __init__(self):
        self.init()
