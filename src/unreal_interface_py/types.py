import geometry_msgs.msg
import world_control_msgs.srv


class ObjectInfo(object):
    IdType = str  # type alias. IDs are simply strs right now

    def __init__(self):
        self.id = ObjectInfo.IdType()
        self.actor_name = ""
        self.pose = geometry_msgs.msg.Pose()
        self.original_spawn_request = None  # type: world_control_msgs.srv.SpawnModelRequest

    def __str__(self):
        return f"Object {self.id}: {self.actor_name}"
