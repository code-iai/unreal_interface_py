import geometry_msgs.msg


class ObjectInfo(object):
    IdType = str  # type alias. IDs are simply strs right now

    def __init__(self):
        self.id = ObjectInfo.IdType()
        self.actor_name = ""
        self.pose = geometry_msgs.msg.Pose()

    def __str__(self):
        return f"Object {self.id}: {self.actor_name}"
