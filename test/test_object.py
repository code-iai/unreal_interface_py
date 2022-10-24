import pytest
import rospy

import world_control_msgs.srv

import unreal_interface_py.object
import unreal_interface_py.types


@pytest.fixture(scope=u'module')
def module_setup(request):
    pass


@pytest.fixture()
def function_setup(request, module_setup):
    """
    :rtype: WorldObject
    """
    pass


uio = unreal_interface_py.object.Object()


class TestObject(object):

    def test_transport_available(self, function_setup):
        assert uio.transport_available()

    def test_spawn_object(self, function_setup):
        req = world_control_msgs.srv.SpawnModelRequest()

        req.name = "AlbiHimbeerJuice"

        req.pose.position.x = 0
        req.pose.position.y = 0
        req.pose.position.z = 1.00
        req.pose.orientation.x = 0
        req.pose.orientation.y = 0
        req.pose.orientation.z = 0
        req.pose.orientation.w = 1

        req.physics_properties.mobility = req.physics_properties.STATIONARY

        req.actor_label = "TestObjectLabel"

        id_of_spawned_object = uio.spawn_object(req)

        # Check proper type usage
        assert (isinstance(id_of_spawned_object, unreal_interface_py.types.ObjectInfo.IdType))

        assert (uio.spawned_object_count() == 1)

        uio.print_all_object_info()

        rospy.sleep(1.5)

        assert uio.delete_object(id_of_spawned_object)
        rospy.sleep(0.5)

        assert True

    def test_get_non_existing_object_info(self, function_setup):
        assert True

    def test_set_object_pose(self, function_setup):
        assert True

    def test_get_object_pose(self, function_setup):
        assert True

    def test_get_object_pose_async(self, function_setup):
        assert True

    def test_delete_all_spawn_objects(self, function_setup):
        assert True

    def test_delete_all_spawn_objects2(self, function_setup):
        assert True

    def test_delete_all_spawn_objects_by_tag(self, function_setup):
        assert True

    def test_delete_on_spawn_due_to_obstruction(self, function_setup):
        assert True
