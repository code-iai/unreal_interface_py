import copy

import geometry_msgs
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

        req.pose.position.x = 0.2
        req.pose.position.y = 0.5
        req.pose.position.z = 1.00
        req.pose.orientation.x = 0
        req.pose.orientation.y = 0
        req.pose.orientation.z = 0
        req.pose.orientation.w = 1

        req.physics_properties.mobility = req.physics_properties.STATIONARY

        req.actor_label = "TestObjectLabel"

        id_of_spawned_object = uio.spawn_object(req)

        # Check proper type usage
        assert isinstance(id_of_spawned_object, unreal_interface_py.types.ObjectInfo.IdType)

        assert uio.spawned_object_count() == 1

        assert uio.is_object_known(id_of_spawned_object)

        uio.print_all_object_info()

        rospy.sleep(1.5)

        assert uio.delete_object(id_of_spawned_object)
        assert uio.spawned_object_count() == 0
        rospy.sleep(0.5)

    def test_get_non_existing_object_info(self, function_setup):
        assert True

    def test_set_object_pose(self, function_setup):
        req = world_control_msgs.srv.SpawnModelRequest()

        req.name = "AlbiHimbeerJuice"

        req.pose.position.x = 0.2
        req.pose.position.y = 0.5
        req.pose.position.z = 1.00
        req.pose.orientation.x = 0
        req.pose.orientation.y = 0
        req.pose.orientation.z = 0
        req.pose.orientation.w = 1

        req.physics_properties.mobility = req.physics_properties.DYNAMIC

        req.actor_label = "SetObjectPoseLabel"

        id_of_spawned_object = uio.spawn_object(req)

        # Check proper type usage
        assert (isinstance(id_of_spawned_object, unreal_interface_py.types.ObjectInfo.IdType))

        assert (uio.spawned_object_count() == 1)

        uio.print_all_object_info()

        rospy.sleep(1.0)

        new_object_pose = copy.deepcopy(req.pose)
        new_object_pose.position.z = 1.5  # Raise the object a bit

        assert uio.set_object_pose(id_of_spawned_object, new_object_pose)
        rospy.sleep(1.0)

        assert uio.delete_object(id_of_spawned_object)
        rospy.sleep(1.0)

        # After deleting an object, the method should return
        assert not uio.set_object_pose(id_of_spawned_object, new_object_pose)
        rospy.sleep(1.0)

    def test_get_object_pose(self, function_setup):
        req = world_control_msgs.srv.SpawnModelRequest()

        req.name = "AlbiHimbeerJuice"

        req.pose.position.x = 0.2
        req.pose.position.y = 0.5
        req.pose.position.z = 1.00
        req.pose.orientation.x = -0.383
        req.pose.orientation.y = -0.791
        req.pose.orientation.z = -0.301
        req.pose.orientation.w = 0.370

        req.physics_properties.mobility = req.physics_properties.STATIONARY

        req.actor_label = "TestObjectLabel"

        id_of_spawned_object = uio.spawn_object(req)

        # Check proper type usage
        assert isinstance(id_of_spawned_object, unreal_interface_py.types.ObjectInfo.IdType)

        assert uio.spawned_object_count() == 1

        uio.print_all_object_info()

        rospy.sleep(1.0)

        # Check pose of object
        pose = uio.get_object_pose(id_of_spawned_object)

        assert pose.position.x == pytest.approx(0.2)
        assert pose.position.y == pytest.approx(0.5)
        assert pose.position.z == pytest.approx(1.0)

        # Quats can be returned with inversed negativity and still be correct
        # Simple conversion by applying abs() on all elements.
        # TODO check if the non-inverted or the fully inverted version of the quat is returned. Not element by element.
        assert abs(pose.orientation.x) == pytest.approx(0.383, rel=1e-2)
        assert abs(pose.orientation.y) == pytest.approx(0.791, rel=1e-2)
        assert abs(pose.orientation.z) == pytest.approx(0.301, rel=1e-2)
        assert abs(pose.orientation.w) == pytest.approx(0.370, rel=1e-2)

        assert uio.delete_object(id_of_spawned_object)
        rospy.sleep(0.5)

    def test_get_object_pose_async(self, function_setup):
        assert True

    def test_delete_all_spawned_objects(self, function_setup):
        req = world_control_msgs.srv.SpawnModelRequest()

        req.name = "AlbiHimbeerJuice"

        req.pose.position.x = 0.2
        req.pose.position.y = 0.5
        req.pose.position.z = 1.00
        req.pose.orientation.x = 0
        req.pose.orientation.y = 0
        req.pose.orientation.z = 0
        req.pose.orientation.w = 1

        req.physics_properties.mobility = req.physics_properties.STATIONARY

        req.actor_label = "TestObjectLabel"

        id_of_spawned_object = uio.spawn_object(req)

        ####################################
        # Second object
        ####################################

        req2 = world_control_msgs.srv.SpawnModelRequest()

        req2.name = "PfannerGruneIcetea"

        req2.pose.position.x = 0.2
        req2.pose.position.y = 0.5
        req2.pose.position.z = 1.5
        req2.pose.orientation.x = 0
        req2.pose.orientation.y = 0
        req2.pose.orientation.z = 0
        req2.pose.orientation.w = 1

        req2.physics_properties.mobility = req2.physics_properties.STATIONARY

        req2.actor_label = "TestObjectLabel2"

        id_of_spawned_object2 = uio.spawn_object(req2)

        assert uio.spawned_object_count() == 2

        assert uio.is_object_known(id_of_spawned_object)
        assert uio.is_object_known(id_of_spawned_object2)

        uio.print_all_object_info()

        rospy.sleep(1.5)

        assert uio.delete_all_spawned_objects()
        assert uio.spawned_object_count() == 0
        rospy.sleep(0.5)

    def test_delete_all_spawned_objects2(self, function_setup):
        assert True

    def test_delete_all_spawned_objects_by_tag(self, function_setup):
        req = world_control_msgs.srv.SpawnModelRequest()

        req.name = "AlbiHimbeerJuice"

        req.pose.position.x = 0.2
        req.pose.position.y = 0.5
        req.pose.position.z = 1.00
        req.pose.orientation.x = 0
        req.pose.orientation.y = 0
        req.pose.orientation.z = 0
        req.pose.orientation.w = 1

        req.physics_properties.mobility = req.physics_properties.STATIONARY

        req.actor_label = "TestObjectLabel"

        id_of_spawned_object = uio.spawn_object(req)

        ####################################
        # Second object
        ####################################

        req2 = world_control_msgs.srv.SpawnModelRequest()

        req2.name = "PfannerGruneIcetea"

        req2.pose.position.x = 0.2
        req2.pose.position.y = 0.5
        req2.pose.position.z = 1.5
        req2.pose.orientation.x = 0
        req2.pose.orientation.y = 0
        req2.pose.orientation.z = 0
        req2.pose.orientation.w = 1

        req2.physics_properties.mobility = req2.physics_properties.STATIONARY

        req2.actor_label = "TestObjectLabel2"

        id_of_spawned_object2 = uio.spawn_object(req2)

        assert uio.spawned_object_count() == 2

        assert uio.is_object_known(id_of_spawned_object)
        assert uio.is_object_known(id_of_spawned_object2)

        uio.print_all_object_info()

        rospy.sleep(1.5)

        assert uio.delete_all_spawned_objects_by_tag()
        assert uio.spawned_object_count() == 0
        rospy.sleep(0.5)

    def test_delete_on_spawn_due_to_obstruction(self, function_setup):
        req = world_control_msgs.srv.SpawnModelRequest()

        req.name = "AlbiHimbeerJuice"

        req.pose.position.x = 0.2
        req.pose.position.y = 0.5
        req.pose.position.z = 1.00
        req.pose.orientation.x = 0
        req.pose.orientation.y = 0
        req.pose.orientation.z = 0
        req.pose.orientation.w = 1

        req.physics_properties.mobility = req.physics_properties.STATIONARY

        req.actor_label = "TestObjectLabel"

        id_of_spawned_object = uio.spawn_object(req)
        rospy.sleep(1.0)

        req2 = copy.deepcopy(req)
        req2.actor_label = "TestObjectLabel2"
        req2.spawn_collision_check = True

        # Check that obstruction/collision is detected and exception is raised
        with pytest.raises(unreal_interface_py.object.SpawnObstructed):
            uio.spawn_object(req2)

        # Check proper type usage
        assert uio.spawned_object_count() == 1
        assert uio.is_object_known(id_of_spawned_object)

        rospy.sleep(0.5)

        assert uio.delete_object(id_of_spawned_object)
        assert uio.spawned_object_count() == 0
        rospy.sleep(0.5)
