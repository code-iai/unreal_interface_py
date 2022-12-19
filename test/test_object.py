import copy
import logging

import pytest
import rospy
import world_control_msgs.srv

import unreal_interface_py.object
import unreal_interface_py.types


# If you want to see the logging output of the module under test, start pytest with:
# pytest --log-cli-level=DEBUG

# Please note that the tests are written for the following UE4 project:
# https://github.com/code-iai/UnrealInterfaceEnv


@pytest.fixture(scope=u'module')
def module_setup(request):
    pass


@pytest.fixture()
def function_setup(request, module_setup):
    """
    :rtype: WorldObject
    """
    pass


def check_pose_equality(pose1, pose2):
    assert pose1.position.x == pytest.approx(pose2.position.x, rel=1e2)
    assert pose1.position.y == pytest.approx(pose2.position.y, rel=1e2)
    assert pose1.position.z == pytest.approx(pose2.position.z, rel=1e2)

    # Quats can be returned with inversed negativity and still be correct
    # Simple conversion by applying abs() on all elements.
    # TODO check if the non-inverted or the fully inverted version of the quat is returned. Not element by element.
    assert abs(pose1.orientation.x) == pytest.approx(abs(pose2.orientation.x), rel=1e2)
    assert abs(pose1.orientation.y) == pytest.approx(abs(pose2.orientation.y), rel=1e2)
    assert abs(pose1.orientation.z) == pytest.approx(abs(pose2.orientation.z), rel=1e2)
    assert abs(pose1.orientation.w) == pytest.approx(abs(pose2.orientation.w), rel=1e2)


uio = unreal_interface_py.object.Object()
uio.logger.setLevel(logging.DEBUG)
rospy.init_node("uio_object_test")


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

        req.actor_label = "GetPoseTestObjectLabel"

        id_of_spawned_object = uio.spawn_object(req)

        # Check proper type usage
        assert isinstance(id_of_spawned_object, unreal_interface_py.types.ObjectInfo.IdType)

        assert uio.spawned_object_count() == 1

        uio.print_all_object_info()

        rospy.sleep(1.0)

        # Check pose of object
        pose = uio.get_object_pose(id_of_spawned_object)

        check_pose_equality(pose, req.pose)

        assert uio.delete_object(id_of_spawned_object)
        rospy.sleep(0.5)

    def test_get_object_pose_async(self, function_setup):
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

        req.actor_label = "GetPoseAsyncTestObjectLabel"

        id_of_spawned_object = uio.spawn_object(req)
        assert uio.spawned_object_count() == 1
        assert uio.is_object_known(id_of_spawned_object)
        rospy.sleep(1.5)  # Sleep a while and wait for a pose update coming in

        object_info = uio.get_object_info(id_of_spawned_object)
        assert object_info is not None
        check_pose_equality(req.pose, object_info.pose)

        ###############################
        # Move object and check again
        ###############################
        new_object_pose = copy.deepcopy(req.pose)
        new_object_pose.position.z = 1.5  # Raise the object a bit

        assert uio.set_object_pose(id_of_spawned_object, new_object_pose)
        rospy.sleep(1.5)  # Wait for a pose update...

        object_info = uio.get_object_info(id_of_spawned_object)
        assert object_info is not None
        check_pose_equality(req.pose, object_info.pose)

        # Tear down
        assert uio.delete_object(id_of_spawned_object)
        assert uio.spawned_object_count() == 0
        rospy.sleep(0.5)

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

        req.actor_label = "DeleteAllTestObjectLabel"

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

        req2.actor_label = "DeleteAllTestObjectLabel2"

        id_of_spawned_object2 = uio.spawn_object(req2)

        assert uio.spawned_object_count() == 2

        assert uio.is_object_known(id_of_spawned_object)
        assert uio.is_object_known(id_of_spawned_object2)

        uio.print_all_object_info()

        rospy.sleep(1.5)

        assert uio.delete_all_spawned_objects()
        assert uio.spawned_object_count() == 0
        rospy.sleep(0.5)

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

        req.actor_label = "DeleteAllTagTestObjectLabel"

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

        req2.actor_label = "DeleteAllTagTestObjectLabel2"

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

        req.actor_label = "ObstructionTestObjectLabel"

        id_of_spawned_object = uio.spawn_object(req)
        rospy.sleep(1.0)

        req2 = copy.deepcopy(req)
        req2.actor_label = "ObstructionTestObjectLabel2"
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

    def test_object_in_unreal(self, function_setup):
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

        req.actor_label = "ObjectInUnrealLabel"

        id_of_spawned_object = uio.spawn_object(req)
        rospy.sleep(1.0)
        assert uio.spawned_object_count() == 1
        assert uio.is_object_known(id_of_spawned_object)

        assert uio.object_in_unreal(id_of_spawned_object)

        assert uio.delete_object(id_of_spawned_object)
        rospy.sleep(1.0)

        assert uio.object_in_unreal(id_of_spawned_object) is False

    def test_detect_single_missing_object_in_unreal(self, function_setup):
        """
        It should be possible to detect if unreal has been closed and we have to resync our belief.
        """
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

        req.actor_label = "detect_single_missing_object_in_unrealLabel"

        id_of_spawned_object = uio.spawn_object(req)
        rospy.sleep(1.0)

        # use the deleteall service directly to fake closing and opening UE4 without touching
        # the internal object representation of unreal_interface
        req = world_control_msgs.srv.DeleteAllRequest()
        req.key_to_delete = unreal_interface_py.object.DEFAULT_SPAWN_TAG_KEY
        req.type_to_delete = unreal_interface_py.object.DEFAULT_SPAWN_TAG_TYPE
        req.ignore_value = True
        uio.delete_all_client(req)
        rospy.sleep(1.0)

        assert uio.object_in_unreal(id_of_spawned_object) is False
        uio.clear_all_object_info()

    def test_all_known_objects_in_unreal(self, function_setup):
        uio.clear_all_object_info()
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

        req.actor_label = "test_detect_missing_object_between_manyAlbi"

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

        req2.actor_label = "test_detect_missing_object_between_manyPfanner"

        id_of_spawned_object2 = uio.spawn_object(req2)
        rospy.sleep(1.5)

        assert uio.all_known_objects_in_unreal()

        # use the deleteall service directly to fake closing and opening UE4 without touching
        # the internal object representation of unreal_interface
        req = world_control_msgs.srv.DeleteAllRequest()
        req.key_to_delete = unreal_interface_py.object.DEFAULT_SPAWN_TAG_KEY
        req.type_to_delete = unreal_interface_py.object.DEFAULT_SPAWN_TAG_TYPE
        req.ignore_value = True
        uio.delete_all_client(req)
        rospy.sleep(1.0)

        assert uio.all_known_objects_in_unreal() is False
