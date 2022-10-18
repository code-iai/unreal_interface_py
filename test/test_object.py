import pytest
import unreal_interface_py.object


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
