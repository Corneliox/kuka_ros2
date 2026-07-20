import asyncio
import importlib.util
import sys
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / 'kuka_ros2_demo' / 'surgical_control_server.py'


def load_module():
    # Stub ROS modules to allow importing the server in a plain Python test env.
    rclpy = types.ModuleType('rclpy')
    rclpy.node = types.ModuleType('rclpy.node')
    rclpy.action = types.ModuleType('rclpy.action')
    rclpy.executors = types.ModuleType('rclpy.executors')
    rclpy.callback_groups = types.ModuleType('rclpy.callback_groups')

    class Node:
        def __init__(self, *args, **kwargs):
            pass

        def create_publisher(self, *args, **kwargs):
            return None

        def create_service(self, *args, **kwargs):
            return None

        def create_client(self, *args, **kwargs):
            return None

        def get_logger(self):
            return types.SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None)

    class ActionClient:
        def __init__(self, *args, **kwargs):
            pass

        def wait_for_server(self):
            return None

    class ReentrantCallbackGroup:
        pass

    class MultiThreadedExecutor:
        pass

    rclpy.node.Node = Node
    rclpy.action.ActionClient = ActionClient
    rclpy.executors.MultiThreadedExecutor = MultiThreadedExecutor
    rclpy.callback_groups.ReentrantCallbackGroup = ReentrantCallbackGroup

    sys.modules['rclpy'] = rclpy
    sys.modules['rclpy.node'] = rclpy.node
    sys.modules['rclpy.action'] = rclpy.action
    sys.modules['rclpy.executors'] = rclpy.executors
    sys.modules['rclpy.callback_groups'] = rclpy.callback_groups

    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    std_msgs_msg.Int8 = type('Int8', (), {})
    sys.modules['std_msgs'] = std_msgs
    sys.modules['std_msgs.msg'] = std_msgs_msg

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')
    geometry_msgs_msg.Pose = type('Pose', (), {})
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = geometry_msgs_msg

    moveit_msgs = types.ModuleType('moveit_msgs')
    moveit_msgs.__path__ = []
    moveit_msgs_msg = types.ModuleType('moveit_msgs.msg')
    for name in [
        'CollisionObject', 'AttachedCollisionObject', 'PlanningScene',
        'MotionPlanRequest', 'Constraints', 'PositionConstraint',
        'OrientationConstraint', 'BoundingVolume'
    ]:
        setattr(moveit_msgs_msg, name, type(name, (), {}))
    moveit_msgs.msg = moveit_msgs_msg
    sys.modules['moveit_msgs'] = moveit_msgs
    sys.modules['moveit_msgs.msg'] = moveit_msgs_msg

    moveit_msgs_srv = types.ModuleType('moveit_msgs.srv')
    moveit_msgs_srv.ApplyPlanningScene = type('ApplyPlanningScene', (), {})
    sys.modules['moveit_msgs.srv'] = moveit_msgs_srv

    moveit_msgs_action = types.ModuleType('moveit_msgs.action')
    moveit_msgs_action.MoveGroup = type('MoveGroup', (), {})
    sys.modules['moveit_msgs.action'] = moveit_msgs_action

    shape_msgs = types.ModuleType('shape_msgs')
    shape_msgs_msg = types.ModuleType('shape_msgs.msg')
    shape_msgs_msg.SolidPrimitive = type('SolidPrimitive', (), {})
    sys.modules['shape_msgs'] = shape_msgs
    sys.modules['shape_msgs.msg'] = shape_msgs_msg

    surgical_msgs = types.ModuleType('surgical_msgs')
    surgical_msgs_srv = types.ModuleType('surgical_msgs.srv')
    surgical_msgs_srv.TaskPickPlace = type('TaskPickPlace', (), {})
    sys.modules['surgical_msgs'] = surgical_msgs
    sys.modules['surgical_msgs.srv'] = surgical_msgs_srv

    constants = types.ModuleType('kuka_ros2_demo.pick_place_constants')
    constants.ORI_X = 0.0
    constants.ORI_Y = 0.0
    constants.ORI_Z = 0.0
    constants.ORI_W = 1.0
    constants.ORIENTATION_TOLERANCE_RAD = 0.2
    constants.PICK_Z_M = 0.060
    constants.APPROACH_CLEARANCE_M = 0.120
    constants.Z_SAFE_M = 0.250
    constants.WS_X_MIN = 0.00
    constants.WS_X_MAX = 0.59
    constants.WS_Y_MIN = -0.37
    constants.WS_Y_MAX = 0.37
    constants.WS_Z_MIN = 0.030
    constants.WS_Z_MAX = 0.600
    constants.PARK_X_M = 0.24
    constants.PARK_Y_M = 0.06
    constants.PARK_Z_M = 1.02
    constants.PARK_ORI_X = 0.0
    constants.PARK_ORI_Y = 0.0
    constants.PARK_ORI_Z = 0.0
    constants.PARK_ORI_W = 1.0

    def get_pick_z_for_object(object_name):
        return 0.060

    constants.get_pick_z_for_object = get_pick_z_for_object
    sys.modules['kuka_ros2_demo.pick_place_constants'] = constants

    package = types.ModuleType('kuka_ros2_demo')
    package.__path__ = [str(ROOT / 'kuka_ros2_demo')]
    sys.modules['kuka_ros2_demo'] = package

    spec = importlib.util.spec_from_file_location('kuka_ros2_demo.surgical_control_server', MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SurgicalControlServerBoundsTest(unittest.TestCase):
    def test_place_pose_outside_bounds_does_not_trigger_workspace_rejection(self):
        module = load_module()
        server = module.SurgicalControlServer.__new__(module.SurgicalControlServer)

        class DummyLogger:
            def info(self, *args, **kwargs):
                pass

            def warn(self, *args, **kwargs):
                pass

        server.get_logger = lambda: DummyLogger()
        server._task_lock = type('DummyLock', (), {'acquire': lambda self, blocking=False: True, 'release': lambda self: None})()

        async def fake_move_to(self, x, y, z, mode, vel, orientation=None):
            return False

        async def fake_remove_object(self, obj):
            return None

        async def fake_attach_object(self, obj):
            return None

        def fake_fail(self, response, message):
            response.success = False
            response.message = message
            return response

        server.move_to = fake_move_to.__get__(server, module.SurgicalControlServer)
        server.remove_object = fake_remove_object.__get__(server, module.SurgicalControlServer)
        server.attach_object = fake_attach_object.__get__(server, module.SurgicalControlServer)
        server._fail = fake_fail.__get__(server, module.SurgicalControlServer)
        server._send_gripper = lambda state: None

        request = types.SimpleNamespace(
            object_id='cube',
            pick_pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.10, y=0.10, z=0.06)),
            place_pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.3476, y=-0.7690, z=0.1127)),
        )
        response = types.SimpleNamespace(success=True, message='')

        result = asyncio.run(server._run_task(request, response))

        self.assertFalse(result.success)
        self.assertEqual(result.message, 'Transit to pick column failed')


if __name__ == '__main__':
    unittest.main()
