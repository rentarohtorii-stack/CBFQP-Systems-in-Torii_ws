import importlib.util
import math
import unittest
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import Mock
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import LookupException

path=Path(__file__).resolve().parents[1] / 'scripts' / 'mr_ee_target.py'
spec=importlib.util.spec_from_file_location('mr',path)
m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)

class ReceiverTest(unittest.TestCase):
    def setUp(self):
        self.node=SimpleNamespace(frame_id='world',server=Mock(),tf_buffer=Mock(),
                                  publish_target=Mock(),get_logger=Mock(return_value=Mock()))
        self.msg=PoseStamped();self.msg.header.frame_id='Unity'
        self.msg.pose.position.x=0.2;self.msg.pose.position.y=-0.1;self.msg.pose.position.z=0.65
    def receive(self):
        m.MrEeTarget.mr_pose_callback(self.node,self.msg)
    def test_calibrated_position_updates_marker_and_command(self):
        t=TransformStamped();t.header.frame_id='world';t.child_frame_id='Unity'
        t.transform.translation.x=1.0
        t.transform.rotation.z=math.sqrt(0.5);t.transform.rotation.w=math.sqrt(0.5)
        self.node.tf_buffer.lookup_transform.return_value=t
        self.receive()
        pose=self.node.server.setPose.call_args.args[1]
        self.assertAlmostEqual(pose.position.x,1.1)
        self.assertAlmostEqual(pose.position.y,0.2)
        self.assertAlmostEqual(pose.position.z,0.65)
        self.assertEqual(self.node.publish_target.call_args.args[0],'world')
        self.assertEqual(self.node.publish_target.call_args.args[1],pose.position)
        self.assertEqual(self.msg.pose.position.x,0.2)
        self.node.server.applyChanges.assert_called_once()
    def test_world_input_not_converted_twice(self):
        self.msg.header.frame_id='world';self.receive()
        self.node.tf_buffer.lookup_transform.assert_not_called()
        self.assertEqual(self.node.publish_target.call_args.args[1],self.msg.pose.position)
    def test_missing_tf_rejected(self):
        self.node.tf_buffer.lookup_transform.side_effect=LookupException('not calibrated')
        self.receive();self.node.publish_target.assert_not_called();self.node.server.setPose.assert_not_called()
    def test_invalid_position_rejected(self):
        for value in [float('nan'),float('inf')]:
            self.msg.pose.position.x=value;self.receive()
        self.node.publish_target.assert_not_called()
    def test_empty_frame_rejected(self):
        self.msg.header.frame_id='';self.receive();self.node.publish_target.assert_not_called()

if __name__ == '__main__':
    unittest.main()
