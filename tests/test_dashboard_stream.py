"""Exercise the production HTTP handler without ROS or a robot connection."""
import ast
import io
from pathlib import Path
import threading
import types
import unittest


class ShortCondition(threading.Condition):
    def __init__(self):
        super().__init__()
        self.waits = 0

    def wait_for(self, predicate, timeout=None):
        self.waits += 1
        return super().wait_for(predicate, timeout=0.01)


class CameraStreamTests(unittest.TestCase):
    def handler(self, jpeg):
        source = Path(__file__).resolve().parents[1] / 'src/risabot_automode/risabot_automode/dashboard.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'DashboardHandler')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'do_GET')
        node = types.SimpleNamespace(camera_clients_lock=threading.Lock(), num_camera_clients=0,
                                     jpeg_condition=ShortCondition(), frame_id=42, latest_jpeg=jpeg)
        namespace = {'_node_ref': node}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
        handler = types.SimpleNamespace(path='/camera_feed', wfile=io.BytesIO(),
                                        send_response=lambda *a: None, send_header=lambda *a: None,
                                        end_headers=lambda: None)
        namespace['do_GET'](handler)
        return handler, node

    def test_empty_view_waits_then_releases_client(self):
        handler, node = self.handler(None)
        self.assertEqual(node.jpeg_condition.waits, 1)
        self.assertEqual(node.num_camera_clients, 0)
        self.assertEqual(handler.wfile.getvalue(), b'')

    def test_frame_is_sent_once_then_idle_connection_expires(self):
        handler, node = self.handler(b'jpeg')
        self.assertEqual(handler.wfile.getvalue().count(b'--frame'), 1)
        self.assertIn(b'Content-Length: 4', handler.wfile.getvalue())
        self.assertEqual(node.jpeg_condition.waits, 2)
        self.assertEqual(node.num_camera_clients, 0)
