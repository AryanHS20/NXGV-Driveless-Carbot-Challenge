"""Record & playback routes (moved verbatim from the dashboard handler)."""
import json
import os

from std_msgs.msg import String


def recording_data(ctx, path):
    """Serve one saved recording JSON from ~/risabot_recordings."""
    from urllib.parse import urlparse, parse_qs
    qs = parse_qs(urlparse(path).query)
    name = qs.get('name', [''])[0]
    result = {'ok': False, 'error': 'No name specified'}
    if name:
        recordings_dir = os.path.expanduser('~/risabot_recordings')
        fpath = os.path.join(recordings_dir, f'{name}.json')
        if os.path.exists(fpath):
            try:
                with open(fpath, 'r') as handle:
                    data = json.load(handle)
                result = {'ok': True, 'data': data}
            except Exception as exc:
                result = {'ok': False, 'error': str(exc)}
        else:
            result = {'ok': False, 'error': 'Recording not found'}
    handler = ctx.h
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/json')
    handler.send_header('Access-Control-Allow-Origin', '*')
    handler.end_headers()
    handler.wfile.write(json.dumps(result).encode())
    return True


def record_playback(ctx, path):
    """Forward a record/playback command to the servo controller."""
    handler = ctx.h
    content_len = int(handler.headers.get('Content-Length', 0))
    body = handler.rfile.read(content_len)
    try:
        data = json.loads(body)
        action = data.get('action', '')
        name = data.get('name', '')
        # Build the command string for servo_controller
        valid_simple = ('record', 'stop', 'playback', 'save', 'list')
        valid_named = ('save', 'load', 'delete', 'set_active')
        if action in valid_simple and not name:
            cmd_str = action
        elif action in valid_named and name:
            cmd_str = f'{action}:{name}'
        else:
            cmd_str = ''
        if cmd_str and ctx.node:
            ctx.node.rp_cmd_pub.publish(String(data=cmd_str))
            resp = {'ok': True, 'msg': f'Sent: {cmd_str}'}
        else:
            resp = {'ok': False, 'error': f'Invalid action: {action}'}
    except Exception as exc:
        resp = {'ok': False, 'error': str(exc)}
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/json')
    handler.send_header('Access-Control-Allow-Origin', '*')
    handler.end_headers()
    handler.wfile.write(json.dumps(resp).encode())
    return True
