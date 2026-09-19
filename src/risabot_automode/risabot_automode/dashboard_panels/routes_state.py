"""Competition-state routes (moved verbatim from the dashboard handler)."""
import json

from std_msgs.msg import String


def reset_odom(ctx, path):
    """Reset display odometry counters (hardware localization unchanged)."""
    handler = ctx.h
    node = ctx.node
    if node:
        with node.data_lock:
            node.data['distance'] = 0.0
            node.data['odom_x'] = 0.0
            node.data['odom_y'] = 0.0
            node.data['odom_yaw'] = 0.0
            node.data['speed'] = 0.0
    resp = {'ok': True, 'msg': 'Display odometry reset; hardware localization unchanged'}
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/json')
    handler.send_header('Access-Control-Allow-Origin', '*')
    handler.end_headers()
    handler.wfile.write(json.dumps(resp).encode())
    return True


def reset_competition(ctx, path):
    """Forward RESET/LAP1/LAP2 competition commands to the state machine."""
    handler = ctx.h
    content_len = int(handler.headers.get('Content-Length', 0))
    body = handler.rfile.read(content_len) if content_len > 0 else b'{}'
    try:
        data = json.loads(body) if body else {}
        cmd = data.get('command', 'reset').upper()
        if cmd in ('RESET', 'LAP1', 'LAP2') and ctx.node:
            msg = String()
            msg.data = cmd
            ctx.node.challenge_pub.publish(msg)
            resp = {'ok': True, 'msg': f'Sent competition command: {cmd}'}
        else:
            resp = {'ok': False, 'error': f'Invalid command: {cmd}'}
    except Exception as exc:
        resp = {'ok': False, 'error': str(exc)}
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/json')
    handler.send_header('Access-Control-Allow-Origin', '*')
    handler.end_headers()
    handler.wfile.write(json.dumps(resp).encode())
    return True
