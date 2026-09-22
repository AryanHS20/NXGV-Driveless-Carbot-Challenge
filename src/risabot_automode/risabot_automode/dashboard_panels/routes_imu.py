"""Hardware IMU calibration route (moved verbatim from the dashboard handler)."""
import json

from std_msgs.msg import String


def calibrate_imu(ctx, path):
    """Forward JSON payload to hardware IMU calibration via servo_controller."""
    handler = ctx.h
    content_len = int(handler.headers.get('Content-Length', 0))
    body = handler.rfile.read(content_len) if content_len > 0 else b'{}'
    try:
        if ctx.node:
            ctx.node.imu_cal_pub.publish(String(data=body.decode('utf-8')))
            resp = {'ok': True, 'msg': 'Calibration command sent'}
        else:
            resp = {'ok': False, 'error': 'Dashboard node not ready'}
    except Exception as exc:
        resp = {'ok': False, 'error': str(exc)}
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/json')
    handler.send_header('Access-Control-Allow-Origin', '*')
    handler.end_headers()
    handler.wfile.write(json.dumps(resp).encode())
    return True
