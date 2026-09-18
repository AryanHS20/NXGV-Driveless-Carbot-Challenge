"""LiDAR top-view data route (moved verbatim from the dashboard handler)."""
import json


def serve_data(ctx, path):
    """JSON LiDAR points plus tunnel debug overlay values."""
    pts = []
    tunnel = False
    node = ctx.node
    if node:
        with node.lidar_lock:
            pts = list(node.lidar_points)
        with node.data_lock:
            tunnel = bool(node.data.get('tunnel_detected', False))
    payload = {'points': pts, 'tunnel': tunnel}
    # Add tunnel debug info if available (JSON format)
    if node:
        with node.tunnel_debug_lock:
            dbg = node.tunnel_debug
        if dbg:
            try:
                dbg_data = json.loads(dbg)
                payload['left_dist'] = dbg_data.get('l', 0)
                payload['right_dist'] = dbg_data.get('r', 0)
                payload['dist_error'] = dbg_data.get('lat', 0)
                payload['angular_z'] = dbg_data.get('w', 0)
                payload['centerline'] = dbg_data.get('cl', [])
            except Exception:
                pass
    handler = ctx.h
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/json')
    handler.send_header('Access-Control-Allow-Origin', '*')
    handler.end_headers()
    handler.wfile.write(json.dumps(payload).encode())
    return True
