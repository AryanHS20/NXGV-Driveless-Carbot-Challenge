"""Track-map telemetry route (serves the cached /v4_telemetry document)."""
import json
import time


def v4_telemetry(ctx, path):
    """Serve the latest V4 telemetry snapshot with its age."""
    handler = ctx.h
    node = ctx.node
    doc = None
    if node is not None:
        try:
            doc = node._v4telemetry
            age = time.monotonic() - node._v4telemetry_mono
        except AttributeError:
            doc, age = None, None
    else:
        age = None
    if not isinstance(doc, dict):
        result = {'ok': False, 'error': 'no telemetry yet'}
    else:
        result = {'ok': True, 'age_sec': round(age, 3), 'telemetry': doc}
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/json')
    handler.send_header('Access-Control-Allow-Origin', '*')
    handler.end_headers()
    handler.wfile.write(json.dumps(result).encode())
    return True
