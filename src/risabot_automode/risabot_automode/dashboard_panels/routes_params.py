"""Parameter tuning routes (moved verbatim from the dashboard handler)."""
import json


def get_param(ctx, path):
    """Read one live ROS parameter (plus its saved default, if known)."""
    from urllib.parse import urlparse, parse_qs
    qs = parse_qs(urlparse(path).query)
    node = qs.get('node', [''])[0]
    param = qs.get('param', [''])[0]
    result = {'ok': False}
    if node and param:
        value, err = ctx.get_param(node, param)
        if err is None:
            result = {'ok': True, 'value': value}
            defaults = ctx.param_defaults.get(node, {})
            if param in defaults:
                result['default'] = defaults[param]
        else:
            result = {'ok': False, 'error': err}
    handler = ctx.h
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/json')
    handler.send_header('Access-Control-Allow-Origin', '*')
    handler.end_headers()
    handler.wfile.write(json.dumps(result).encode())
    return True


def set_param(ctx, path):
    """Set one live ROS parameter from a JSON POST body."""
    handler = ctx.h
    content_len = int(handler.headers.get('Content-Length', 0))
    body = handler.rfile.read(content_len)
    try:
        data = json.loads(body)
        node = data['node']
        param = data['param']
        value = str(data['value'])
        ok, msg = ctx.set_param(node, param, value)
        if ok:
            resp = {'ok': True, 'msg': msg}
        else:
            resp = {'ok': False, 'error': msg}
    except Exception as exc:
        resp = {'ok': False, 'error': str(exc)}
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/json')
    handler.send_header('Access-Control-Allow-Origin', '*')
    handler.end_headers()
    handler.wfile.write(json.dumps(resp).encode())
    return True


def save_defaults(ctx, path):
    """Persist changed parameters back to the source params.yaml."""
    handler = ctx.h
    resp = ctx.save_defaults()
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/json')
    handler.send_header('Access-Control-Allow-Origin', '*')
    handler.end_headers()
    handler.wfile.write(json.dumps(resp).encode())
    return True
