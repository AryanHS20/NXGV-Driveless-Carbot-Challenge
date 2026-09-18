"""Shell routes: main page, live data, teach page, 404 (from the handler)."""


def serve_index(ctx, path):
    """Serve the main dashboard page."""
    handler = ctx.h
    handler.send_response(200)
    handler.send_header('Content-Type', 'text/html; charset=utf-8')
    handler.send_header('Cache-Control', 'no-cache')
    handler.end_headers()
    handler.wfile.write(ctx.dashboard_html.encode())
    return True


def serve_data(ctx, path):
    """Serve the aggregated live-data JSON snapshot."""
    handler = ctx.h
    data = ctx.node.get_json() if ctx.node else '{}'
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/json')
    handler.send_header('Access-Control-Allow-Origin', '*')
    handler.end_headers()
    handler.wfile.write(data.encode())
    return True


def serve_teach(ctx, path):
    """Serve the teach-mode page."""
    handler = ctx.h
    handler.send_response(200)
    handler.send_header('Content-Type', 'text/html; charset=utf-8')
    handler.send_header('Cache-Control', 'no-cache')
    handler.end_headers()
    handler.wfile.write(ctx.teach_html.encode())
    return True


def serve_404(ctx, path):
    """Unknown route."""
    handler = ctx.h
    handler.send_response(404)
    handler.end_headers()
    return True
