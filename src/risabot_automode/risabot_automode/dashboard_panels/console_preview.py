"""Run a loopback-only preview without ROS: python -m ...console_preview.

No simulated sensor values, robot publishers, launch commands or driver access.
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace

from . import routes_console as routes


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        ctx = SimpleNamespace(h=self, node=None)
        if self.path in ('/', '/console', '/console/'):
            routes.serve(ctx, self.path)
        elif self.path.startswith('/api/console/'):
            routes.get(ctx, self.path)
        else:
            routes.response(self, 404, dict(ok=False, error='Preview endpoint not found'))

    def do_POST(self):
        routes.action(SimpleNamespace(h=self, node=None), self.path)

    def log_message(self, *_args):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8766)
    args = parser.parse_args()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    print(f'Local draft preview: http://127.0.0.1:{args.port}/console', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if routes._SESSION is not None:
            routes._SESSION.close()


if __name__ == '__main__':
    main()
