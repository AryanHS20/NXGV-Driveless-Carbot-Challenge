#!/usr/bin/env python3
"""Serve the simulator and proxy the robot dashboard's read-only /data feed.

Run on the laptop: python live_server.py --dashboard http://ROBOT_IP:8080
The server binds to localhost and never sends a command to the robot.
"""

import argparse
import json
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen


class LiveHandler(SimpleHTTPRequestHandler):
    dashboard_url = ''

    def do_GET(self):
        if urlsplit(self.path).path != '/api/live':
            return super().do_GET()
        try:
            request = Request(self.dashboard_url + '/data', headers={'Accept': 'application/json'})
            with urlopen(request, timeout=1.0) as response:
                payload = response.read(256_001)
            if len(payload) > 256_000:
                raise ValueError('Dashboard response is too large')
            data = json.loads(payload)
            if not isinstance(data, dict):
                raise ValueError('Dashboard returned no data object')
            status = 200
        except (HTTPError, URLError, TimeoutError, ValueError, json.JSONDecodeError) as exc:
            data = {'error': f'Dashboard unavailable: {exc}'}
            status = 503
        body = json.dumps(data, separators=(',', ':')).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dashboard', required=True, help='Robot dashboard origin, e.g. http://192.168.1.20:8080')
    parser.add_argument('--port', type=int, default=8765, help='Laptop localhost port (default: 8765)')
    args = parser.parse_args()
    parsed = urlsplit(args.dashboard)
    if parsed.scheme != 'http' or not parsed.hostname or parsed.path not in ('', '/') or parsed.query or parsed.fragment:
        parser.error('--dashboard must be an http://host:port origin')
    if not 1024 <= args.port <= 65535:
        parser.error('--port must be between 1024 and 65535')
    LiveHandler.dashboard_url = args.dashboard.rstrip('/')
    directory = str(Path(__file__).resolve().parent)
    handler = partial(LiveHandler, directory=directory)
    with ThreadingHTTPServer(('127.0.0.1', args.port), handler) as server:
        print(f'Open http://127.0.0.1:{args.port}/ in the laptop browser', flush=True)
        print(f'Reading hardware odometry from {LiveHandler.dashboard_url}/data', flush=True)
        server.serve_forever()


if __name__ == '__main__':
    main()
