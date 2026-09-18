"""Camera stream routes (moved verbatim from the dashboard handler)."""
import threading


def serve_feed(ctx, path):
    """MJPEG multipart stream of the active camera view."""
    handler = ctx.h
    handler.send_response(200)
    handler.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
    handler.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0')
    handler.send_header('Connection', 'close')
    handler.send_header('Pragma', 'no-cache')
    handler.end_headers()

    node = ctx.node
    if not node:
        return True

    with node.camera_clients_lock:
        node.num_camera_clients += 1

    try:
        last_frame_id = -1
        while True:
            jpeg_bytes = None
            with node.jpeg_condition:
                # A view switch clears the JPEG before its first frame.
                # Wait for BOTH a new id and an image; otherwise this
                # handler spins continuously while that view is empty.
                ready = node.jpeg_condition.wait_for(
                    lambda: (node.frame_id != last_frame_id
                             and node.latest_jpeg is not None),
                    timeout=5.0)
                if not ready:
                    break  # Release abandoned clients; the player reconnects.

                if node.frame_id != last_frame_id and node.latest_jpeg:
                    last_frame_id = node.frame_id
                    jpeg_bytes = node.latest_jpeg

            if jpeg_bytes:
                frame = (b'--frame\r\n'
                         b'Content-Type: image/jpeg\r\n'
                         b'Content-Length: ' + str(len(jpeg_bytes)).encode() + b'\r\n'
                         b'\r\n' + jpeg_bytes + b'\r\n')
                handler.wfile.write(frame)
            else:
                # Timeout fired but no new frame
                pass
    except Exception:
        pass
    finally:
        with node.camera_clients_lock:
            node.num_camera_clients = max(0, node.num_camera_clients - 1)
    return True


def set_view(ctx, path):
    """Switch the active camera view (and its debug publisher)."""
    from urllib.parse import urlparse, parse_qs
    qs = parse_qs(urlparse(path).query)
    view = qs.get('view', ['raw'])[0]
    node = ctx.node
    if node:
        node.active_camera_view = view
        with node.jpeg_condition:
            node.latest_jpeg = None
            # Force the condition to wake any blocked clients
            node.frame_id += 1
            node.jpeg_condition.notify_all()

    # Auto-toggle show_debug for performance
    def auto_toggle_debug(selected_view):
        nodes_to_enable = set()
        if selected_view == 'line_follower':
            nodes_to_enable.add('line_follower_camera')
        elif selected_view == 'obstacle':
            nodes_to_enable.add('obstacle_avoidance_camera')
        elif selected_view in ('traffic_light', 'signage'):
            nodes_to_enable.add('signage_detector')

        all_nodes = {'line_follower_camera', 'obstacle_avoidance_camera', 'signage_detector'}
        for node_name in all_nodes:
            val_str = 'true' if node_name in nodes_to_enable else 'false'
            ctx.set_param(node_name, 'show_debug', val_str)

    threading.Thread(target=auto_toggle_debug, args=(view,), daemon=True).start()

    handler = ctx.h
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/json')
    handler.end_headers()
    handler.wfile.write(b'{"ok":true}')
    return True
