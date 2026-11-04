#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlsplit
from urllib.request import Request, urlopen

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from klippy.extras import bfp_dual_mesh_math


DEFAULTS = {
    'x0': 220.0,
    'x1': 590.0,
    'y': 400.0,
    'mechanical_delta': 0.12,
    'fade': 1.0,
    'x_tilt': 0.0,
    'y_tilt': 0.0,
    'bowl': 0.32,
    'max_motor_offset': 0.20,
}
LEFT_X = -129.25
RIGHT_X = 949.23
FRONT_Y = 70.65
REAR_Y = 884.25
AUTHORITY_LAMBDA = 0.0025
MIN_CONDITION_Q = 0.10
MAX_NOZZLE_DIFFERENTIAL = 1.0
PLATE_WIDTH = 806.0
PLATE_HEIGHT = 800.0


def _mesh_z(x, y, params):
    x_norm = (x / PLATE_WIDTH) - 0.5
    y_norm = (y / PLATE_HEIGHT) - 0.5
    bowl_shape = 2.0 * (x_norm * x_norm + y_norm * y_norm)
    return (
        params['x_tilt'] * x_norm
        + params['y_tilt'] * y_norm
        + params['bowl'] * bowl_shape
    )


def _number(params, name, minimum, maximum):
    try:
        value = float(params.get(name, DEFAULTS[name]))
    except (TypeError, ValueError):
        raise ValueError('%s must be a number' % name)
    if not math.isfinite(value) or value < minimum or value > maximum:
        raise ValueError('%s must be between %.3f and %.3f'
                         % (name, minimum, maximum))
    return value


def evaluate_scenario(request):
    params = {
        'x0': _number(request, 'x0', 0.0, PLATE_WIDTH),
        'x1': _number(request, 'x1', 0.0, PLATE_WIDTH),
        'y': _number(request, 'y', 0.0, PLATE_HEIGHT),
        'mechanical_delta': _number(request, 'mechanical_delta', -0.6, 0.6),
        'fade': _number(request, 'fade', 0.0, 1.0),
        'x_tilt': _number(request, 'x_tilt', -1.0, 1.0),
        'y_tilt': _number(request, 'y_tilt', -1.0, 1.0),
        'bowl': _number(request, 'bowl', -0.5, 0.5),
        'max_motor_offset': _number(
            request, 'max_motor_offset', 0.05, 0.30),
    }

    mesh_t0 = _mesh_z(params['x0'], params['y'], params)
    mesh_t1 = _mesh_z(params['x1'], params['y'], params)
    mesh_diff = mesh_t1 - mesh_t0
    requested_d = params['mechanical_delta'] + params['fade'] * mesh_diff
    if abs(requested_d) > MAX_NOZZLE_DIFFERENTIAL:
        raise ValueError(
            'Requested differential %.3f mm exceeds the %.1f mm simulator '
            'safety limit' % (requested_d, MAX_NOZZLE_DIFFERENTIAL))

    weights_t0 = bfp_dual_mesh_math.gantry_weights(
        params['x0'], params['y'], LEFT_X, RIGHT_X, FRONT_Y, REAR_Y)
    weights_t1 = bfp_dual_mesh_math.gantry_weights(
        params['x1'], params['y'], LEFT_X, RIGHT_X, FRONT_Y, REAR_Y)

    try:
        (
            offsets,
            condition_q,
            achieved_d,
            residual,
            authority,
            gain,
            uncapped_max,
            hard_scale,
        ) = bfp_dual_mesh_math.solve_offsets(
            weights_t0,
            weights_t1,
            requested_d,
            AUTHORITY_LAMBDA,
            MIN_CONDITION_Q,
            params['max_motor_offset'])
    except ValueError as e:
        raise ValueError(str(e))

    anchor_error = sum(
        weight * offset
        for weight, offset in zip(weights_t0, offsets))

    grid_size = 31
    mesh_grid = [
        [
            _mesh_z(
                PLATE_WIDTH * column / (grid_size - 1),
                PLATE_HEIGHT * row / (grid_size - 1),
                params)
            for column in range(grid_size)
        ]
        for row in range(grid_size)
    ]

    return {
        'params': params,
        'mesh_grid': mesh_grid,
        'mesh_t0': mesh_t0,
        'mesh_t1': mesh_t1,
        'mesh_diff': mesh_diff,
        'requested_d': requested_d,
        'achieved_d': achieved_d,
        'residual': residual,
        'authority': authority,
        'condition_q': condition_q,
        'min_condition_q': MIN_CONDITION_Q,
        'offsets': offsets,
        'uncapped_max': uncapped_max,
        'hard_scale': hard_scale,
        'gain': gain,
        'max_nozzle_differential': MAX_NOZZLE_DIFFERENTIAL,
        'anchor_error': anchor_error,
        'mesh_range': [
            min(min(row) for row in mesh_grid),
            max(max(row) for row in mesh_grid),
        ],
    }


def normalize_moonraker_response(payload, tracking_object='bfp_dual_mesh'):
    if 'error' in payload:
        error = payload['error']
        if isinstance(error, dict):
            raise ValueError(error.get('message', 'Moonraker API error'))
        raise ValueError(str(error))

    result = payload.get('result', {})
    status = result.get('status', {})
    tracking = status.get(tracking_object, {})
    bed_mesh = status.get('bed_mesh', {})
    toolhead = status.get('toolhead', {})
    position = toolhead.get('position', [])

    mesh_grid = bed_mesh.get('mesh_matrix') or []
    if not isinstance(mesh_grid, list):
        mesh_grid = []
    mesh_grid = [
        [float(value) for value in row]
        for row in mesh_grid
        if isinstance(row, list)
    ]
    if mesh_grid and any(len(row) != len(mesh_grid[0]) for row in mesh_grid):
        raise ValueError('Moonraker returned an irregular bed mesh matrix')
    if any(not math.isfinite(value) for row in mesh_grid for value in row):
        raise ValueError('Moonraker returned non-finite bed mesh values')

    mesh_min = bed_mesh.get('mesh_min', [0., 0.])
    mesh_max = bed_mesh.get('mesh_max', [0., 0.])
    mesh_min = [float(value) for value in mesh_min]
    mesh_max = [float(value) for value in mesh_max]
    if len(mesh_min) != 2 or len(mesh_max) != 2:
        raise ValueError('Moonraker returned invalid bed mesh bounds')

    if len(mesh_grid) >= 2 and len(mesh_grid[0]) >= 2:
        mesh_range = [
            min(min(row) for row in mesh_grid),
            max(max(row) for row in mesh_grid),
        ]
    else:
        mesh_range = [0., 0.]

    def finite_or_none(value):
        if value is None:
            return None
        number = float(value)
        if not math.isfinite(number):
            return None
        return number

    t0_x = finite_or_none(tracking.get('t0_x'))
    t1_x = finite_or_none(tracking.get('t1_x'))
    y = finite_or_none(tracking.get('y'))
    if t0_x is None and len(position) > 0:
        t0_x = finite_or_none(position[0])
    if y is None and len(position) > 1:
        y = finite_or_none(position[1])

    return {
        'live': True,
        'tracking_available': tracking_object in status,
        'params': {'x0': t0_x, 'x1': t1_x, 'y': y},
        'mesh_grid': mesh_grid,
        'mesh_bounds': [mesh_min[0], mesh_max[0], mesh_min[1], mesh_max[1]],
        'mesh_range': mesh_range,
        'mesh_profile': bed_mesh.get('profile_name', ''),
        'homed_axes': toolhead.get('homed_axes', ''),
        'tracking_enabled': bool(tracking.get('enabled', False)),
        'baseline_dirty': bool(tracking.get('baseline_dirty', False)),
        'motor_map_verified': tracking.get('motor_map_verified'),
        'mode': tracking.get('mode'),
        'mesh_t0': finite_or_none(tracking.get('mesh_t0')),
        'mesh_t1': finite_or_none(tracking.get('mesh_t1')),
        'mesh_diff': finite_or_none(tracking.get('mesh_diff')),
        'fade': finite_or_none(tracking.get('fade')),
        'mechanical_delta': finite_or_none(
            tracking.get('mechanical_delta')),
        'requested_d': finite_or_none(tracking.get('requested_d')),
        'achieved_d': finite_or_none(tracking.get('achieved_d')),
        'residual': finite_or_none(tracking.get('residual')),
        'authority': finite_or_none(tracking.get('authority')),
        'condition_q': finite_or_none(tracking.get('conditioning_q')),
        'anchor_error': finite_or_none(tracking.get('anchor_error')),
        'min_condition_q': finite_or_none(
            tracking.get('min_condition_q')) or MIN_CONDITION_Q,
        'max_motor_offset': finite_or_none(
            tracking.get('max_motor_offset')),
        'max_nozzle_differential': finite_or_none(
            tracking.get('max_nozzle_differential')) or MAX_NOZZLE_DIFFERENTIAL,
        'offsets': tracking.get('last_offsets', [0., 0., 0., 0.]),
        'motor_positions': tracking.get('motor_positions', []),
        'hard_scale': finite_or_none(tracking.get('hard_scale')),
        'disable_reason': tracking.get('disable_reason', ''),
        'eventtime': finite_or_none(result.get('eventtime')),
    }


def fetch_moonraker_status(moonraker_url, token='', tracking_object='bfp_dual_mesh'):
    parsed = urlsplit(moonraker_url.strip())
    if parsed.scheme not in ('http', 'https') or not parsed.netloc:
        raise ValueError('Enter a Moonraker HTTP URL, for example http://printer:7125')
    if parsed.username or parsed.password or parsed.query or parsed.fragment:
        raise ValueError('Moonraker URL must not contain credentials or query parameters')
    if parsed.path not in ('', '/'):
        raise ValueError('Use the Moonraker server root URL without a path')
    if not tracking_object or any(
            char not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_'
            for char in tracking_object):
        raise ValueError('Tracking object name may contain only letters, numbers, and underscores')

    query = '?%s&toolhead&bed_mesh' % quote(tracking_object, safe='_')
    request = Request(
        parsed.scheme + '://' + parsed.netloc
        + '/printer/objects/query' + query,
        headers={'Accept': 'application/json'})
    if token:
        request.add_header('Authorization', 'Bearer ' + token.strip())
    try:
        with urlopen(request, timeout=4.0) as response:
            payload = json.loads(response.read(2 * 1024 * 1024))
    except HTTPError as e:
        if e.code in (401, 403):
            raise ValueError(
                'Moonraker denied access. Check authorization and the optional token.')
        raise ValueError('Moonraker returned HTTP %d' % e.code)
    except URLError as e:
        raise ValueError('Could not reach Moonraker: %s' % e.reason)
    except TimeoutError:
        raise ValueError('Moonraker request timed out')
    return normalize_moonraker_response(payload, tracking_object)


class SimulatorHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ('/', '/index.html'):
            self.send_error(404)
            return
        page_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'bfp_dual_mesh_simulator.html')
        with open(page_path, 'rb') as page:
            body = page.read()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path not in ('/api/evaluate', '/api/live'):
            self.send_error(404)
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if length > 16384:
                raise ValueError('Request is too large')
            request = json.loads(self.rfile.read(length))
            if self.path == '/api/live':
                result = fetch_moonraker_status(
                    request.get('moonraker_url', ''),
                    request.get('token', ''),
                    request.get('tracking_object', 'bfp_dual_mesh'))
            else:
                result = evaluate_scenario(request)
            response = {'ok': True, 'result': result}
            status = 200
        except (ValueError, TypeError, json.JSONDecodeError) as e:
            response = {'ok': False, 'error': str(e)}
            status = 400
        body = json.dumps(response).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        return


def main():
    parser = argparse.ArgumentParser(
        description='Visual simulator and read-only Moonraker status viewer')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()

    if args.host not in ('127.0.0.1', 'localhost', '::1'):
        parser.error('The simulator only binds to localhost')

    server = ThreadingHTTPServer((args.host, args.port), SimulatorHandler)
    print('BFP dual mesh simulator: http://%s:%d/' %
          (args.host, server.server_port))
    print('Live mode uses read-only Moonraker object queries; no G-code is sent.')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nStopping simulator.')
    finally:
        server.server_close()


if __name__ == '__main__':
    main()