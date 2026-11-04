import json
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from klippy.extras import bfp_dual_mesh_math
from scripts import bfp_dual_mesh_simulator


MOONRAKER_FIXTURE = {
    'result': {
        'eventtime': 12.5,
        'status': {
            'bfp_dual_mesh': {
                'enabled': True,
                'baseline_dirty': False,
                'motor_map_verified': True,
                'last_offsets': [0.01, 0.01, -0.01, -0.01],
                'motor_positions': [10., 10., 10., 10.],
                'mode': 'COPY',
                't0_x': 100.,
                't1_x': 500.,
                'y': 200.,
                'mesh_t0': 0.01,
                'mesh_t1': 0.11,
                'mesh_diff': 0.10,
                'fade': 1.,
                'mechanical_delta': 0.,
                'requested_d': 0.1,
                'achieved_d': 0.09,
                'residual': 0.01,
                'authority': 0.9,
                'conditioning_q': 0.2,
                'anchor_error': 0.,
                'hard_scale': 1.,
                'disable_reason': '',
            },
            'toolhead': {
                'position': [100., 200., 10., 0.],
                'homed_axes': 'xyz',
            },
            'bed_mesh': {
                'profile_name': 'default',
                'mesh_min': [0., 0.],
                'mesh_max': [806., 800.],
                'mesh_matrix': [[0., 0.1], [-0.1, 0.2]],
            },
        },
    },
}


class _MoonrakerFixtureHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.server.request_path = self.path
        self.server.authorization = self.headers.get('Authorization')
        body = json.dumps(MOONRAKER_FIXTURE).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        return


class DualMeshMathTest(unittest.TestCase):
    def setUp(self):
        self.left_x = -100.
        self.right_x = 900.
        self.front_y = 0.
        self.rear_y = 800.

    def weights(self, x, y):
        return bfp_dual_mesh_math.gantry_weights(
            x, y, self.left_x, self.right_x,
            self.front_y, self.rear_y)

    def solve(self, w0, w1, requested_d, max_motor_offset=0.2):
        return bfp_dual_mesh_math.solve_offsets(
            w0, w1, requested_d, 0.0025, 0.10, max_motor_offset)

    def test_nominal_solution_preserves_t0_and_pairs_z_motors(self):
        w0 = self.weights(150., 400.)
        w1 = self.weights(650., 400.)

        result = self.solve(w0, w1, 0.1)
        offsets = result[0]

        self.assertAlmostEqual(sum(w * z for w, z in zip(w0, offsets)), 0.)
        self.assertEqual(offsets[0], offsets[1])
        self.assertEqual(offsets[2], offsets[3])
        self.assertGreater(result[2], 0.)

    def test_zero_requested_correction_returns_zero_offsets(self):
        w0 = self.weights(400., 400.)
        w1 = self.weights(405., 400.)

        result = self.solve(w0, w1, 0.)

        self.assertEqual(result[0], [0., 0., 0., 0.])
        self.assertEqual(result[2], 0.)

    def test_motor_offset_is_capped(self):
        w0 = self.weights(150., 400.)
        w1 = self.weights(650., 400.)

        result = self.solve(w0, w1, 0.5, max_motor_offset=0.05)

        self.assertLessEqual(max(abs(v) for v in result[0]), 0.05)
        self.assertLess(result[7], 1.)

    def test_poorly_conditioned_nozzle_positions_are_rejected(self):
        w0 = self.weights(400., 400.)
        w1 = self.weights(405., 400.)

        with self.assertRaisesRegex(ValueError, 'poorly conditioned'):
            self.solve(w0, w1, 0.1)

    def test_moonraker_http_query_maps_status_and_bearer_token(self):
        server = ThreadingHTTPServer(
            ('127.0.0.1', 0), _MoonrakerFixtureHandler)
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        try:
            url = 'http://127.0.0.1:%d' % server.server_port
            result = bfp_dual_mesh_simulator.fetch_moonraker_status(
                url, 'fixture-token')
        finally:
            server.shutdown()
            thread.join()
            server.server_close()

        self.assertEqual(
            server.request_path,
            '/printer/objects/query?bfp_dual_mesh&toolhead&bed_mesh')
        self.assertEqual(server.authorization, 'Bearer fixture-token')
        self.assertTrue(result['tracking_enabled'])
        self.assertEqual(result['params']['x1'], 500.)
        self.assertEqual(result['mesh_grid'][1][0], -0.1)
        self.assertEqual(result['anchor_error'], 0.)


if __name__ == '__main__':
    unittest.main()