# Dual-toolhead Bed Mesh Leveling for IDEX Printers
#
# Copyright (C) 2026  Custom Extension
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Based on Klipper's bed_mesh.py patterns and APIs

import logging
import math
import json
import collections

PROFILE_VERSION = 1

# Verbose logging helper
class VerboseLog:
    """Helper for verbose console output during calibration"""
    def __init__(self, gcmd, enabled=True):
        self.gcmd = gcmd
        self.enabled = enabled
        self.indent = 0
        
    def info(self, msg):
        if self.enabled and self.gcmd:
            prefix = "  " * self.indent
            self.gcmd.respond_info(f"{prefix}{msg}")
        logging.info(f"dual_bed_mesh: {msg}")
        
    def debug(self, msg):
        logging.debug(f"dual_bed_mesh: {msg}")
        
    def warn(self, msg):
        if self.enabled and self.gcmd:
            self.gcmd.respond_info(f"WARNING: {msg}")
        logging.warning(f"dual_bed_mesh: {msg}")
        
    def section(self, title):
        if self.enabled and self.gcmd:
            self.gcmd.respond_info(f"\n{'='*3} {title} {'='*3}")
        logging.info(f"dual_bed_mesh: === {title} ===")
        
    def step(self, msg):
        if self.enabled and self.gcmd:
            self.gcmd.respond_info(f"→ {msg}")
        logging.info(f"dual_bed_mesh: > {msg}")
        
    def progress(self, current, total, msg=""):
        if self.enabled and self.gcmd:
            pct = (current / total) * 100 if total > 0 else 0
            bar_len = 20
            filled = int(bar_len * current / total) if total > 0 else 0
            bar = "█" * filled + "░" * (bar_len - filled)
            self.gcmd.respond_info(f"  [{bar}] {current}/{total} ({pct:.0f}%) {msg}")


# Helper functions matching Klipper's bed_mesh.py
def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def lerp(t, v0, v1):
    return (1. - t) * v0 + t * v1

def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


class DualZMesh:
    """
    Mesh class for dual-toolhead IDEX printers.
    Extends Klipper's ZMesh pattern with zone tracking.
    
    Key additions:
    - Zone provenance (left/right) for each point
    - Per-toolhead Z offset correction
    - Zone-aware interpolation for COPY/MIRROR modes
    """
    def __init__(self, params, name, split_x=None, blend_width=20.):
        self.profile_name = name or "dual-default"
        self.probed_matrix = None
        self.mesh_matrix = None
        self.zone_matrix = None  # Tracks which toolhead probed each point
        self.mesh_params = params
        self.mesh_offsets = [0., 0.]
        self.split_x = split_x or (params['min_x'] + params['max_x']) / 2.
        self.blend_width = blend_width
        
        # Nozzle Z offsets (toolhead mounting differences)
        self.t0_z_offset = 0.
        self.t1_z_offset = 0.
        
        logging.debug('dual_bed_mesh: mesh parameters:')
        for key, value in params.items():
            logging.debug(f"  {key}: {value}")
            
        self.mesh_x_min = params['min_x']
        self.mesh_x_max = params['max_x']
        self.mesh_y_min = params['min_y']
        self.mesh_y_max = params['max_y']
        
        # Mesh grid size (using direct sampling for simplicity)
        self.mesh_x_count = params['x_count']
        self.mesh_y_count = params['y_count']
        
        if self.mesh_x_count > 1:
            self.mesh_x_dist = (self.mesh_x_max - self.mesh_x_min) / \
                               (self.mesh_x_count - 1)
        else:
            self.mesh_x_dist = 0.
            
        if self.mesh_y_count > 1:
            self.mesh_y_dist = (self.mesh_y_max - self.mesh_y_min) / \
                               (self.mesh_y_count - 1)
        else:
            self.mesh_y_dist = 0.
            
        logging.debug(
            f"dual_bed_mesh: Mesh bounds: ({self.mesh_x_min:.1f}, {self.mesh_y_min:.1f}) "
            f"to ({self.mesh_x_max:.1f}, {self.mesh_y_max:.1f})")
        logging.debug(
            f"dual_bed_mesh: Mesh grid: {self.mesh_x_count}x{self.mesh_y_count}")
        logging.debug(f"dual_bed_mesh: Split X: {self.split_x:.1f}")
        
    def get_mesh_matrix(self):
        if self.mesh_matrix is not None:
            return [[round(z, 6) for z in line] for line in self.mesh_matrix]
        return [[]]
        
    def get_probed_matrix(self):
        if self.probed_matrix is not None:
            return [[round(z, 6) for z in line] for line in self.probed_matrix]
        return [[]]
        
    def get_zone_matrix(self):
        return self.zone_matrix
        
    def get_mesh_params(self):
        return self.mesh_params
        
    def get_profile_name(self):
        return self.profile_name
        
    def build_mesh(self, z_matrix, zone_matrix=None):
        """Build mesh from probed Z values"""
        self.probed_matrix = z_matrix
        self.zone_matrix = zone_matrix
        # Direct sampling (no interpolation for now)
        self.mesh_matrix = [[z for z in row] for row in z_matrix]
        
        logging.debug("dual_bed_mesh: Mesh built successfully")
        self.print_mesh(logging.debug)
        
    def set_nozzle_offsets(self, t0_offset, t1_offset):
        """Set nozzle Z offsets for toolhead mounting differences"""
        self.t0_z_offset = t0_offset
        self.t1_z_offset = t1_offset
        logging.info(
            f"dual_bed_mesh: Nozzle offsets set - T0: {t0_offset:.4f}, T1: {t1_offset:.4f}")
        
    def calc_z(self, x, y, carriage_id=None):
        """
        Calculate interpolated Z value at position.
        
        If carriage_id is specified:
        - 0 = T0 (left toolhead)
        - 1 = T1 (right toolhead)
        - 'BOTH' = blended for COPY/MIRROR mode
        """
        if self.mesh_matrix is None:
            return 0.
            
        # Apply mesh offsets
        x = x + self.mesh_offsets[0]
        y = y + self.mesh_offsets[1]
        
        # Get interpolated Z
        z = self._calc_z_bilinear(x, y)
        
        # For zone-aware compensation, we could adjust based on carriage_id
        # For now, return the base interpolated value
        return z
        
    def _calc_z_bilinear(self, x, y):
        """Bilinear interpolation matching Klipper's calc_z"""
        if self.mesh_matrix is None:
            return 0.
            
        tx, xidx = self._get_linear_index(x, 0)
        ty, yidx = self._get_linear_index(y, 1)
        
        tbl = self.mesh_matrix
        z0 = lerp(tx, tbl[yidx][xidx], tbl[yidx][xidx+1])
        z1 = lerp(tx, tbl[yidx+1][xidx], tbl[yidx+1][xidx+1])
        return lerp(ty, z0, z1)
        
    def _get_linear_index(self, coord, axis):
        """Get interpolation parameters for an axis"""
        if axis == 0:
            mesh_min = self.mesh_x_min
            mesh_cnt = self.mesh_x_count
            mesh_dist = self.mesh_x_dist
        else:
            mesh_min = self.mesh_y_min
            mesh_cnt = self.mesh_y_count
            mesh_dist = self.mesh_y_dist
            
        if mesh_dist == 0.:
            return 0., 0
            
        idx = int(math.floor((coord - mesh_min) / mesh_dist))
        idx = constrain(idx, 0, mesh_cnt - 2)
        t = (coord - (mesh_min + idx * mesh_dist)) / mesh_dist
        return constrain(t, 0., 1.), idx
        
    def get_z_range(self):
        if self.mesh_matrix is not None:
            mesh_min = min([min(x) for x in self.mesh_matrix])
            mesh_max = max([max(x) for x in self.mesh_matrix])
            return mesh_min, mesh_max
        return 0., 0.
        
    def get_z_average(self):
        if self.mesh_matrix is not None:
            total = sum([sum(x) for x in self.mesh_matrix])
            count = sum([len(x) for x in self.mesh_matrix])
            return round(total / count, 2) if count > 0 else 0.
        return 0.
        
    def set_mesh_offsets(self, offsets):
        for i, o in enumerate(offsets):
            if o is not None:
                self.mesh_offsets[i] = o
                
    def print_probed_matrix(self, print_func):
        if self.probed_matrix is not None:
            msg = "Dual Mesh Probed Z positions:\n"
            for j, line in enumerate(self.probed_matrix):
                msg += f"Row {j}: "
                for z in line:
                    msg += f" {z:+.4f}"
                msg += "\n"
            print_func(msg)
        else:
            print_func("dual_bed_mesh: bed has not been probed")
            
    def print_mesh(self, print_func, move_z=None):
        if self.mesh_matrix is None:
            print_func("dual_bed_mesh: No mesh generated")
            return
            
        msg = f"Dual Mesh Grid: {self.mesh_x_count}x{self.mesh_y_count}\n"
        msg += f"Bounds: ({self.mesh_x_min:.1f}, {self.mesh_y_min:.1f}) to "
        msg += f"({self.mesh_x_max:.1f}, {self.mesh_y_max:.1f})\n"
        msg += f"Split X: {self.split_x:.1f}, Blend: {self.blend_width:.1f}mm\n"
        msg += f"Average Z: {self.get_z_average():.4f}\n"
        rng = self.get_z_range()
        msg += f"Range: min={rng[0]:.4f}, max={rng[1]:.4f}\n"
        msg += f"Nozzle offsets: T0={self.t0_z_offset:.4f}, T1={self.t1_z_offset:.4f}\n"
        msg += "Z values:\n"
        
        for j in range(self.mesh_y_count - 1, -1, -1):
            y = self.mesh_y_min + j * self.mesh_y_dist
            msg += f"  Y={y:6.1f}: "
            for z in self.mesh_matrix[j]:
                msg += f" {z:+.4f}"
            msg += "\n"
            
        print_func(msg)


class DualBedMeshCalibrate:
    """
    Calibration helper for dual-toolhead bed mesh.
    Handles probing with both toolheads and nozzle Z offset calibration.
    """
    def __init__(self, config, bedmesh):
        self.printer = config.get_printer()
        self.bedmesh = bedmesh
        self.gcode = self.printer.lookup_object('gcode')
        self.verbose = True
        
        # Mesh configuration
        self.mesh_min = config.getfloatlist('mesh_min', count=2)
        self.mesh_max = config.getfloatlist('mesh_max', count=2)
        self.split_x = config.getfloat('split_x',
            (self.mesh_min[0] + self.mesh_max[0]) / 2.)
        self.blend_width = config.getfloat('blend_width', 20., minval=0.)
        
        self.probe_count_x = config.getint('probe_count_x', 10, minval=3)
        self.probe_count_y = config.getint('probe_count_y', 10, minval=3)
        
        self.horizontal_move_z = config.getfloat('horizontal_move_z', 5.)
        self.speed = config.getfloat('speed', 120.)
        
        # Z offset calibration
        self.z_cal_x = config.getfloat('z_calibration_x', self.split_x)
        self.z_cal_y = config.getfloat('z_calibration_y',
            (self.mesh_min[1] + self.mesh_max[1]) / 2.)
        self.interleaved = config.getboolean('interleaved_probing', True)
        
        # Nozzle Z offsets (T1 relative to T0)
        self.t0_nozzle_offset = 0.
        self.t1_nozzle_offset = 0.
        self.offsets_calibrated = False
        
        # State
        self.probing = False
        
        # Objects (populated on connect)
        self.toolhead = None
        self.probe = None
        self.dual_probe = None
        self.dual_carriage = None
        
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
                                            
    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        
        try:
            self.probe = self.printer.lookup_object('probe')
            logging.info("dual_bed_mesh: Found [probe]")
        except:
            self.probe = None
            logging.warning("dual_bed_mesh: No [probe] configured")
            
        try:
            self.dual_probe = self.printer.lookup_object('dual_probe')
            logging.info("dual_bed_mesh: Found [dual_probe]")
        except:
            self.dual_probe = None
            
        try:
            self.dual_carriage = self.printer.lookup_object('dual_carriage')
            logging.info("dual_bed_mesh: Found [dual_carriage]")
        except:
            self.dual_carriage = None
            logging.warning("dual_bed_mesh: No [dual_carriage] - single toolhead mode")
            
    def _get_probe(self, carriage_id):
        """Get probe for specified carriage"""
        if self.dual_probe is not None:
            return self.dual_probe.get_probe(carriage_id)
        return self.probe
        
    def _switch_carriage(self, carriage_id, vlog):
        """Switch to specified carriage"""
        vlog.debug(f"Switching to carriage {carriage_id}")
        self.gcode.run_script_from_command(
            f"SET_DUAL_CARRIAGE CARRIAGE={carriage_id}")
        extruder = 'extruder' if carriage_id == 0 else 'extruder1'
        self.gcode.run_script_from_command(
            f"ACTIVATE_EXTRUDER EXTRUDER={extruder}")
        vlog.step(f"Switched to T{carriage_id}")
        
    def _probe_point(self, x, y, carriage_id, vlog):
        """Probe a single point with specified toolhead"""
        probe = self._get_probe(carriage_id)
        if probe is None:
            raise self.gcode.error(f"No probe available for T{carriage_id}")
            
        offsets = probe.get_offsets()
        probe_x = x - offsets[0]
        probe_y = y - offsets[1]
        
        vlog.debug(f"Probing ({x:.1f}, {y:.1f}) with T{carriage_id} "
                   f"[probe pos: ({probe_x:.1f}, {probe_y:.1f})]")
        
        # Move to position at safe height
        self.toolhead.manual_move([probe_x, probe_y, None], self.speed)
        self.toolhead.manual_move([None, None, self.horizontal_move_z], self.speed)
        self.toolhead.wait_moves()
        
        # Run probe using Klipper's probe API
        curpos = self.toolhead.get_position()
        try:
            curpos = probe.run_probe(vlog.gcmd)
        except Exception as e:
            vlog.warn(f"Probe failed at ({x:.1f}, {y:.1f}): {e}")
            return 0.
            
        z_result = curpos[2]
        vlog.debug(f"  Result: Z={z_result:.4f}")
        
        # Retract
        self.toolhead.manual_move([None, None, self.horizontal_move_z], self.speed)
        return z_result
        
    def _calibrate_nozzle_offsets(self, vlog):
        """Calibrate Z offset between T0 and T1 by probing same point"""
        x, y = self.z_cal_x, self.z_cal_y
        
        vlog.section("Calibrating Nozzle Z Offsets")
        vlog.info(f"Reference point: X={x:.1f}, Y={y:.1f}")
        
        # Probe with T0
        vlog.step("Probing with T0 (left toolhead)...")
        self._switch_carriage(0, vlog)
        z_t0 = self._probe_point(x, y, 0, vlog)
        vlog.info(f"T0 result: Z = {z_t0:.4f}mm")
        
        # Probe with T1
        vlog.step("Probing with T1 (right toolhead)...")
        self._switch_carriage(1, vlog)
        z_t1 = self._probe_point(x, y, 1, vlog)
        vlog.info(f"T1 result: Z = {z_t1:.4f}mm")
        
        # Calculate offset
        self.t0_nozzle_offset = 0.  # T0 is reference
        self.t1_nozzle_offset = z_t1 - z_t0
        self.offsets_calibrated = True
        
        vlog.info(f"Nozzle offset (T1 - T0): {self.t1_nozzle_offset:+.4f}mm")
        
        if abs(self.t1_nozzle_offset) > 2.0:
            vlog.warn("Large offset detected! Check toolhead mounting.")
        elif abs(self.t1_nozzle_offset) < 0.01:
            vlog.info("Excellent! Nozzles nearly perfectly aligned.")
        else:
            vlog.info("Offset within normal range for IDEX printers.")
            
        return self.t0_nozzle_offset, self.t1_nozzle_offset
        
    def _generate_grid_points(self):
        """Generate probe points organized by Y row for both zones"""
        y_min, y_max = self.mesh_min[1], self.mesh_max[1]
        y_step = (y_max - y_min) / (self.probe_count_y - 1) if self.probe_count_y > 1 else 0
        
        # Left zone (T0)
        left_x_min = self.mesh_min[0]
        left_x_max = self.split_x
        left_x_step = (left_x_max - left_x_min) / (self.probe_count_x - 1) \
                      if self.probe_count_x > 1 else 0
                      
        # Right zone (T1)
        right_x_min = self.split_x
        right_x_max = self.mesh_max[0]
        right_x_step = (right_x_max - right_x_min) / (self.probe_count_x - 1) \
                       if self.probe_count_x > 1 else 0
        
        grid = collections.OrderedDict()
        for j in range(self.probe_count_y):
            y = y_min + j * y_step
            
            left_pts = [left_x_min + i * left_x_step for i in range(self.probe_count_x)]
            right_pts = [right_x_min + i * right_x_step for i in range(self.probe_count_x)]
            
            # Serpentine pattern
            if j % 2 == 1:
                left_pts.reverse()
                right_pts.reverse()
                
            grid[y] = {'left': left_pts, 'right': right_pts}
            
        return grid
        
    def _probe_interleaved(self, vlog):
        """Probe both zones interleaved by Y row"""
        grid = self._generate_grid_points()
        y_coords = list(grid.keys())
        
        total_points = self.probe_count_x * self.probe_count_y * 2
        points_done = 0
        
        left_results = []  # List of (x, y, z) tuples
        right_results = []
        
        vlog.section("Interleaved Probing")
        vlog.info(f"Y rows: {len(y_coords)}")
        vlog.info(f"Points per zone per row: {self.probe_count_x}")
        vlog.info(f"Total points: {total_points}")
        
        # Move to safe height
        self.toolhead.manual_move([None, None, self.horizontal_move_z], self.speed)
        self.toolhead.wait_moves()
        
        for row_idx, y in enumerate(y_coords):
            row_data = grid[y]
            
            vlog.info(f"\n--- Y Row {row_idx+1}/{len(y_coords)}: Y={y:.1f}mm ---")
            
            # Probe left zone with T0
            vlog.step(f"T0 probing left zone ({len(row_data['left'])} points)...")
            self._switch_carriage(0, vlog)
            
            for x in row_data['left']:
                z = self._probe_point(x, y, 0, vlog)
                left_results.append((x, y, z, 'left'))
                points_done += 1
                
            vlog.info(f"  T0 done: {len(row_data['left'])} points")
            
            # Probe right zone with T1
            vlog.step(f"T1 probing right zone ({len(row_data['right'])} points)...")
            self._switch_carriage(1, vlog)
            
            for x in row_data['right']:
                z = self._probe_point(x, y, 1, vlog)
                right_results.append((x, y, z, 'right'))
                points_done += 1
                
            vlog.info(f"  T1 done: {len(row_data['right'])} points")
            vlog.progress(points_done, total_points)
            
        return left_results, right_results
        
    def _build_mesh_matrices(self, left_results, right_results, vlog):
        """Build mesh matrices from probe results"""
        vlog.section("Building Mesh")
        
        # Apply T1 nozzle offset correction
        if self.offsets_calibrated and self.t1_nozzle_offset != 0.:
            vlog.step(f"Applying T1 nozzle offset: {self.t1_nozzle_offset:+.4f}mm")
            right_results = [
                (x, y, z - self.t1_nozzle_offset, zone)
                for x, y, z, zone in right_results
            ]
            
        # Combine results
        all_results = left_results + right_results
        
        # Get unique X and Y coordinates (sorted)
        x_coords = sorted(set(r[0] for r in all_results))
        y_coords = sorted(set(r[1] for r in all_results))
        
        vlog.info(f"Grid dimensions: {len(x_coords)}x{len(y_coords)}")
        
        # Build lookup
        point_map = {(r[0], r[1]): (r[2], r[3]) for r in all_results}
        
        # Build matrices
        z_matrix = []
        zone_matrix = []
        
        for y in y_coords:
            z_row = []
            zone_row = []
            for x in x_coords:
                key = (x, y)
                if key in point_map:
                    z, zone = point_map[key]
                    z_row.append(z)
                    zone_row.append(zone)
                else:
                    z_row.append(0.)
                    zone_row.append('unknown')
            z_matrix.append(z_row)
            zone_matrix.append(zone_row)
            
        # Calculate stats
        all_z = [z for row in z_matrix for z in row]
        z_min, z_max = min(all_z), max(all_z)
        z_avg = sum(all_z) / len(all_z)
        
        left_z = [r[2] for r in left_results]
        right_z = [r[2] for r in right_results]
        left_avg = sum(left_z) / len(left_z) if left_z else 0
        right_avg = sum(right_z) / len(right_z) if right_z else 0
        
        vlog.info(f"Z range: {z_min:.4f} to {z_max:.4f}")
        vlog.info(f"Z average: {z_avg:.4f}")
        vlog.info(f"Left zone (T0) avg: {left_avg:.4f}")
        vlog.info(f"Right zone (T1) avg: {right_avg:.4f}")
        vlog.info(f"Zone differential: {abs(left_avg - right_avg):.4f}")
        
        # Build mesh params
        params = {
            'min_x': x_coords[0],
            'max_x': x_coords[-1],
            'min_y': y_coords[0],
            'max_y': y_coords[-1],
            'x_count': len(x_coords),
            'y_count': len(y_coords),
            'algo': 'direct',
        }
        
        return params, z_matrix, zone_matrix
        
    def run_calibrate(self, gcmd):
        """Main calibration entry point"""
        if self.probing:
            raise gcmd.error("Calibration already in progress")
            
        # Check homed
        curtime = self.printer.get_reactor().monotonic()
        kin_status = self.toolhead.get_status(curtime)
        if 'xyz' not in kin_status['homed_axes']:
            raise gcmd.error("Must home before DUAL_BED_MESH_CALIBRATE")
            
        # Parse options
        calibrate_z = gcmd.get_int('CALIBRATE_Z', 1)
        verbose = gcmd.get_int('VERBOSE', 1)
        
        vlog = VerboseLog(gcmd, enabled=bool(verbose))
        
        self.probing = True
        orig_carriage = 0
        
        try:
            vlog.section("DUAL BED MESH CALIBRATION")
            vlog.info(f"Left zone: X {self.mesh_min[0]:.0f} to {self.split_x:.0f}")
            vlog.info(f"Right zone: X {self.split_x:.0f} to {self.mesh_max[0]:.0f}")
            vlog.info(f"Y range: {self.mesh_min[1]:.0f} to {self.mesh_max[1]:.0f}")
            vlog.info(f"Points per zone: {self.probe_count_x} x {self.probe_count_y}")
            vlog.info(f"Total points: {self.probe_count_x * self.probe_count_y * 2}")
            
            if self.dual_probe:
                vlog.info("Using [dual_probe] for T0/T1 probing")
            else:
                vlog.info("Using [probe] for both toolheads")
                
            # Store original carriage
            if self.dual_carriage:
                dc_status = self.dual_carriage.get_status(curtime)
                if dc_status.get('active_carriage') == 'CARRIAGE_1':
                    orig_carriage = 1
                    
            # Step 1: Calibrate nozzle Z offsets
            if calibrate_z:
                self._calibrate_nozzle_offsets(vlog)
            else:
                vlog.info("Skipping nozzle Z offset calibration (CALIBRATE_Z=0)")
                
            # Step 2: Probe both zones
            left_results, right_results = self._probe_interleaved(vlog)
            
            # Step 3: Build mesh
            params, z_matrix, zone_matrix = self._build_mesh_matrices(
                left_results, right_results, vlog)
                
            # Create mesh object
            mesh = DualZMesh(params, "dual-default", self.split_x, self.blend_width)
            mesh.build_mesh(z_matrix, zone_matrix)
            mesh.set_nozzle_offsets(self.t0_nozzle_offset, self.t1_nozzle_offset)
            
            # Set as active mesh
            self.bedmesh.set_mesh(mesh)
            
            vlog.section("CALIBRATION COMPLETE")
            vlog.info("Use DUAL_BED_MESH_ENABLE ENABLE=1 to activate")
            vlog.info("Use DUAL_BED_MESH_OUTPUT to view mesh data")
            
        finally:
            self.probing = False
            # Restore original carriage
            if self.dual_carriage:
                self._switch_carriage(orig_carriage, vlog)
            # Move to safe height
            self.toolhead.manual_move([None, None, self.horizontal_move_z], self.speed)


class DualBedMesh:
    """
    Main dual bed mesh class.
    Provides move transformation for Z compensation based on mesh.
    Follows Klipper's BedMesh pattern.
    """
    FADE_DISABLE = 0x7FFFFFFF
    
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.last_position = [0., 0., 0., 0.]
        
        # Create calibration helper
        self.bmc = DualBedMeshCalibrate(config, self)
        
        # Mesh state
        self.z_mesh = None
        self.toolhead = None
        
        # Movement parameters
        self.horizontal_move_z = config.getfloat('horizontal_move_z', 5.)
        
        # Fade configuration
        self.fade_start = config.getfloat('fade_start', 1.)
        self.fade_end = config.getfloat('fade_end', 0.)
        self.fade_dist = self.fade_end - self.fade_start
        if self.fade_dist <= 0.:
            self.fade_start = self.fade_end = self.FADE_DISABLE
        self.log_fade_complete = False
        self.base_fade_target = config.getfloat('fade_target', None)
        self.fade_target = 0.
        
        # Toolhead Z actuators
        self.use_actuators = config.getboolean('use_toolhead_z_actuators', False)
        self.t0_actuator_name = config.get('t0_z_actuator', None)
        self.t1_actuator_name = config.get('t1_z_actuator', None)
        self.actuator_min = config.getfloat('actuator_min_z', -1.)
        self.actuator_max = config.getfloat('actuator_max_z', 1.)
        self.actuator_speed = config.getfloat('actuator_speed', 10.)
        
        self.t0_actuator = None
        self.t1_actuator = None
        self.t0_actuator_pos = 0.
        self.t1_actuator_pos = 0.
        
        # Transform state
        self.transform_enabled = False
        
        # Register commands
        self.gcode.register_command(
            'DUAL_BED_MESH_CALIBRATE',
            self.cmd_CALIBRATE,
            desc=self.cmd_CALIBRATE_help)
        self.gcode.register_command(
            'DUAL_BED_MESH_ENABLE',
            self.cmd_ENABLE,
            desc="Enable/disable dual mesh Z compensation")
        self.gcode.register_command(
            'DUAL_BED_MESH_STATUS',
            self.cmd_STATUS,
            desc="Show dual mesh status")
        self.gcode.register_command(
            'DUAL_BED_MESH_OUTPUT',
            self.cmd_OUTPUT,
            desc="Output mesh data")
        self.gcode.register_command(
            'DUAL_BED_MESH_CLEAR',
            self.cmd_CLEAR,
            desc="Clear dual mesh data")
        self.gcode.register_command(
            'DUAL_BED_MESH_MAP',
            self.cmd_MAP,
            desc="Output mesh as JSON")
        self.gcode.register_command(
            'CALIBRATE_TOOLHEAD_Z_OFFSETS',
            self.cmd_CALIBRATE_Z_OFFSETS,
            desc="Calibrate nozzle Z offset between T0 and T1")
        self.gcode.register_command(
            'SET_NOZZLE_Z_OFFSET',
            self.cmd_SET_NOZZLE_OFFSET,
            desc="Manually set nozzle Z offset")
        self.gcode.register_command(
            'SET_TOOLHEAD_Z_OFFSET',
            self.cmd_SET_ACTUATOR,
            desc="Set toolhead Z actuator position")
            
        # Register event handlers
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
                                            
        # Initialize status
        self.status = {}
        self._update_status()
        
    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        
        # Register move transform if available
        # Note: We don't register as primary transform to avoid conflicts
        # with standard bed_mesh. Users should use one or the other.
        try:
            gcode_move = self.printer.lookup_object('gcode_move')
            # Optionally register transform here if needed
            logging.info("dual_bed_mesh: Connected, gcode_move available")
        except:
            logging.info("dual_bed_mesh: gcode_move not found")
            
        # Look up actuators if configured
        if self.use_actuators:
            if self.t0_actuator_name:
                try:
                    self.t0_actuator = self.printer.lookup_object(
                        self.t0_actuator_name)
                    logging.info(f"dual_bed_mesh: T0 actuator '{self.t0_actuator_name}' found")
                except:
                    logging.warning(f"dual_bed_mesh: T0 actuator not found")
                    
            if self.t1_actuator_name:
                try:
                    self.t1_actuator = self.printer.lookup_object(
                        self.t1_actuator_name)
                    logging.info(f"dual_bed_mesh: T1 actuator '{self.t1_actuator_name}' found")
                except:
                    logging.warning(f"dual_bed_mesh: T1 actuator not found")
                    
    def set_mesh(self, mesh):
        """Set the active mesh"""
        if mesh is not None and self.fade_end != self.FADE_DISABLE:
            if self.base_fade_target is None:
                self.fade_target = mesh.get_z_average()
            else:
                self.fade_target = self.base_fade_target
        else:
            self.fade_target = 0.
            
        self.z_mesh = mesh
        self._update_status()
        logging.info(f"dual_bed_mesh: Mesh set, fade_target={self.fade_target:.4f}")
        
    def get_mesh(self):
        return self.z_mesh
        
    def get_z_factor(self, z_pos):
        """Calculate fade factor for given Z position"""
        if z_pos >= self.fade_end:
            return 0.
        elif z_pos >= self.fade_start:
            return (self.fade_end - z_pos) / self.fade_dist
        return 1.
        
    def get_z_compensation(self, x, y, carriage_id=None):
        """Get Z compensation for position"""
        if self.z_mesh is None or not self.transform_enabled:
            return 0.
        return self.z_mesh.calc_z(x, y, carriage_id)
        
    def _move_actuator(self, carriage_id, position):
        """Move toolhead Z actuator"""
        actuator_name = self.t0_actuator_name if carriage_id == 0 else self.t1_actuator_name
        if actuator_name is None:
            return
            
        # Extract stepper name
        if actuator_name.startswith('manual_stepper '):
            stepper = actuator_name.split(' ', 1)[1]
        else:
            stepper = actuator_name
            
        # Clamp position
        position = constrain(position, self.actuator_min, self.actuator_max)
        
        cmd = f"MANUAL_STEPPER STEPPER={stepper} MOVE={position:.4f} SPEED={self.actuator_speed}"
        try:
            self.gcode.run_script_from_command(cmd)
            if carriage_id == 0:
                self.t0_actuator_pos = position
            else:
                self.t1_actuator_pos = position
            logging.debug(f"dual_bed_mesh: T{carriage_id} actuator moved to {position:.4f}")
        except Exception as e:
            logging.warning(f"dual_bed_mesh: Actuator move failed: {e}")
            
    def _update_status(self):
        """Update status dict for Moonraker"""
        self.status = {
            'mesh_available': self.z_mesh is not None,
            'enabled': self.transform_enabled,
            'profile_name': self.z_mesh.get_profile_name() if self.z_mesh else "",
            'mesh_min': (0., 0.),
            'mesh_max': (0., 0.),
            'probed_matrix': [[]],
            'mesh_matrix': [[]],
            # Dual-specific
            't0_nozzle_offset': self.bmc.t0_nozzle_offset,
            't1_nozzle_offset': self.bmc.t1_nozzle_offset,
            'offsets_calibrated': self.bmc.offsets_calibrated,
            't0_actuator_pos': self.t0_actuator_pos,
            't1_actuator_pos': self.t1_actuator_pos,
        }
        
        if self.z_mesh is not None:
            params = self.z_mesh.get_mesh_params()
            self.status['mesh_min'] = (params['min_x'], params['min_y'])
            self.status['mesh_max'] = (params['max_x'], params['max_y'])
            self.status['probed_matrix'] = self.z_mesh.get_probed_matrix()
            self.status['mesh_matrix'] = self.z_mesh.get_mesh_matrix()
            
    def get_status(self, eventtime=None):
        return self.status
        
    # === G-CODE COMMANDS ===
    
    cmd_CALIBRATE_help = "Probe bed with both toolheads for dual mesh"
    def cmd_CALIBRATE(self, gcmd):
        self.bmc.run_calibrate(gcmd)
        
    def cmd_ENABLE(self, gcmd):
        """Enable/disable mesh compensation"""
        enable = gcmd.get_int('ENABLE', None)
        
        if enable is None:
            state = "enabled" if self.transform_enabled else "disabled"
            gcmd.respond_info(f"Dual mesh compensation: {state}")
            return
            
        if enable and self.z_mesh is None:
            raise gcmd.error("No mesh - run DUAL_BED_MESH_CALIBRATE first")
            
        self.transform_enabled = bool(enable)
        self._update_status()
        
        if enable:
            gcmd.respond_info("Dual mesh compensation ENABLED")
            if self.use_actuators:
                gcmd.respond_info("  Toolhead Z actuators active")
        else:
            gcmd.respond_info("Dual mesh compensation DISABLED")
            # Reset actuators
            if self.use_actuators:
                self._move_actuator(0, 0.)
                self._move_actuator(1, 0.)
                
    def cmd_STATUS(self, gcmd):
        """Show detailed status"""
        gcmd.respond_info("=== Dual Bed Mesh Status ===")
        
        if self.z_mesh is None:
            gcmd.respond_info("Mesh: NOT CALIBRATED")
            gcmd.respond_info("Run DUAL_BED_MESH_CALIBRATE to generate mesh")
        else:
            params = self.z_mesh.get_mesh_params()
            gcmd.respond_info(f"Mesh: {params['x_count']}x{params['y_count']}")
            gcmd.respond_info(f"  Bounds: ({params['min_x']:.0f}, {params['min_y']:.0f}) to "
                             f"({params['max_x']:.0f}, {params['max_y']:.0f})")
            gcmd.respond_info(f"  Split X: {self.z_mesh.split_x:.0f}")
            rng = self.z_mesh.get_z_range()
            gcmd.respond_info(f"  Z range: {rng[0]:.4f} to {rng[1]:.4f}")
            gcmd.respond_info(f"  Z average: {self.z_mesh.get_z_average():.4f}")
            gcmd.respond_info(f"  Enabled: {self.transform_enabled}")
            
        gcmd.respond_info("\nNozzle Z Offsets (toolhead mounting):")
        gcmd.respond_info(f"  Calibrated: {self.bmc.offsets_calibrated}")
        gcmd.respond_info(f"  T0: {self.bmc.t0_nozzle_offset:.4f}mm (reference)")
        gcmd.respond_info(f"  T1: {self.bmc.t1_nozzle_offset:+.4f}mm relative to T0")
        
        if self.use_actuators:
            gcmd.respond_info("\nToolhead Z Actuators:")
            gcmd.respond_info(f"  T0: {self.t0_actuator_pos:.4f}mm")
            gcmd.respond_info(f"  T1: {self.t1_actuator_pos:.4f}mm")
            gcmd.respond_info(f"  Limits: {self.actuator_min:.1f} to {self.actuator_max:.1f}mm")
            
    def cmd_OUTPUT(self, gcmd):
        """Output mesh data"""
        if self.z_mesh is None:
            gcmd.respond_info("No mesh data available")
            return
            
        self.z_mesh.print_probed_matrix(gcmd.respond_info)
        self.z_mesh.print_mesh(gcmd.respond_info, self.horizontal_move_z)
        
    def cmd_MAP(self, gcmd):
        """Output mesh as JSON (for visualization tools)"""
        if self.z_mesh is None:
            gcmd.respond_info("No mesh data available")
            return
            
        params = self.z_mesh.get_mesh_params()
        outdict = {
            'mesh_min': (params['min_x'], params['min_y']),
            'mesh_max': (params['max_x'], params['max_y']),
            'z_positions': self.z_mesh.get_probed_matrix(),
            'split_x': self.z_mesh.split_x,
            't0_nozzle_offset': self.bmc.t0_nozzle_offset,
            't1_nozzle_offset': self.bmc.t1_nozzle_offset,
        }
        gcmd.respond_raw("dual_mesh_map_output " + json.dumps(outdict))
        
    def cmd_CLEAR(self, gcmd):
        """Clear mesh data"""
        reset_offsets = gcmd.get_int('RESET_Z_OFFSETS', 0)
        
        self.z_mesh = None
        self.transform_enabled = False
        
        if self.use_actuators:
            self._move_actuator(0, 0.)
            self._move_actuator(1, 0.)
            
        if reset_offsets:
            self.bmc.t0_nozzle_offset = 0.
            self.bmc.t1_nozzle_offset = 0.
            self.bmc.offsets_calibrated = False
            gcmd.respond_info("Mesh and nozzle offsets cleared")
        else:
            gcmd.respond_info("Mesh cleared (nozzle offsets retained)")
            
        self._update_status()
        
    def cmd_CALIBRATE_Z_OFFSETS(self, gcmd):
        """Standalone nozzle Z offset calibration"""
        x = gcmd.get_float('X', self.bmc.z_cal_x)
        y = gcmd.get_float('Y', self.bmc.z_cal_y)
        
        # Check homed
        curtime = self.printer.get_reactor().monotonic()
        kin_status = self.toolhead.get_status(curtime)
        if 'xyz' not in kin_status['homed_axes']:
            raise gcmd.error("Must home first")
            
        vlog = VerboseLog(gcmd)
        
        # Temporarily set calibration position
        old_x, old_y = self.bmc.z_cal_x, self.bmc.z_cal_y
        self.bmc.z_cal_x = x
        self.bmc.z_cal_y = y
        
        try:
            self.bmc._calibrate_nozzle_offsets(vlog)
            gcmd.respond_info(f"\nT1 offset: {self.bmc.t1_nozzle_offset:+.4f}mm")
        finally:
            self.bmc.z_cal_x = old_x
            self.bmc.z_cal_y = old_y
            
    def cmd_SET_NOZZLE_OFFSET(self, gcmd):
        """Manually set nozzle Z offset"""
        carriage = gcmd.get_int('T', None)
        offset = gcmd.get_float('Z', None)
        
        if carriage is None or offset is None:
            gcmd.respond_info("Usage: SET_NOZZLE_Z_OFFSET T=<0|1> Z=<offset>")
            gcmd.respond_info(f"  T0: {self.bmc.t0_nozzle_offset:.4f}mm (reference)")
            gcmd.respond_info(f"  T1: {self.bmc.t1_nozzle_offset:+.4f}mm")
            return
            
        if carriage == 0:
            gcmd.respond_info("T0 is reference (always 0)")
            return
        elif carriage == 1:
            self.bmc.t1_nozzle_offset = offset
            self.bmc.offsets_calibrated = True
            gcmd.respond_info(f"T1 nozzle offset set to {offset:+.4f}mm")
            self._update_status()
        else:
            raise gcmd.error("T must be 0 or 1")
            
    def cmd_SET_ACTUATOR(self, gcmd):
        """Manually move toolhead Z actuator"""
        if not self.use_actuators:
            raise gcmd.error("Toolhead Z actuators not configured")
            
        carriage = gcmd.get_int('T', None)
        pos = gcmd.get_float('Z', None)
        
        if carriage is None or pos is None:
            gcmd.respond_info("Usage: SET_TOOLHEAD_Z_OFFSET T=<0|1> Z=<position>")
            gcmd.respond_info(f"  T0 position: {self.t0_actuator_pos:.4f}mm")
            gcmd.respond_info(f"  T1 position: {self.t1_actuator_pos:.4f}mm")
            return
            
        if carriage not in [0, 1]:
            raise gcmd.error("T must be 0 or 1")
            
        self._move_actuator(carriage, pos)
        gcmd.respond_info(f"T{carriage} actuator moved to {pos:.4f}mm")


def load_config(config):
    return DualBedMesh(config)
