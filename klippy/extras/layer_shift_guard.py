# Layer shift detection and recovery helper (BFPv2)
#
# Polls TMC driver registers for evidence of skipped steps / stalls.
# On trigger, runs a recovery macro to pause, re-home XY, and resume.

import logging


class LayerShiftGuard:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()

        self.enabled = config.getboolean('enabled', True)
        self.driver_type = config.get('driver_type', 'tmc5160').strip()
        self.stepper_names = [s.strip() for s in config.getlist('steppers')]
        self.poll_interval = config.getfloat(
            'poll_interval', 0.20, above=0.02)
        # Minimum time (seconds) after a print starts before detection can
        # trigger. This avoids false positives during startup sequencing.
        self.arm_delay = config.getfloat('arm_delay', 2.0, minval=0.0)
        self.lost_steps_threshold = config.getint(
            'lost_steps_threshold', 1, minval=1)
        self.trigger_on_stallguard = config.getboolean(
            'trigger_on_stallguard', True)

        self.z_hop = config.getfloat('z_hop', 10.0, minval=0.0)
        self.z_hop_speed = config.getfloat('z_hop_speed', 10.0, above=0.0)
        self.pause_park = config.getint('pause_park', 0, minval=0, maxval=1)

        self._drivers = []
        self._last_lost = {}
        self._cooldown_until = 0.0
        self._recoveries = 0
        self._last_trigger = None

        self._was_printing = False
        self._armed_after = 0.0

        self._timer = self.reactor.register_timer(self._poll, self.reactor.NOW)
        self.printer.register_event_handler('klippy:connect', self._handle_connect)

        self.gcode.register_command(
            'LAYER_SHIFT_GUARD_STATUS', self.cmd_STATUS,
            desc='Report layer_shift_guard state')
        self.gcode.register_command(
            'LAYER_SHIFT_GUARD_ENABLE', self.cmd_ENABLE,
            desc='Enable/disable layer_shift_guard polling (ENABLE=0/1)')
        self.gcode.register_command(
            'LAYER_SHIFT_GUARD_RESET', self.cmd_RESET,
            desc='Reset layer_shift_guard counters/baselines')
        self.gcode.register_command(
            'LAYER_SHIFT_GUARD_TRIGGER', self.cmd_TRIGGER,
            desc='Debug: force a layer_shift_guard trigger (ARM=1)')

    def _handle_connect(self):
        missing = []
        drivers = []
        for stepper in self.stepper_names:
            obj_name = f'{self.driver_type} {stepper}'
            drv = self.printer.lookup_object(obj_name, None)
            if drv is None:
                missing.append(obj_name)
                continue
            drivers.append((stepper, drv))

        self._drivers = drivers
        for stepper, drv in self._drivers:
            try:
                self._last_lost[stepper] = drv.mcu_tmc.get_register('LOST_STEPS')
            except Exception:
                self._last_lost[stepper] = None

        if missing:
            self.gcode.respond_info(
                'layer_shift_guard: missing driver objects: %s' % (', '.join(missing),)
            )

    def _is_printing(self, eventtime):
        v_sd = self.printer.lookup_object('virtual_sdcard', None)
        if v_sd is None or not v_sd.is_active():
            return False
        ps = self.printer.lookup_object('print_stats', None)
        if ps is None:
            return True
        return ps.get_status(eventtime).get('state') == 'printing'

    def _is_paused(self, eventtime):
        pr = self.printer.lookup_object('pause_resume', None)
        if pr is None:
            return False
        status = pr.get_status(eventtime)
        return bool(status.get('is_paused', False))

    def _homed_axes(self, eventtime):
        toolhead = self.printer.lookup_object('toolhead', None)
        if toolhead is None:
            return ''
        return toolhead.get_status(eventtime).get('homed_axes', '') or ''

    def _poll(self, eventtime):
        try:
            return self._poll_inner(eventtime)
        except Exception:
            logging.exception('layer_shift_guard poll failed')
            # Keep polling; errors will show in klippy.log.
            return eventtime + self.poll_interval

    def _poll_inner(self, eventtime):
        if not self.enabled or not self._drivers:
            return eventtime + self.poll_interval

        if eventtime < self._cooldown_until:
            return eventtime + self.poll_interval

        printing = self._is_printing(eventtime) and not self._is_paused(eventtime)
        if not printing:
            # Reset baselines whenever we're not actively printing.
            self._was_printing = False
            self._armed_after = 0.0
            for stepper, drv in self._drivers:
                try:
                    self._last_lost[stepper] = drv.mcu_tmc.get_register('LOST_STEPS')
                except Exception:
                    self._last_lost[stepper] = None
            return eventtime + self.poll_interval

        if not self._was_printing:
            # Rising edge into printing: arm after a short delay.
            self._was_printing = True
            self._armed_after = eventtime + self.arm_delay
            for stepper, drv in self._drivers:
                try:
                    self._last_lost[stepper] = drv.mcu_tmc.get_register('LOST_STEPS')
                except Exception:
                    self._last_lost[stepper] = None
            return eventtime + self.poll_interval

        if eventtime < self._armed_after:
            # Still within arming delay: keep updating baselines only.
            for stepper, drv in self._drivers:
                try:
                    self._last_lost[stepper] = drv.mcu_tmc.get_register('LOST_STEPS')
                except Exception:
                    self._last_lost[stepper] = None
            return eventtime + self.poll_interval

        homed = self._homed_axes(eventtime)
        if 'x' not in homed or 'y' not in homed:
            # Never attempt recovery moves unless XY are homed.
            for stepper, drv in self._drivers:
                try:
                    self._last_lost[stepper] = drv.mcu_tmc.get_register('LOST_STEPS')
                except Exception:
                    self._last_lost[stepper] = None
            return eventtime + self.poll_interval

        for stepper, drv in self._drivers:
            lost = None
            stall = 0
            try:
                lost = drv.mcu_tmc.get_register('LOST_STEPS')
            except Exception:
                pass
            if self.trigger_on_stallguard:
                try:
                    drv_status = drv.mcu_tmc.get_register('DRV_STATUS')
                    stall = drv.fields.get_field('stallguard', drv_status)
                except Exception:
                    stall = 0

            last = self._last_lost.get(stepper)
            delta = None
            if lost is not None and last is not None:
                try:
                    delta = int(lost) - int(last)
                except Exception:
                    delta = None

            if stall or (delta is not None and delta >= self.lost_steps_threshold):
                # Final guard: only recover if we're still actively printing.
                self._trigger(
                    eventtime,
                    stepper,
                    lost,
                    delta,
                    stall,
                    run_recover=printing and ('x' in homed and 'y' in homed),
                )
                return eventtime + self.poll_interval

            self._last_lost[stepper] = lost

        return eventtime + self.poll_interval

    def _trigger(self, eventtime, stepper, lost, delta, stall, *, run_recover=True):
        self._recoveries += 1
        self._last_trigger = {
            'time': eventtime,
            'stepper': stepper,
            'lost_steps': lost,
            'delta': delta,
            'stallguard': int(bool(stall)),
        }
        self._cooldown_until = eventtime + 10.0

        msg = (
            'layer_shift_guard: trigger on %s (stallguard=%s lost_steps=%s delta=%s)'
            % (stepper, int(bool(stall)), lost, delta)
        )
        logging.warning(msg)
        self.gcode.respond_info(msg)

        if run_recover:
            # Delegate motion/state handling to the user macro so it can be
            # customized without editing python.
            # Default macro is provided in bfp_layer_shift_guard.cfg.
            script = (
                'LAYER_SHIFT_RECOVER Z_HOP=%.3f Z_HOP_SPEED=%.3f\n'
                % (self.z_hop, self.z_hop_speed)
            )
            try:
                self.gcode.run_script(script)
            except Exception:
                # Never allow recovery failures to cascade into print aborts.
                logging.exception('layer_shift_guard: recovery macro failed')
                self.gcode.respond_info('layer_shift_guard: recovery failed (ignored)')
        else:
            self.gcode.respond_info('layer_shift_guard: recovery suppressed (not safe)')

        # Refresh baselines to avoid repeated triggers from monotonic counters.
        for s, drv in self._drivers:
            try:
                self._last_lost[s] = drv.mcu_tmc.get_register('LOST_STEPS')
            except Exception:
                self._last_lost[s] = None

    def cmd_STATUS(self, gcmd):
        driver_names = [s for s, _ in self._drivers]
        gcmd.respond_info(
            'layer_shift_guard: enabled=%s drivers=%d recoveries=%d cooldown=%.1fs steppers=%s'
            % (
                int(bool(self.enabled)),
                len(self._drivers),
                self._recoveries,
                max(0.0, self._cooldown_until - self.reactor.monotonic()),
                driver_names,
            )
        )
        if self._last_trigger is not None:
            gcmd.respond_info('layer_shift_guard: last_trigger=%s' % (self._last_trigger,))

    def cmd_ENABLE(self, gcmd):
        self.enabled = bool(gcmd.get_int('ENABLE', int(self.enabled)))
        gcmd.respond_info('layer_shift_guard: enabled=%d' % (int(self.enabled),))

    def cmd_RESET(self, gcmd):
        self._recoveries = 0
        self._last_trigger = None
        self._cooldown_until = 0.0
        for stepper, drv in self._drivers:
            try:
                self._last_lost[stepper] = drv.mcu_tmc.get_register('LOST_STEPS')
            except Exception:
                self._last_lost[stepper] = None
        gcmd.respond_info('layer_shift_guard: reset')

    def cmd_TRIGGER(self, gcmd):
        if gcmd.get_int('ARM', 0) != 1:
            raise self.printer.command_error('layer_shift_guard: requires ARM=1')

        stepper = gcmd.get('STEPPER', None)
        if stepper is None:
            if not self._drivers:
                raise self.printer.command_error('layer_shift_guard: no drivers configured')
            stepper = self._drivers[0][0]

        # Allow a dry-run trigger without running the recovery macro.
        run_recover = bool(gcmd.get_int('RECOVER', 1))

        lost = gcmd.get_int('LOST_STEPS', None)
        delta = gcmd.get_int('DELTA', None)
        stall = gcmd.get_int('STALLGUARD', 1)

        self._trigger(
            self.reactor.monotonic(),
            stepper,
            lost,
            delta,
            stall,
            run_recover=run_recover,
        )


def load_config(config):
    return LayerShiftGuard(config)

