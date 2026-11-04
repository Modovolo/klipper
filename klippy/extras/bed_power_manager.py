# bed_power_manager.py
# Multi-zone bed manager with:
#  - Warm-up: two-bed rotation; only power zones that need heat
#  - Maintenance: single-bed rotation with anti-blip hysteresis
#  - Protection: hot zones are protected during warm-up, and will be "rescued"
#    first if they drift and need heat again.
#
# Required heaters:
#   [heater_generic heater_bed_FL], _FR, _BL, _BR
#
# printer.cfg:
#   [bed_power_manager]
#     # --- Timing and window controls ---
#     window: 5.0            # base dwell window (s) per rotation cycle
#     jitter: 0.5            # randomize window slightly to avoid sync
#     max_on: 2              # how many heaters can be ON at once (warm-up phase)
#
#     # --- Thermal logic ---
#     wait_tolerance: 2.0    # ±°C considered "at temp" for M190
#     need_heat_margin: 0.30 # °C above which a zone counts as "needs heat"
#     preempt_margin: 0.60   # °C gap required to pre-empt another zone mid-window
#
#     # --- Maintenance (single-bed rotation) ---
#     maint_min_on: 1.0      # min seconds ON for a zone once turned on
#     maint_min_off: 1.5     # min seconds OFF before a zone may be turned on again
#     maint_hi: 0.05         # °C overshoot threshold to turn OFF (anti-blip)
#     maint_lo: 0.10         # °C undershoot threshold to turn ON
#     maint_enter_tol: 1.0   # enter single-bed mode when all within ±this °C
#     maint_exit_tol: 2.0    # leave single-bed mode if any drift exceeds this °C
#     maint_stable_s: 1.0    # all-within dwell time before entering single mode
#
#     # --- Protection during warm-up ---
#     protect_tol: 1.0       # zone is "protected" if |target - temp| <= this °C
#     protect_gain: 0.20     # small boost to non-protected (colder) zones
#     protect_rescue: True   # if a protected zone starts needing heat, force it ON first
#
from __future__ import annotations
import time, random

ZMAP = ["FL", "FR", "BL", "BR"]
HEATER_SECTIONS = [
    "heater_generic heater_bed_FL",
    "heater_generic heater_bed_FR",
    "heater_generic heater_bed_BL",
    "heater_generic heater_bed_BR",
]

class Zone:
    def __init__(self, name, heater, log):
        self.name = name
        self.heater = heater
        self.user_target = 0.0
        self.htgt = 0.0               # currently applied heater target (0 or user_target)
        self.last_on = 0.0
        self.last_off = 0.0
        self.log = log

    def cur_temp(self):
        try:
            st = self.heater.get_status(time.time())
            return float(st.get("temperature", 0.0))
        except Exception:
            return 0.0

    def apply(self, want_on: bool):
        tgt = self.user_target if want_on and self.user_target > 0 else 0.0
        if tgt != self.htgt:
            try:
                self.heater.set_temp(float(tgt))
                self.htgt = tgt
                now = time.time()
                if tgt > 0: self.last_on = now
                else:       self.last_off = now
            except Exception as e:
                self.log("BPM WARN: write %s->%s failed: %s" % (self.name, tgt, e))

class BedPowerManager:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        R = self.printer.get_reactor()

        # --- Tunables ---
        g = config.get
        self.window = float(g("window", 5.0))
        self.jitter = float(g("jitter", 0.5))
        self.max_on = int(g("max_on", 2))
        self.need_heat_margin = float(g("need_heat_margin", 0.30))
        self.preempt_margin = float(g("preempt_margin", 0.60))

        self.wait_tol = float(g("wait_tolerance", 2.0))

        self.maint_min_on  = float(g("maint_min_on", 1.0))
        self.maint_min_off = float(g("maint_min_off", 1.5))
        self.maint_hi      = float(g("maint_hi", 0.05))
        self.maint_lo      = float(g("maint_lo", 0.10))
        self.maint_enter_tol = float(g("maint_enter_tol", 1.0))
        self.maint_exit_tol  = float(g("maint_exit_tol", 2.0))
        self.maint_stable_s  = float(g("maint_stable_s", 1.0))

        self.protect_tol    = float(g("protect_tol", 1.0))
        self.protect_gain   = float(g("protect_gain", 0.20))
        self.protect_rescue = config.getboolean("protect_rescue", True)

        # Resolve heaters
        self.z = []
        for zname, sec in zip(ZMAP, HEATER_SECTIONS):
            try:
                heater = self.printer.lookup_object(sec)
            except Exception:
                heater = None
            self.z.append(Zone(zname, heater, self._log))

        # State
        self.enabled = True
        self.mode = "two"     # "two" (two-bed rotation) or "single" (single-bed rotation)
        self.protected = set()   # zones protected (while in warm-up)
        self.reactor = R
        self.timer = R.register_timer(self._on_tick)
        self.last_switch = R.monotonic()
        self.slot_len = self.window
        self.powered = set()     # indices currently granted power in warm-up
        self.single_idx = 0      # index for single-bed round-robin
        self.single_start = 0.0
        self.stable_since = 0.0

        # Commands
        self.gcode.register_command("BPM_ON", self.cmd_BPM_ON)
        self.gcode.register_command("BPM_OFF", self.cmd_BPM_OFF)
        self.gcode.register_command("BPM_OFF_ALL", self.cmd_BPM_OFF_ALL)
        self.gcode.register_command("BPM_STATUS", self.cmd_BPM_STATUS)
        self.gcode.register_command("BPM_SET", self.cmd_BPM_SET)
        self.gcode.register_command("M140", self.cmd_M140)
        self.gcode.register_command("M190", self.cmd_M190)

        R.update_timer(self.timer, R.monotonic() + 0.25)

    # ---- helpers ----
    def _log(self, msg): self.gcode.respond_info(msg)
    def _now(self): return self.reactor.monotonic()

    def _active_idxs(self):
        return [i for i, z in enumerate(self.z) if z.user_target > 0.0 and z.heater is not None]

    def _deficit(self, i):
        z = self.z[i]
        return float(z.user_target - z.cur_temp())

    def _needs_heat(self, i, margin=None):
        if margin is None: margin = self.need_heat_margin
        return self._deficit(i) > margin

    def _all_within(self, tol):
        act = self._active_idxs()
        if not act: return False
        for i in act:
            if abs(self._deficit(i)) > tol: return False
        return True

    def _set_all_off(self):
        for z in self.z: z.apply(False)

    # ---- Mode control ----
    def _try_enter_single(self, now):
        if self._all_within(self.maint_enter_tol):
            if self.stable_since == 0.0: self.stable_since = now
            if (now - self.stable_since) >= self.maint_stable_s:
                self.mode = "single"
                self.protected.clear()         # clear protection on entry
                act = self._active_idxs()
                if act:
                    self.single_idx = max(act, key=lambda i: self._deficit(i)) % 4
                self.single_start = now
                self._log("BPM: enter single-bed-rotation")
        else:
            self.stable_since = 0.0

    def _maybe_leave_single(self, now):
        if not self._all_within(self.maint_exit_tol):
            self.mode = "two"
            self.last_switch = now - self.window
            self.slot_len = self.window
            self._log("BPM: leave single-bed-rotation -> two-bed-rotation")

    # ---- Warm-up (two-bed) selection ----
    def _select_two(self, now):
        act = self._active_idxs()
        if not act: return set()

        # Mark which zones are "protected" (close to target)
        self.protected = {i for i in act if abs(self._deficit(i)) <= self.protect_tol}

        # Anyone who is protected but is now cooling enough to need heat?
        rescue = set()
        if self.protect_rescue:
            for i in self.protected:
                if self._needs_heat(i, margin=self.maint_lo):
                    rescue.add(i)

        # Candidates that actually need heat (by normal margin)
        needs = [i for i in act if self._needs_heat(i)]

        # If we have rescue(s), they take precedence
        if rescue:
            ranked_rescue = sorted(rescue, key=lambda i: self._deficit(i), reverse=True)
            if len(ranked_rescue) >= self.max_on:
                return set(ranked_rescue[:self.max_on])
            sel = list(ranked_rescue)

            # Fill remaining slots from other "needs", preferring colder, non-protected zones first
            remaining = [i for i in needs if i not in rescue]
            remaining_sorted = sorted(
                remaining,
                key=lambda i: (self._deficit(i) + (self.protect_gain if i not in self.protected else 0.0)),
                reverse=True
            )
            for i in remaining_sorted:
                if len(sel) >= self.max_on: break
                sel.append(i)
            return set(sel)

        # No rescue pressure; rank by deficit with a small bias toward non-protected (colder) zones
        ranked = sorted(
            needs,
            key=lambda i: (self._deficit(i) + (self.protect_gain if i not in self.protected else 0.0)),
            reverse=True
        )
        if len(ranked) <= self.max_on:
            best = set(ranked)
        else:
            best = set(ranked[:self.max_on])

        # Respect dwell unless a waiting zone beats the worst powered by preempt_margin
        if self.powered and self.powered.issubset(best) and len(self.powered) == min(self.max_on, len(best)):
            dwell_elapsed = (now - self.last_switch) >= self.slot_len
            if not dwell_elapsed:
                waiting = set(best) - self.powered
                if waiting:
                    max_wait = max(self._deficit(i) for i in waiting)
                    min_pow  = min(self._deficit(i) for i in self.powered)
                    if max_wait <= (min_pow + self.preempt_margin):
                        return set(self.powered)
        return best

    def _drive_two(self, sel):
        for i, z in enumerate(self.z):
            z.apply(i in sel)

    # ---- Single-bed maintenance driver ----
    def _drive_single(self, now):
        act = self._active_idxs()
        if not act:
            self._set_all_off()
            return

        if self.single_idx not in act:
            act_sorted = sorted(act)
            self.single_idx = act_sorted[0]

        i = self.single_idx
        z = self.z[i]
        t = z.cur_temp()
        tgt = z.user_target

        need_on  = (t <= (tgt - self.maint_lo))
        need_off = (t >= (tgt + self.maint_hi))

        since_on  = (time.time() - z.last_on)  if z.last_on  else 1e9
        since_off = (time.time() - z.last_off) if z.last_off else 1e9

        if z.htgt > 0.0:
            if need_off and since_on >= self.maint_min_on:
                z.apply(False)
                # hop to the most needy other active zone (if any)
                nexts = [j for j in act if j != i]
                if nexts:
                    j = max(nexts, key=lambda k: self._deficit(k))
                    self.single_idx = j
                    self.single_start = now
            else:
                z.apply(True)
        else:
            if need_on and since_off >= self.maint_min_off:
                z.apply(True)
            else:
                needers = [j for j in act if self._needs_heat(j, margin=self.maint_lo)]
                if needers:
                    j = max(needers, key=lambda k: self._deficit(k))
                    if j != i:
                        self.single_idx = j
                        self.single_start = now
                for k in act:
                    if k != self.single_idx:
                        self.z[k].apply(False)

        for k in range(4):
            if k != self.single_idx:
                self.z[k].apply(False)

    # ---- Timer ----
    def _on_tick(self, eventtime):
        try:
            if not self.enabled:
                return eventtime + 0.5

            now = self._now()

            if self.mode == "two":
                self._try_enter_single(now)
            else:
                self._maybe_leave_single(now)

            if self.mode == "two":
                sel = self._select_two(now)
                if sel != self.powered:
                    self.powered = sel
                    self.last_switch = now
                    j = random.uniform(-self.jitter, self.jitter) if len(sel) and len(self._active_idxs()) > self.max_on else 0.0
                    self.slot_len = max(0.5, self.window + j)
                self._drive_two(self.powered)
            else:
                self._drive_single(now)

            return eventtime + 0.25
        except Exception as e:
            self._log("BPM ERROR: %s" % e)
            self.enabled = False
            self._set_all_off()
            return eventtime + 1.0

    # ---- GCODE ----
    def cmd_BPM_ON(self, gcmd):
        self.enabled = True
        self._log("BPM: enabled")

    def cmd_BPM_OFF(self, gcmd):
        self.enabled = False
        for z in self.z: z.user_target = 0.0
        self._set_all_off()
        self._log("BPM: disabled (zones OFF)")

    def cmd_BPM_OFF_ALL(self, gcmd):
        for z in self.z:
            z.user_target = 0.0
        self._set_all_off()
        self.enabled = False
        self._log("All bed zones set to 0 and powered OFF")

    def cmd_BPM_STATUS(self, gcmd):
        hdr = ("mode=%s window=%.2fs jitter=%.2fs max_on=%d "
               "single: min_on=%.2fs min_off=%.2fs enter=±%.2f exit=%.2f hi=%.2f lo=%.2f "
               "protect_gain=%.2f protect_tol=%.2f rescue=%s") % (
            self.mode, self.window, self.jitter, self.max_on,
            self.maint_min_on, self.maint_min_off, self.maint_enter_tol, self.maint_exit_tol,
            self.maint_hi, self.maint_lo, self.protect_gain, self.protect_tol, str(self.protect_rescue)
        )
        parts = []
        for i, z in enumerate(self.z):
            tag = "(ON)" if (self.mode == "two" and i in self.powered) or (self.mode == "single" and i == self.single_idx and z.htgt > 0.0) else ""
            prot = " [P]" if (self.mode == "two" and i in self.protected) else ""
            parts.append("%s: cur=%.1f utgt=%.2f %s%s" % (z.name, z.cur_temp(), z.user_target, tag, prot))
        self._log("BPM: %s | %s" % (hdr, "  ".join(parts)))

    def cmd_BPM_SET(self, gcmd):
        zarg = gcmd.get("Z", default="ALL").strip().upper()
        sval = gcmd.get("S", default=None)
        t = float(sval) if sval is not None else 0.0

        if zarg == "ALL":
            for z in self.z:
                z.user_target = t
            self._log("BPM: ALL -> %.2f°C" % t)
        else:
            if zarg not in ZMAP:
                raise gcmd.error("BPM_SET: unknown zone '%s' (use FL|FR|BL|BR|ALL)" % zarg)
            idx = ZMAP.index(zarg)
            self.z[idx].user_target = t
            self._log("BPM: %s -> %.2f°C" % (ZMAP[idx], t))

        self.enabled = True
        self.mode = "two"
        self.protected.clear()
        self.last_switch = self._now() - self.window
        self.slot_len = self.window

    def _apply_M_set(self, t: float, tsel: int, cmd: str):
        if tsel == 0:
            for z in self.z: z.user_target = t
            self._log("BPM: ALL -> %.2f°C" % t)
        else:
            if not (1 <= tsel <= 4):
                raise self.gcode.error("%s: T must be 0..4" % cmd)
            idx = tsel - 1
            self.z[idx].user_target = t
            self._log("BPM: %s -> %.2f°C" % (ZMAP[idx], t))
        self.enabled = True
        self.mode = "two"
        self.protected.clear()
        self.last_switch = self._now() - self.window
        self.slot_len = self.window

    def cmd_M140(self, gcmd):
        hasS = "S" in gcmd.get_command_parameters()
        t = float(gcmd.get("S", default="0" if hasS else "0"))
        tsel = int(gcmd.get("T", default="0"))
        self._apply_M_set(t, tsel, "M140")

    def cmd_M190(self, gcmd):
        if "S" not in gcmd.get_command_parameters():
            return
        t = float(gcmd.get("S"))
        tsel = int(gcmd.get("T", default="0"))
        self._apply_M_set(t, tsel, "M190")

        tol = self.wait_tol
        start = time.time()
        while True:
            act = self._active_idxs()
            ok = True
            for i in act:
                if self.z[i].user_target > 0.0:
                    if abs(self.z[i].cur_temp() - self.z[i].user_target) > tol:
                        ok = False
                        break
            if ok:
                break
            self.reactor.pause(0.25)
            if (time.time() - start) > 36000:
                self._log("BPM WARN: M190 timed out waiting ±%.1f°C; continuing." % tol)
                break

def load_config(config):
    return BedPowerManager(config)
