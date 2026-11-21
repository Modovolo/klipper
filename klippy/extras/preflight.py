# BFP V2 temperature preflight v2.0 -- experimental Klipper extra.
# GPL-3.0-or-later. Install only with the separately supplied hook after tests.
# Original G-code remains untouched. No changes to TAM/homing_override.
import hashlib
import logging
import math
import os
import re
import secrets

VERSION = '2.0.0-beta'
ZONES = ('FL', 'BL', 'FR', 'BR')
FIELDS = ('NOZZLE',) + ZONES
NUM = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)'
CMD = re.compile(r'^\s*([A-Z_][A-Z_0-9]*)\b', re.I)
SPARAM = re.compile(r'(?<!\S)S\s*=?\s*(' + NUM + r')(?=\s|$)', re.I)
TOOL = re.compile(r'(?<!\S)T\s*=?\s*(\d+)(?=\s|$)', re.I)
PARAM = re.compile(r'(?<!\S)([A-Z_][A-Z_0-9]*)\s*=\s*(' + NUM + r')(?=\s|$)', re.I)
EMOVE = re.compile(r'(?<!\S)E\s*=?\s*(' + NUM + r')(?=\s|$)', re.I)
START_MACROS = ('START_PRINT', 'PRINT_START')
NOZZLE_HINTS = ('EXTRUDER', 'EXTRUDER_TEMP', 'NOZZLE', 'NOZZLE_TEMP', 'HOTEND_TEMP')
BED_HINTS = ('BED', 'BED_TEMP', 'BED_TEMPERATURE')
NAMED_HEATER = re.compile(r'(?<!\S)HEATER=([^\s]+)', re.I)
TARGET = re.compile(r'(?<!\S)TARGET=('+NUM+r')(?=\s|$)', re.I)


def instruction(line):
    code = line.decode('utf-8', 'replace') if isinstance(line, bytes) else line
    code = code.split(';', 1)[0].strip()
    match = CMD.match(code)
    return (match.group(1).upper() if match else '', code)


def is_positive_extrusion(cmd, code):
    if cmd not in ('G0', 'G1', 'G00', 'G01'):
        return False
    m = EMOVE.search(code)
    return bool(m and float(m.group(1)) > 0)


def scan_file(filename, tmap, scan_bytes=32 * 1024 * 1024, heater_names=None):
    """Scan instructions before first positive-E move, not layer-change commands.

    Direct BFP M104/M109 Tn are trusted only through configured tmap.
    START_PRINT and M140 are *hints*, always require review if no direct
    primary heater instruction exists. Never silently accept ambient targets.
    """
    direct = {}
    name_map = {name:field for field,name in (heater_names or {}).items()}
    hints = {}
    warnings = []
    first_extrusion = False
    start_macro = False
    scanned = 0
    off_before_start = set()
    with open(filename, 'rb') as fh:
        for raw in fh:
            scanned += len(raw)
            if scanned > scan_bytes:
                raise ValueError('Startup exceeds scan limit; refusing to print')
            cmd, code = instruction(raw)
            if not cmd:
                continue
            if is_positive_extrusion(cmd, code):
                first_extrusion = True
                break
            if cmd in ('M104', 'M109', 'M140', 'M190'):
                sm = SPARAM.search(code)
                if not sm:
                    continue
                temp = float(sm.group(1))
                if cmd in ('M104', 'M109'):
                    tm = TOOL.search(code)
                    t = int(tm.group(1)) if tm else 0
                    target = tmap.get(t)
                    if target:
                        direct[target] = temp
                        if temp <= 0:
                            off_before_start.add(target)
                        else:
                            off_before_start.discard(target)
                else:
                    hints['BED'] = temp  # Standard M140 not a multi-zone BFP command
            elif cmd == 'SET_HEATER_TEMPERATURE':
                nm = NAMED_HEATER.search(code)
                tm = TARGET.search(code)
                if nm and tm and nm.group(1) in name_map:
                    target = name_map[nm.group(1)]
                    temp = float(tm.group(1))
                    direct[target] = temp
                    if temp <= 0:
                        off_before_start.add(target)
                    else:
                        off_before_start.discard(target)
            elif cmd in START_MACROS:
                start_macro = True
                params = {k.upper(): float(v) for k,v in PARAM.findall(code)}
                for key in NOZZLE_HINTS:
                    if key in params:
                        hints['NOZZLE'] = params[key]
                        break
                for key in BED_HINTS:
                    if key in params:
                        hints['BED'] = params[key]
                        break
    if off_before_start:
        warnings.append('Explicit heater-off command before extrusion: ' + ','.join(sorted(off_before_start)))
    if not first_extrusion:
        raise ValueError('No positive extrusion found within startup scan; cannot locate safety wait')
    result = {}
    source = {}
    for key in FIELDS:
        if key in direct and direct[key] > 0:
            result[key], source[key] = direct[key], 'direct'
        elif key == 'NOZZLE' and hints.get('NOZZLE', 0) > 0:
            result[key], source[key] = hints['NOZZLE'], 'hint'
        elif key in ('FL', 'BL') and hints.get('BED', -1) >= 0:
            result[key], source[key] = hints['BED'], 'hint'
        elif key in ('FR', 'BR'):
            result[key], source[key] = 0.0, 'off-default'
        else:
            result[key], source[key] = None, 'missing'
    # Detect later explicit off (not confused with zero for unused secondary zones).
    for key in ('NOZZLE', 'FL', 'BL'):
        if key in off_before_start:
            raise ValueError('File turns off %s before first extrusion; correct G-code' % key)
    auto = all(source[key] == 'direct' for key in ('NOZZLE','FL','BL'))
    if source['NOZZLE'] == 'hint' or source['FL'] == 'hint':
        warnings.append('Macro parameters are hints, not proven heater commands')
    return {'values':result, 'sources':source, 'auto':auto,
            'has_start_macro':start_macro, 'warnings':warnings}


def sha256(filename):
    h = hashlib.sha256()
    with open(filename, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b''):
            h.update(chunk)
    return h.digest()


def fmt(x):
    return '%g' % x


def stage_gcode(source, dest, session, heater_names, values):
    """Make an executable copy: preheat before all G-code; wait after START_PRINT
    (or immediately before first positive E if there is no START_PRINT).
    """
    preheat = ['; PREFLIGHT GENERATED FILE - source unchanged',
               'PREFLIGHT_ASSERT ID=' + session,
               'BFP_PREFLIGHT_ASSERT ID=' + session]
    for field, name in heater_names.items():
        target = values[field]
        # Only touch zones intentionally selected, but turn idle right beds off
        # at the start; later COPY/MIRROR macros may turn them on.
        preheat.append('SET_HEATER_TEMPERATURE HEATER=%s TARGET=%s' % (name, fmt(target)))
    preheat.append('; PREFLIGHT: run startup normally while heaters warm')
    wait = ['; PREFLIGHT: safety wait, after homing/QGL/mesh when in START_PRINT',
            'PREFLIGHT_WAIT ID=' + session,
            'BFP_PREFLIGHT_WAIT ID=' + session]
    inserted_heat = inserted_wait = False
    saw_macro = False
    with open(source, 'rb') as src, open(dest, 'wb') as output:
        for raw in src:
            cmd, code = instruction(raw)
            if cmd and not inserted_heat:
                output.write(('\n'.join(preheat) + '\n').encode('utf-8'))
                inserted_heat = True
            if is_positive_extrusion(cmd, code) and not inserted_wait:
                output.write(('\n'.join(wait) + '\n').encode('utf-8'))
                inserted_wait = True
            output.write(raw)
            if cmd in START_MACROS and not inserted_wait:
                output.write(('\n' + '\n'.join(wait) + '\n').encode('utf-8'))
                inserted_wait = True
                saw_macro = True
    if not inserted_heat or not inserted_wait:
        raise ValueError('Cannot stage file without startup and extrusion guard')
    return {'wait_after_start': saw_macro}


class Preflight:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.sd = None
        self.field_to_heater = {
            'NOZZLE': config.get('nozzle_heater', 'extruder'),
            'FL': config.get('heater_fl', 'heater_bed_FL'),
            'BL': config.get('heater_bl', 'heater_bed_BL'),
            'FR': config.get('heater_fr', 'heater_bed_FR'),
            'BR': config.get('heater_br', 'heater_bed_BR'),
        }
        # Existing BFP Gcode from user confirms T0/T2/T4; T3/T5 MUST be
        # verified on live config before trusting right-zone commands.
        t_values = [config.getint('tool_nozzle', 0, minval=0),
                    config.getint('tool_fl', 2, minval=0),
                    config.getint('tool_bl', 4, minval=0),
                    config.getint('tool_fr', 3, minval=0),
                    config.getint('tool_br', 5, minval=0)]
        if len(set(t_values)) != len(t_values):
            raise config.error('BFP preflight: tool indexes must be distinct')
        self.tmap = dict(zip(t_values, FIELDS))
        self.nozzle_min = config.getfloat('min_nozzle', 150., minval=1.)
        self.nozzle_max = config.getfloat('max_nozzle', 360., above=self.nozzle_min)
        self.bed_max = config.getfloat('max_bed', 130., minval=1.)
        self.scan_bytes = config.getint('scan_bytes', 33554432, minval=4096)
        self.wait_tolerance = config.getfloat('wait_tolerance', 4., minval=0., maxval=20.)
        self.wait_timeout = config.getint('wait_timeout_seconds', 1200, minval=60)
        self.max_preheat_beds = config.getint('max_preheat_beds', 2, minval=1, maxval=4)
        self.pending = None
        self.approved = None
        self.hook_checked = False
        self._temp_heaters = {}
        self.printer.register_event_handler('klippy:ready', self._ready)
        for name, handler in (
            # Preferred generic command names
            ('PREFLIGHT_BEGIN', self.begin),
            ('PREFLIGHT_TEST', self.test),
            ('PREFLIGHT_SET', self.set_values),
            ('PREFLIGHT_APPROVE', self.approve),
            ('PREFLIGHT_CANCEL', self.cancel),
            ('PREFLIGHT_STATUS', self.status),
            ('PREFLIGHT_WAIT', self.wait),
            ('PREFLIGHT_ASSERT', self.assert_stage),
            ('PREFLIGHT_EDIT', self.edit),
            ('PREFLIGHT_KEY', self.key),
            ('PREFLIGHT_CLEAR', self.clear),
            ('PREFLIGHT_DEL', self.delete),
            ('PREFLIGHT_DONE', self.done),
            # Backwards-compatible command names
            ('BFP_PREFLIGHT_BEGIN', self.begin),
            ('BFP_PREFLIGHT_TEST', self.test),
            ('BFP_PREFLIGHT_SET', self.set_values),
            ('BFP_PREFLIGHT_APPROVE', self.approve),
            ('BFP_PREFLIGHT_CANCEL', self.cancel),
            ('BFP_PREFLIGHT_STATUS', self.status),
            ('BFP_PREFLIGHT_WAIT', self.wait),
            ('BFP_PREFLIGHT_ASSERT', self.assert_stage),
            ('BFP_PREFLIGHT_EDIT', self.edit),
            ('BFP_PREFLIGHT_KEY', self.key),
            ('BFP_PREFLIGHT_CLEAR', self.clear),
            ('BFP_PREFLIGHT_DEL', self.delete),
            ('BFP_PREFLIGHT_DONE', self.done),
        ):
            self.gcode.register_command(name, handler, desc='Temperature preflight')

    def _ready(self):
        self.sd = self.printer.lookup_object('virtual_sdcard', None)
        if self.sd is None:
            logging.error('BFP PREFLIGHT requires [virtual_sdcard]')
            return
        heaters = self.printer.lookup_object('heaters', None)
        if heaters is None:
            logging.error('BFP PREFLIGHT cannot find heaters')
            return
        try:
            self._temp_heaters = {field: heaters.lookup_heater(name)
                                  for field,name in self.field_to_heater.items()}
        except Exception:
            logging.exception('BFP PREFLIGHT heater mapping invalid; printing will be blocked')
            self._temp_heaters = {}

    def _require_mapped(self, gcmd):
        if len(self._temp_heaters) != 5 or not self.sd:
            raise gcmd.error('BFP PREFLIGHT: five heater names or virtual_sdcard missing; check printer.cfg')

    def _file(self, filename):
        if not self.sd:
            raise self.gcode.error('BFP preflight requires [virtual_sdcard]')
        root = os.path.realpath(self.sd.sdcard_dirname)
        filename = filename.lstrip('/')
        if '\r' in filename or '\n' in filename or not filename:
            raise self.gcode.error('BFP preflight: invalid filename')
        path = os.path.realpath(os.path.join(root, filename))
        if os.path.commonpath((root,path)) != root or not path.lower().endswith(('.gcode','.g','.gco')):
            raise self.gcode.error('BFP preflight: G-code must be inside SD root')
        if os.path.basename(filename).startswith('pf_') or filename.startswith(('_preflight_stage/', '.preflight_stage/')):
            raise self.gcode.error('BFP preflight: generated files cannot be started directly: ' + filename)
        if not os.path.isfile(path):
            files = [f for f, _ in self.sd.get_file_list(check_subdirs=True) if f.lower() == filename.lower()]
            if len(files) != 1:
                raise self.gcode.error('BFP preflight: file not found')
            filename = files[0]
            path = os.path.realpath(os.path.join(root, filename))
            if os.path.commonpath((root,path)) != root:
                raise self.gcode.error('BFP preflight: invalid path')
        return filename, path

    def _validate(self, field, val, gcmd):
        if val is None or not math.isfinite(val):
            raise gcmd.error('BFP preflight: %s must be a finite number' % field)
        minimum = self.nozzle_min if field == 'NOZZLE' else (0 if field in ('FR','BR') else 1)
        maximum = self.nozzle_max if field == 'NOZZLE' else self.bed_max
        if not minimum <= val <= maximum:
            raise gcmd.error('BFP preflight: %s must be %g..%g C' % (field,minimum,maximum))
        # Reject an operator target beyond the physical heater's limit.
        heater = self._temp_heaters[field]
        if val != 0 and not heater.min_temp <= val <= heater.max_temp:
            raise gcmd.error('BFP preflight: %s exceeds configured heater range' % field)

    def _session(self, gcmd):
        if not self.pending:
            raise gcmd.error('BFP preflight: no pending prompt')
        if gcmd.get('ID') != self.pending['id']:
            raise gcmd.error('BFP preflight: stale/incorrect prompt ID')
        return self.pending

    def _prompt(self):
        p = self.pending
        if not p: return
        v = p['values']
        label = os.path.basename(p['filename'])[:80].replace('|','_')
        lines = [label, 'Nozzle=%s  FL=%s  BL=%s  FR=%s  BR=%s' %
                 tuple(('missing' if v[x] is None else fmt(v[x])) for x in FIELDS),
                 'Edit each temperature; 0C = off (right beds only).']
        self.gcode.respond_raw('// action:prompt_end')
        self.gcode.respond_raw('// action:prompt_begin BFP V2 - Temperature Preflight')
        for line in lines:
            self.gcode.respond_raw('// action:prompt_text '+line)
        for field in FIELDS:
            self.gcode.respond_raw('// action:prompt_button Set %s|PREFLIGHT_EDIT ID=%s FIELD=%s|warning' % (field,p['id'],field))
        if all(v[x] is not None for x in FIELDS):
            self.gcode.respond_raw('// action:prompt_button_group_start')
            self.gcode.respond_raw('// action:prompt_button Heat and Print|PREFLIGHT_APPROVE ID=%s|primary' % p['id'])
            self.gcode.respond_raw('// action:prompt_button Cancel Print|PREFLIGHT_CANCEL ID=%s|error' % p['id'])
            self.gcode.respond_raw('// action:prompt_button_group_end')
        else:
            self.gcode.respond_raw('// action:prompt_button Cancel Print|PREFLIGHT_CANCEL ID=%s|error' % p['id'])
        self.gcode.respond_raw('// action:prompt_show')
        # KlipperScreen supports 'ks_show' panel actions in recent releases.
        # The custom GTK panel uses real editable Gtk.Entry fields; the built-in
        # prompt remains a 0..9 keypad fallback for Mainsail and older clients.
        meta = {'id':p['id'], 'name': label, 'values': v,
                'mins': {'NOZZLE':self.nozzle_min,'FL':1,'BL':1,'FR':0,'BR':0},
                'maxs': {'NOZZLE':self.nozzle_max,'FL':self.bed_max,
                         'BL':self.bed_max,'FR':self.bed_max,'BR':self.bed_max}}
        self.gcode.respond_raw('// action:ks_show preflight extra=' + repr(meta))

    def _keypad(self):
        p = self.pending
        field = p['editing']; digits = p['digits']
        self.gcode.respond_raw('// action:prompt_end')
        self.gcode.respond_raw('// action:prompt_begin Set BFP '+field)
        self.gcode.respond_raw('// action:prompt_text Enter Celsius: '+(digits or '---'))
        for row in ('123','456','789','0'):
            self.gcode.respond_raw('// action:prompt_button_group_start')
            for digit in row:
                self.gcode.respond_raw('// action:prompt_button %s|PREFLIGHT_KEY ID=%s KEY=%s|info' % (digit,p['id'],digit))
            self.gcode.respond_raw('// action:prompt_button_group_end')
        self.gcode.respond_raw('// action:prompt_button Backspace|PREFLIGHT_DEL ID=%s|secondary' % p['id'])
        self.gcode.respond_raw('// action:prompt_button Clear|PREFLIGHT_CLEAR ID=%s|secondary' % p['id'])
        self.gcode.respond_raw('// action:prompt_button Use Value|PREFLIGHT_DONE ID=%s|primary' % p['id'])
        self.gcode.respond_raw('// action:prompt_show')

    def begin(self, gcmd):
        self._begin(gcmd, False)

    def test(self, gcmd):
        self._begin(gcmd, True)

    def _begin(self,gcmd,dry_run):
        self._require_mapped(gcmd)
        logging.info('BFP PREFLIGHT BEGIN filename=%r dry_run=%s', gcmd.get('FILENAME', None), dry_run)
        if self.pending or (self.sd and (self.sd.is_active() or self.sd.current_file is not None)):
            raise gcmd.error('BFP preflight: printer busy or prompt pending')
        name,path = self._file(gcmd.get('FILENAME'))

        # Allow motion-only / diagnostic G-code files to opt out of preflight.
        # This is useful for printer kinematics/stall testing where there may be
        # no extrusion (and therefore no safe place to inject the preflight wait).
        #
        # Marker examples (case-insensitive, in the first ~64KiB of file):
        #   ; PREFLIGHT_DISABLE
        #   ; BFP_PREFLIGHT_DISABLE
        #   ; BFP PREFLIGHT: DISABLE
        #
        # When present, we start the original file directly (no staging).
        try:
            if self._file_disables_preflight(path):
                if dry_run:
                    self.gcode.respond_info('BFP preflight: disabled by file directive (dry run)')
                    return
                self.gcode.respond_info('BFP preflight: disabled by file directive; starting unmodified file')
                # Direct launch avoids re-entering the SDCARD_PRINT_FILE hook.
                sd_line = 'SDCARD_PRINT_FILE FILENAME=' + name
                sd_gcmd = self.gcode.create_gcode_command(
                    'SDCARD_PRINT_FILE',
                    sd_line,
                    {'FILENAME': name}
                )
                self.sd.cmd_SDCARD_PRINT_FILE(sd_gcmd)
                return
        except Exception:
            logging.exception('BFP PREFLIGHT disable-directive scan failed')
            # Fall through to normal preflight.
        try:
            parsed = scan_file(path,self.tmap,self.scan_bytes,self.field_to_heater)
            digest = sha256(path)
        except (OSError, ValueError) as exc:
            raise gcmd.error('BFP preflight: %s' % exc)
        for field,val in parsed['values'].items():
            if val is not None:
                self._validate(field,val,gcmd)
        self.approved = None
        self.pending = {'id': secrets.token_hex(6), 'filename':name,'path':path,
                        'digest':digest, 'values':dict(parsed['values']),
                        'sources':parsed['sources'], 'warnings':parsed['warnings'],
                        'editing':None, 'digits':'', 'dry_run':dry_run}
        if parsed['auto'] and not dry_run:
            self.gcode.respond_info('BFP preflight: verified source temperatures, starting')
            self._execute(gcmd)
        else:
            self.gcode.respond_info('BFP preflight: file HELD before any G-code; approval required')
            self._prompt()

    def _file_disables_preflight(self, path):
        # Keep this scan small and fast; marker is expected in the header.
        # Use bytes matching to avoid encoding surprises.
        max_bytes = min(int(self.scan_bytes), 65536)
        with open(path, 'rb') as fh:
            data = fh.read(max_bytes).lower()
        return (
            # Preferred generic markers
            b'preflight_disable' in data
            or b'preflight: disable' in data
            or b'no preflight' in data
            # Backwards-compatible markers
            or b'bfp_preflight_disable' in data
            or b'bfp preflight: disable' in data
            or b'bfp no preflight' in data
        )

    def set_values(self,gcmd):
        p = self._session(gcmd)
        updates = {}
        for field in FIELDS:
            raw = gcmd.get(field, None)
            if raw is not None:
                try:
                    v = float(raw)
                except (ValueError,OverflowError):
                    raise gcmd.error('BFP preflight: %s must be numeric' % field)
                self._validate(field,v,gcmd)
                updates[field] = v
        if not updates:
            raise gcmd.error('BFP preflight: no values provided')
        p['values'].update(updates)
        for k in updates:
            p['sources'][k] = 'operator'
        self.gcode.respond_info('BFP preflight: temperature fields updated')

    def approve(self,gcmd):
        self._session(gcmd)
        self._execute(gcmd)

    def _execute(self,gcmd):
        p = self.pending
        if not p:
            raise gcmd.error('BFP preflight: no pending print')
        self._require_mapped(gcmd)
        for key in FIELDS:
            self._validate(key, p['values'][key],gcmd)
        active_beds = sum(p['values'][key] > 0 for key in ZONES)
        if active_beds > self.max_preheat_beds:
            raise gcmd.error('BFP preflight: %d bed zones requested, but max_preheat_beds=%d; verify 30A heater power budget and change config' % (active_beds,self.max_preheat_beds))
        if sha256(p['path']) != p['digest']:
            self.pending = None
            raise gcmd.error('BFP preflight: file changed since check')
        if p['dry_run']:
            self.pending = None
            self.gcode.respond_raw('// action:prompt_end')
            gcmd.respond_info('BFP preflight dry run PASSED: no heat or movement')
            return
        root = os.path.realpath(self.sd.sdcard_dirname)
        subdir = os.path.join(root,'.preflight_stage')
        os.makedirs(subdir, exist_ok=True)
        stage_name = 'pf_' + p['id'] + '.gcode'
        stage_full = os.path.join(subdir,stage_name)
        stage_rel = '.preflight_stage/' + stage_name
        try:
            stage_gcode(p['path'],stage_full,p['id'],self.field_to_heater,p['values'])
            self.approved = {'id':p['id'], 'path':os.path.realpath(stage_full),
                             'original':p['filename'], 'values':p['values']}
            self.pending = None
            self.gcode.respond_raw('// action:prompt_end')
            # Direct launch avoids re-entering the SDCARD_PRINT_FILE hook.
            sd_line = 'SDCARD_PRINT_FILE FILENAME=' + stage_rel
            sd_gcmd = self.gcode.create_gcode_command(
                'SDCARD_PRINT_FILE',
                sd_line,
                {'FILENAME': stage_rel}
            )
            logging.info(
                'BFP PREFLIGHT DIRECT HANDOFF source=%r stage=%r',
                p['filename'], stage_rel
            )
            self.sd.cmd_SDCARD_PRINT_FILE(sd_gcmd)
        except Exception:
            logging.exception('BFP PREFLIGHT could not stage/start')
            self.approved = None
            self.gcode.run_script_from_command('TURN_OFF_HEATERS')
            raise

    def assert_stage(self,gcmd):
        p=self.approved
        if not p or gcmd.get('ID') != p['id'] or not self.sd:
            raise gcmd.error('BFP preflight: unapproved staged print')
        path = self.sd.file_path()
        if not path or os.path.realpath(path) != p['path']:
            raise gcmd.error('BFP preflight: print file mismatch')

    def wait(self,gcmd):
        self.assert_stage(gcmd)
        values = self.approved['values']
        # Restore targets in case START_PRINT reset them. Do NOT turn off zones
        # in this stage: COPY/MIRROR may have enabled right zones intentionally.
        object_heaters = self.printer.lookup_object('heaters')
        for field in FIELDS:
            val = values[field]
            if val > 0:
                object_heaters.set_temperature(self._temp_heaters[field],val,wait=False)
        reactor = self.printer.get_reactor()
        start = reactor.monotonic()
        while True:
            now = reactor.monotonic()
            incomplete = []
            for field in FIELDS:
                val = values[field]
                if val <= 0: continue
                actual,target = self._temp_heaters[field].get_temp(now)
                if target <= 0 or actual < val-self.wait_tolerance:
                    incomplete.append(field)
            if not incomplete:
                break
            if self.printer.is_shutdown() or now-start > self.wait_timeout:
                raise gcmd.error('BFP preflight: temperature wait failed: '+','.join(incomplete))
            reactor.pause(now+1.)
        self.gcode.respond_info('BFP preflight: all requested heaters ready; extrusion permitted')

    def cancel(self,gcmd):
        self._session(gcmd)
        self.pending=None
        self.gcode.respond_raw('// action:prompt_end')
        gcmd.respond_info('BFP preflight: print cancelled before startup')

    def status(self,gcmd):
        self._require_mapped(gcmd)
        p = self.pending
        gcmd.respond_info('BFP preflight %s; %s' % (VERSION,
                          ('HELD '+p['filename'] if p else 'idle')))
        gcmd.respond_info('Heater names: '+str(self.field_to_heater))
        if p:
            gcmd.respond_info('Values: '+str(p['values']))
            self._prompt()
        handlers = getattr(self.gcode, 'ready_gcode_handlers', {})
        hook = (
            'SDCARD_PRINT_FILE_BASE' in handlers
            or 'BFP_SDCARD_PRINT_FILE_BASE' in handlers
        )
        gcmd.respond_info('Print-start hook: %s' % ('ACTIVE' if hook else 'NOT ACTIVE - normal Print clicks bypass preflight'))

    # Generic prompt keypad fallback, one selected field at a time.
    def edit(self,gcmd):
        p=self._session(gcmd)
        field=gcmd.get('FIELD').upper()
        if field not in FIELDS:
            raise gcmd.error('BFP preflight: unknown temperature field')
        p['editing'],p['digits']=field,''
        self._keypad()
    def key(self,gcmd):
        p=self._session(gcmd)
        if p['editing'] not in FIELDS:
            raise gcmd.error('BFP preflight: no keypad field')
        digit=gcmd.get_int('KEY',minval=0,maxval=9)
        if len(p['digits'])>2:
            raise gcmd.error('BFP preflight: max 3 digits')
        p['digits']+=str(digit)
        # Keep keypad open.
    def clear(self,gcmd):
        p=self._session(gcmd)
        p['digits']=''
        # Keep keypad open.
    def delete(self,gcmd):
        p=self._session(gcmd)
        p['digits']=p['digits'][:-1]
        # Keep keypad open.
    def done(self,gcmd):
        p=self._session(gcmd)
        if not p['editing'] or not p['digits']:
            raise gcmd.error('BFP preflight: enter digits')
        field=p['editing']
        val=float(p['digits'])
        self._validate(field,val,gcmd)
        p['values'][field]=val
        p['sources'][field]='operator'
        p['editing']=None
        self._prompt()

    def get_status(self,eventtime):
        p=self.pending
        return {'state': 'waiting' if p else ('printing' if self.approved else 'idle'),
                'filename':p['filename'] if p else None,
                'values':dict(p['values']) if p else None,
                'id':p['id'] if p else None}


def load_config(config):
    return Preflight(config)
