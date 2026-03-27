"""
Ground Station – Python side
Reads ASCII debug output from the ESP32 GS over serial.
Sends text commands that match handleSerialLine() in gs.cpp.
Logs parsed data to per-type CSV files.
"""

import re
import serial
import serial.tools.list_ports
import threading
import queue
import csv
import os
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from datetime import datetime

# ─────────────────────────────────────────────
#  ASCII LINE PARSERS
#  Each returns (type_name, fields_dict) or None
# ─────────────────────────────────────────────

def _parse_status_block(lines):
    """
    Parse a STATUS block (multi-line):
      STATUS seq: N
      State     : X
      Armed     : X
      GPS fix   : X
      LiPo      : X V
      Flags     : 0xXXXX (...)
    """
    fields = {}
    for line in lines:
        line = line.strip()
        m = re.match(r'STATUS seq:\s*(\d+)', line)
        if m: fields['seq'] = int(m.group(1)); continue
        m = re.match(r'State\s*:\s*(\S+)', line)
        if m: fields['state'] = m.group(1); continue
        m = re.match(r'Armed\s*:\s*(\d+)', line)
        if m: fields['armed'] = int(m.group(1)); continue
        m = re.match(r'GPS fix\s*:\s*(\d+)', line)
        if m: fields['gps_fix'] = int(m.group(1)); continue
        m = re.match(r'LiPo\s*:\s*([\d.]+)\s*V', line)
        if m: fields['lipo_V'] = float(m.group(1)); continue
        m = re.match(r'Flags\s*:\s*(0x[0-9A-Fa-f]+)', line)
        if m: fields['flags'] = m.group(1); continue
    return fields if fields else None

# Single-line parsers — each returns (type_name, dict) or None
_SINGLE_PARSERS = []

def _parser(fn):
    _SINGLE_PARSERS.append(fn)
    return fn

@_parser
def _parse_ack(line):
    m = re.match(r'ACK seq=(\d+) acked=(\d+) status=(\S+)', line)
    if m:
        return 'ACK', {'seq': int(m.group(1)), 'acked_seq': int(m.group(2)),
                       'status': m.group(3)}

@_parser
def _parse_event(line):
    m = re.match(r'EVENT seq=(\d+) name=(\S+) src=(\S+) value=(-?\d+)', line)
    if m:
        return 'EVENT', {'seq': int(m.group(1)), 'event': m.group(2),
                         'source': m.group(3), 'value': int(m.group(4))}

@_parser
def _parse_info(line):
    m = re.match(r'(INFO|ERROR) seq=(\d+) src=(\S+) code=(\S+) value=(-?\d+)(?:\s+\(([^)]+)\))?', line)
    if m:
        fields = {'seq': int(m.group(2)), 'source': m.group(3),
                  'code': m.group(4), 'value': int(m.group(5))}
        if m.group(6):
            fields['detail'] = m.group(6)
        return m.group(1), fields

@_parser
def _parse_gps(line):
    m = re.match(r'GPS seq=(\d+) fix=(\d+) lat=([-\d.]+) lon=([-\d.]+) alt=([-\d.]+) m', line)
    if m:
        return 'GPS', {'seq': int(m.group(1)), 'fix_type': int(m.group(2)),
                       'lat': float(m.group(3)), 'lon': float(m.group(4)),
                       'alt_m': float(m.group(5))}

@_parser
def _parse_primary(line):
    m = re.match(r'PRIMARY_MISSION seq=(\d+) pressure=(-?\d+) Pa temp=([-\d.]+) C', line)
    if m:
        return 'PRIMARY_MISSION', {'seq': int(m.group(1)),
                                   'pressure_Pa': int(m.group(2)),
                                   'temperature_C': float(m.group(3))}

@_parser
def _parse_attitude(line):
    m = re.match(r'ATTITUDE seq=(\d+) roll=([-\d.]+) pitch=([-\d.]+) yaw=([-\d.]+) deg', line)
    if m:
        return 'ATTITUDE', {'seq': int(m.group(1)), 'roll_deg': float(m.group(2)),
                            'pitch_deg': float(m.group(3)), 'yaw_deg': float(m.group(4))}

@_parser
def _parse_imu(line):
    m = re.match(r'IMU raw: acc\[g\] ([-\d.]+), ([-\d.]+), ([-\d.]+) gyro\[dps\] ([-\d.]+), ([-\d.]+), ([-\d.]+)', line)
    if m:
        vals = [float(m.group(i)) for i in range(1, 7)]
        return 'IMU_RAW', {'ax_g': vals[0], 'ay_g': vals[1], 'az_g': vals[2],
                           'gx_dps': vals[3], 'gy_dps': vals[4], 'gz_dps': vals[5]}

@_parser
def _parse_tx(line):
    m = re.match(r'\[GS\] TX cmd=(\S+) dest=(\d+) seq=(\d+) arg0=(-?\d+) arg1=(-?\d+)', line)
    if m:
        return 'TX', {'cmd': m.group(1), 'dest': int(m.group(2)),
                      'seq': int(m.group(3)), 'arg0': int(m.group(4)),
                      'arg1': int(m.group(5))}

def parse_line(line: str):
    """Try all single-line parsers. Returns (type_name, fields) or (None, None)."""
    for fn in _SINGLE_PARSERS:
        result = fn(line)
        if result:
            return result
    return None, None

# ─────────────────────────────────────────────
#  CSV LOGGER
# ─────────────────────────────────────────────
class CsvLogger:
    def __init__(self, log_dir: str):
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        self._writers = {}

    def log(self, type_name: str, fields: dict):
        row = {'timestamp': datetime.now().isoformat()}
        row.update(fields)

        if type_name not in self._writers:
            path = os.path.join(self.log_dir, f'{type_name}.csv')
            fh = open(path, 'a', newline='')
            fieldnames = list(row.keys())
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            if os.path.getsize(path) == 0:
                writer.writeheader()
            self._writers[type_name] = (fh, writer, fieldnames)

        fh, writer, fieldnames = self._writers[type_name]
        for k in row:
            if k not in fieldnames:
                fieldnames.append(k)
        writer.writerow({k: row.get(k, '') for k in fieldnames})
        fh.flush()

    def close(self):
        for fh, writer, _ in self._writers.values():
            fh.close()
        self._writers.clear()

# ─────────────────────────────────────────────
#  SERIAL THREAD
# ─────────────────────────────────────────────
class SerialWorker(threading.Thread):
    def __init__(self, port: str, baud: int, rx_queue: queue.Queue):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.rx_queue = rx_queue
        self._ser = None
        self._stop_evt = threading.Event()
        self._tx_queue = queue.Queue()
        self.connected = False
        self.error = None

    def send_line(self, text: str):
        self._tx_queue.put((text.strip() + '\n').encode())

    def stop(self):
        self._stop_evt.set()

    def run(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.connected = True
            buf = b''
            while not self._stop_evt.is_set():
                while not self._tx_queue.empty():
                    try:
                        self._ser.write(self._tx_queue.get_nowait())
                    except Exception:
                        pass
                try:
                    chunk = self._ser.read(256)
                except Exception as e:
                    self.error = str(e)
                    break
                if chunk:
                    buf += chunk
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        try:
                            text = line.decode('utf-8', errors='replace').strip()
                            if text:
                                self.rx_queue.put(text)
                        except Exception:
                            pass
        except Exception as e:
            self.error = str(e)
        finally:
            self.connected = False
            if self._ser and self._ser.is_open:
                self._ser.close()

# ─────────────────────────────────────────────
#  GUI
# ─────────────────────────────────────────────
class GroundStationApp(tk.Tk):
    DARK_BG  = '#0d1117'
    PANEL_BG = '#161b22'
    BORDER   = '#30363d'
    GREEN    = '#39d353'
    CYAN     = '#58a6ff'
    YELLOW   = '#e3b341'
    RED      = '#f85149'
    TEXT_FG  = '#c9d1d9'
    DIM_FG   = '#8b949e'
    MONO     = ('Courier New', 9)
    UI       = ('Segoe UI', 9)
    HEAD     = ('Segoe UI', 9, 'bold')

    TAG_COLOURS = {
        'STATUS':          '#39d353',
        'ACK':             '#8b949e',
        'GPS':             '#e3b341',
        'EVENT':           '#d2a8ff',
        'INFO':            '#58a6ff',
        'ERROR':           '#f85149',
        'ATTITUDE':        '#79c0ff',
        'IMU_RAW':         '#ffa657',
        'PRIMARY_MISSION': '#56d364',
        'TX':              '#e3b341',
        'RAW':             '#4a5568',
    }

    def __init__(self):
        super().__init__()
        self.title('Ground Station')
        self.geometry('1150x800')
        self.configure(bg=self.DARK_BG)

        self._worker: SerialWorker = None
        self._rx_queue = queue.Queue()
        self._logger: CsvLogger = None
        self._log_dir = f'gs_logs_{datetime.now().strftime("%Y%m%d_%H%M%S")}'

        self._in_status_block = False
        self._status_lines = []

        self._build_styles()
        self._build_ui()
        self._logger = CsvLogger(self._log_dir)
        self._poll()

    def _build_styles(self):
        s = ttk.Style(self)
        s.theme_use('clam')
        s.configure('TFrame', background=self.DARK_BG)
        s.configure('TLabel', background=self.DARK_BG, foreground=self.TEXT_FG, font=self.UI)
        s.configure('TButton', background=self.BORDER, foreground=self.TEXT_FG,
                    font=self.UI, relief='flat', padding=4)
        s.map('TButton', background=[('active', '#21262d')])
        s.configure('Accent.TButton', background='#238636', foreground='#fff',
                    font=self.HEAD, relief='flat', padding=4)
        s.map('Accent.TButton', background=[('active', '#2ea043')])
        s.configure('TCombobox', fieldbackground=self.PANEL_BG, background=self.PANEL_BG,
                    foreground=self.TEXT_FG)
        s.configure('TEntry', fieldbackground=self.PANEL_BG, foreground=self.TEXT_FG,
                    insertcolor=self.TEXT_FG)

    def _build_ui(self):
        top = tk.Frame(self, bg=self.PANEL_BG, height=42)
        top.pack(fill='x')
        top.pack_propagate(False)
        tk.Label(top, text='◈ GROUND STATION', bg=self.PANEL_BG, fg=self.CYAN,
                 font=('Courier New', 13, 'bold')).pack(side='left', padx=14)
        self._conn_dot = tk.Label(top, text='●', bg=self.PANEL_BG, fg=self.RED,
                                  font=('Courier New', 14))
        self._conn_dot.pack(side='right', padx=6)
        self._conn_lbl = tk.Label(top, text='DISCONNECTED', bg=self.PANEL_BG,
                                  fg=self.RED, font=self.HEAD)
        self._conn_lbl.pack(side='right', padx=2)
        tk.Label(top, text=f'logs → {self._log_dir}/', bg=self.PANEL_BG,
                 fg=self.DIM_FG, font=('Courier New', 8)).pack(side='right', padx=10)

        paned = tk.PanedWindow(self, orient='horizontal', bg=self.DARK_BG,
                               sashwidth=4, sashrelief='flat')
        paned.pack(fill='both', expand=True, padx=4, pady=4)

        left = tk.Frame(paned, bg=self.DARK_BG)
        paned.add(left, minsize=290)
        self._build_connection(left)
        self._build_status_panel(left)
        self._build_gps_panel(left)
        self._build_attitude_panel(left)

        right = tk.Frame(paned, bg=self.DARK_BG)
        paned.add(right, minsize=520)
        self._build_log(right)
        self._build_commands(right)

    def _build_connection(self, parent):
        f = self._panel(parent, 'CONNECTION')
        row = tk.Frame(f, bg=self.PANEL_BG)
        row.pack(fill='x', padx=8, pady=4)
        tk.Label(row, text='Port', bg=self.PANEL_BG, fg=self.DIM_FG,
                 font=self.UI).grid(row=0, column=0, sticky='w', padx=4)
        self._port_var = tk.StringVar()
        self._port_cb = ttk.Combobox(row, textvariable=self._port_var, width=14)
        self._port_cb.grid(row=0, column=1, padx=4)
        ttk.Button(row, text='↺', width=3, command=self._refresh_ports).grid(row=0, column=2)
        tk.Label(row, text='Baud', bg=self.PANEL_BG, fg=self.DIM_FG,
                 font=self.UI).grid(row=1, column=0, sticky='w', padx=4, pady=3)
        self._baud_var = tk.StringVar(value='115200')
        ttk.Combobox(row, textvariable=self._baud_var, width=14,
                     values=['9600','19200','57600','115200','230400','500000','1000000']
                     ).grid(row=1, column=1, padx=4)
        self._conn_btn = ttk.Button(f, text='CONNECT', style='Accent.TButton',
                                    command=self._toggle_connect)
        self._conn_btn.pack(anchor='w', padx=8, pady=(0, 6))
        self._refresh_ports()

    def _build_status_panel(self, parent):
        f = self._panel(parent, 'FLIGHT STATUS')
        self._sv = {}
        for label, key, colour in [
            ('State',   'state',   self.GREEN),
            ('Armed',   'armed',   self.GREEN),
            ('GPS Fix', 'gps_fix', self.GREEN),
            ('Battery', 'lipo_V',  self.GREEN),
            ('Flags',   'flags',   self.DIM_FG),
        ]:
            r = tk.Frame(f, bg=self.PANEL_BG)
            r.pack(fill='x', padx=10, pady=1)
            tk.Label(r, text=f'{label}:', bg=self.PANEL_BG, fg=self.DIM_FG,
                     font=self.UI, width=10, anchor='w').pack(side='left')
            v = tk.StringVar(value='—')
            self._sv[key] = v
            tk.Label(r, textvariable=v, bg=self.PANEL_BG, fg=colour,
                     font=self.MONO, anchor='w').pack(side='left')
        tk.Frame(f, bg=self.PANEL_BG, height=4).pack()

    def _build_gps_panel(self, parent):
        f = self._panel(parent, 'GPS')
        self._gv = {}
        for label, key in [('Lat','lat'),('Lon','lon'),
                            ('Alt (m)','alt_m'),('Fix','fix_type')]:
            r = tk.Frame(f, bg=self.PANEL_BG)
            r.pack(fill='x', padx=10, pady=1)
            tk.Label(r, text=f'{label}:', bg=self.PANEL_BG, fg=self.DIM_FG,
                     font=self.UI, width=10, anchor='w').pack(side='left')
            v = tk.StringVar(value='—')
            self._gv[key] = v
            tk.Label(r, textvariable=v, bg=self.PANEL_BG, fg=self.YELLOW,
                     font=self.MONO, anchor='w').pack(side='left')
        tk.Frame(f, bg=self.PANEL_BG, height=4).pack()

    def _build_attitude_panel(self, parent):
        f = self._panel(parent, 'ATTITUDE / IMU')
        self._av = {}
        for label, key in [('Roll (°)','roll_deg'),('Pitch (°)','pitch_deg'),
                            ('Yaw (°)','yaw_deg'),('Ax (g)','ax_g'),
                            ('Ay (g)','ay_g'),('Az (g)','az_g')]:
            r = tk.Frame(f, bg=self.PANEL_BG)
            r.pack(fill='x', padx=10, pady=1)
            tk.Label(r, text=f'{label}:', bg=self.PANEL_BG, fg=self.DIM_FG,
                     font=self.UI, width=12, anchor='w').pack(side='left')
            v = tk.StringVar(value='—')
            self._av[key] = v
            tk.Label(r, textvariable=v, bg=self.PANEL_BG, fg=self.CYAN,
                     font=self.MONO, anchor='w').pack(side='left')
        tk.Frame(f, bg=self.PANEL_BG, height=4).pack()

    def _build_log(self, parent):
        f = self._panel(parent, 'SERIAL LOG')
        f.pack(fill='both', expand=True, padx=4, pady=4)
        ctrl = tk.Frame(f, bg=self.PANEL_BG)
        ctrl.pack(fill='x', padx=6, pady=2)
        tk.Label(ctrl, text='Filter:', bg=self.PANEL_BG, fg=self.DIM_FG,
                 font=self.UI).pack(side='left', padx=4)
        self._filter_var = tk.StringVar(value='ALL')
        types = ['ALL','STATUS','GPS','ATTITUDE','IMU_RAW','ACK','EVENT',
                 'INFO','ERROR','PRIMARY_MISSION','TX','RAW']
        ttk.Combobox(ctrl, textvariable=self._filter_var, values=types,
                     width=16, state='readonly').pack(side='left')
        ttk.Button(ctrl, text='Clear', command=self._clear_log).pack(side='right', padx=4)
        self._log = scrolledtext.ScrolledText(
            f, bg=self.DARK_BG, fg=self.TEXT_FG, font=self.MONO,
            insertbackground=self.TEXT_FG, relief='flat',
            state='disabled', height=20, wrap='none')
        self._log.pack(fill='both', expand=True, padx=6, pady=(2,6))
        for tag, colour in self.TAG_COLOURS.items():
            self._log.tag_config(tag, foreground=colour)

    def _build_commands(self, parent):
        f = self._panel(parent, 'COMMANDS')
        f.pack(fill='x', padx=4, pady=(0, 4))

        grid = tk.Frame(f, bg=self.PANEL_BG)
        grid.pack(fill='x', padx=6, pady=4)

        # Strings must match handleSerialLine() exactly
        btns = [
            ('Ping FC',      'ping fc'),
            ('Ping Bridge',  'ping bridge'),
            ('Status',       'status'),
            ('Start Camera', 'startcam'),
            ('Stop Camera',  'stopcam'),
            ('Take Photo',   'photo'),
            ('Test Baro',    'test_baro'),
            ('Test IMU',     'test_imu'),
            ('Test Therm.',  'test_thermistor'),
            ('Test LDR',     'test_ldr'),
            ('Test GPS',     'test_gps'),
        ]
        for i, (label, cmd) in enumerate(btns):
            ttk.Button(grid, text=label,
                       command=lambda c=cmd: self._send(c)
                       ).grid(row=i // 6, column=i % 6, padx=3, pady=2, sticky='ew')
        for c in range(6):
            grid.columnconfigure(c, weight=1)

        sr = tk.Frame(f, bg=self.PANEL_BG)
        sr.pack(fill='x', padx=8, pady=(2, 4))
        tk.Label(sr, text='Set State:', bg=self.PANEL_BG, fg=self.DIM_FG,
                 font=self.UI).pack(side='left', padx=4)
        states = ['0 (IDLE)','1 (PRELAUNCH)','2 (ROCKET)','3 (FREEFLIGHT)',
                  '4 (POWERED_FLIGHT)','5 (GROUND)','6 (FAULT)']
        self._state_var = tk.StringVar(value=states[0])
        ttk.Combobox(sr, textvariable=self._state_var, values=states,
                     width=20, state='readonly').pack(side='left', padx=4)
        ttk.Button(sr, text='Send', style='Accent.TButton',
                   command=self._send_state).pack(side='left', padx=4)

        cr = tk.Frame(f, bg=self.PANEL_BG)
        cr.pack(fill='x', padx=8, pady=(0, 8))
        tk.Label(cr, text='Raw:', bg=self.PANEL_BG, fg=self.DIM_FG,
                 font=self.UI).pack(side='left', padx=4)
        self._cli_var = tk.StringVar()
        e = ttk.Entry(cr, textvariable=self._cli_var, width=42)
        e.pack(side='left', padx=4, fill='x', expand=True)
        e.bind('<Return>', lambda _: self._send_raw())
        ttk.Button(cr, text='Send', command=self._send_raw).pack(side='left', padx=4)
        tk.Label(cr, text='any command from handleSerialLine()', bg=self.PANEL_BG,
                 fg=self.DIM_FG, font=('Courier New', 8)).pack(side='left', padx=4)

    def _panel(self, parent, title):
        outer = tk.Frame(parent, bg=self.BORDER)
        outer.pack(fill='x', padx=4, pady=3)
        inner = tk.Frame(outer, bg=self.PANEL_BG)
        inner.pack(fill='both', expand=True, padx=1, pady=1)
        tk.Label(inner, text=title, bg=self.PANEL_BG, fg=self.CYAN,
                 font=('Courier New', 9, 'bold')).pack(anchor='w', padx=10, pady=(4,2))
        tk.Frame(inner, bg=self.BORDER, height=1).pack(fill='x', padx=6)
        return inner

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_cb['values'] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

    def _toggle_connect(self):
        if self._worker and self._worker.connected:
            self._worker.stop()
            self._worker = None
            self._set_conn(False)
            return
        port = self._port_var.get()
        if not port:
            messagebox.showerror('Error', 'Select a serial port')
            return
        self._worker = SerialWorker(port, int(self._baud_var.get()), self._rx_queue)
        self._worker.start()
        self.after(400, self._check_conn)

    def _check_conn(self):
        if self._worker and self._worker.connected:
            self._set_conn(True)
        elif self._worker and self._worker.error:
            messagebox.showerror('Connection Error', self._worker.error)
            self._worker = None

    def _set_conn(self, state: bool):
        if state:
            self._conn_dot.config(fg=self.GREEN)
            self._conn_lbl.config(text='CONNECTED', fg=self.GREEN)
            self._conn_btn.config(text='DISCONNECT')
        else:
            self._conn_dot.config(fg=self.RED)
            self._conn_lbl.config(text='DISCONNECTED', fg=self.RED)
            self._conn_btn.config(text='CONNECT')

    def _send(self, text: str):
        if not self._worker or not self._worker.connected:
            self._log_line('Not connected', 'ERROR')
            return
        self._worker.send_line(text)
        self._log_line(f'>> {text}', 'TX')

    def _send_state(self):
        val = self._state_var.get().split()[0]
        self._send(f'state {val}')

    def _send_raw(self):
        line = self._cli_var.get().strip()
        self._cli_var.set('')
        if line:
            self._send(line)

    def _poll(self):
        try:
            while True:
                line = self._rx_queue.get_nowait()
                self._handle_line(line)
        except queue.Empty:
            pass

        if self._worker and not self._worker.connected and self._worker.error:
            self._set_conn(False)
            self._log_line(f'Serial error: {self._worker.error}', 'ERROR')
            self._worker = None

        self.after(20, self._poll)

    def _handle_line(self, line: str):
        # ── STATUS block (delimited by dashes) ──
        if '------------------------------' in line:
            if self._in_status_block and self._status_lines:
                fields = _parse_status_block(self._status_lines)
                if fields:
                    self._update_status(fields)
                    self._logger.log('STATUS', fields)
                    ts = datetime.now().strftime('%H:%M:%S.%f')[:12]
                    summary = (f"[{ts}] STATUS           "
                               f"state={fields.get('state','?')}  "
                               f"armed={fields.get('armed','?')}  "
                               f"bat={fields.get('lipo_V','?')}V  "
                               f"gps={fields.get('gps_fix','?')}")
                    filt = self._filter_var.get()
                    if filt in ('ALL', 'STATUS'):
                        self._log_line(summary, 'STATUS')
                self._status_lines = []
                self._in_status_block = False
            else:
                self._in_status_block = True
                self._status_lines = []
            return

        if self._in_status_block:
            self._status_lines.append(line)
            return

        # ── Single-line packets ──
        type_name, fields = parse_line(line)
        filt = self._filter_var.get()

        if type_name and fields:
            self._logger.log(type_name, fields)
            ts = datetime.now().strftime('%H:%M:%S.%f')[:12]
            parts = '  '.join(f'{k}={v}' for k, v in fields.items())
            display = f'[{ts}] {type_name:<16} {parts}'

            if filt in ('ALL', type_name):
                self._log_line(display, type_name)

            if type_name == 'GPS':
                for k in ('lat','lon','alt_m','fix_type'):
                    if k in fields and k in self._gv:
                        v = fields[k]
                        self._gv[k].set(f'{v:.7f}' if isinstance(v, float) else str(v))

            elif type_name == 'ATTITUDE':
                for k in ('roll_deg','pitch_deg','yaw_deg'):
                    if k in fields and k in self._av:
                        self._av[k].set(f'{fields[k]:.2f}')

            elif type_name == 'IMU_RAW':
                for k in ('ax_g','ay_g','az_g'):
                    if k in fields and k in self._av:
                        self._av[k].set(f'{fields[k]:.3f}')
        else:
            if filt in ('ALL', 'RAW'):
                self._log_line(line, 'RAW')

    def _update_status(self, fields: dict):
        for k, v in fields.items():
            if k in self._sv:
                self._sv[k].set(str(v))

    def _log_line(self, text: str, tag: str = ''):
        self._log.config(state='normal')
        self._log.insert('end', text + '\n', tag or ())
        self._log.see('end')
        lines = int(self._log.index('end-1c').split('.')[0])
        if lines > 3000:
            self._log.delete('1.0', f'{lines - 3000}.0')
        self._log.config(state='disabled')

    def _clear_log(self):
        self._log.config(state='normal')
        self._log.delete('1.0', 'end')
        self._log.config(state='disabled')

    def on_close(self):
        if self._worker:
            self._worker.stop()
        if self._logger:
            self._logger.close()
        self.destroy()


if __name__ == '__main__':
    app = GroundStationApp()
    app.protocol('WM_DELETE_WINDOW', app.on_close)
    app.mainloop()
