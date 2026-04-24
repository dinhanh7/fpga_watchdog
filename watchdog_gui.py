import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import struct
import threading
import time
import queue

# === Constants & Theme ===
BG_COLOR = "#0f172a"
PANEL_BG = "#1e293b"
TEXT_COLOR = "#f8fafc"
ACCENT_BLUE = "#3b82f6"
ACCENT_GREEN = "#10b981"
ACCENT_RED = "#ef4444"
ACCENT_YELLOW = "#f59e0b"
BORDER_COLOR = "#334155"

REG_CTRL, REG_TWD_MS, REG_TRST_MS, REG_ARM_DELAY, REG_STATUS = 0x00, 0x04, 0x08, 0x0C, 0x10
CMD_WRITE, CMD_READ, CMD_KICK, CMD_STATUS = 0x01, 0x02, 0x03, 0x04
CTRL_EN_SW, CTRL_WDI_SRC, CTRL_CLR_FLT = (1<<0), (1<<1), (1<<2)
ST_EN_EFF, ST_FAULT, ST_ENOUT, ST_WDO, ST_KICK_SRC = (1<<0), (1<<1), (1<<2), (1<<3), (1<<4)

# === UART Protocol Helpers ===
def build_frame(cmd, addr, data_bytes=[]):
    core = [cmd, addr, len(data_bytes)] + data_bytes
    chk = 0
    for b in core: chk ^= b
    return bytes([0x55] + core + [chk])

def recv_frame(ser, timeout=0.2):
    ser.timeout = timeout
    # Sync header
    for _ in range(5):
        b = ser.read(1)
        if not b: return None
        if b[0] == 0x55: break
    else:
        return None
    # Read CMD, ADDR, LEN
    hdr = ser.read(3)
    if len(hdr) < 3: return None
    # Read DATA + CHK
    body = ser.read(hdr[2] + 1)
    if len(body) < hdr[2] + 1: return None
    return bytes([0x55]) + hdr + body

# === Main Application ===
class WatchdogGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Watchdog Monitor Pro")
        self.root.geometry("1100x750")
        self.root.configure(bg=BG_COLOR)

        self.ser = None
        self.polling = False
        self.kick_running = False
        self._poll_pending = False
        self._kick_pending = False

        # Single command queue — ALL serial I/O goes through one worker thread
        self._cmd_queue = queue.Queue()
        self._worker_running = True
        self._worker_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self._worker_thread.start()

        self._setup_styles()
        self._build_ui()

    # ------------------------------------------------------------------ #
    #  SERIAL WORKER — the ONLY thread that touches self.ser              #
    # ------------------------------------------------------------------ #
    def _serial_worker(self):
        """Single-threaded command executor. Eliminates all lock contention."""
        while self._worker_running:
            try:
                item = self._cmd_queue.get(timeout=0.05)
            except queue.Empty:
                continue

            kind = item[0]

            if kind == "poll":
                self._do_poll()
            elif kind == "kick":
                self._do_kick()
            elif kind == "cmd":
                cmd, addr, data_bytes, expect_data, callback = item[1:]
                result = self._do_send(cmd, addr, data_bytes, expect_data)
                if callback:
                    self.root.after(0, callback, result)
            elif kind == "write_cfg":
                twd, trst, arm = item[1:]
                self._do_send(CMD_WRITE, REG_TWD_MS, list(struct.pack(">I", twd)), False)
                time.sleep(0.05)
                self._do_send(CMD_WRITE, REG_TRST_MS, list(struct.pack(">I", trst)), False)
                time.sleep(0.05)
                self._do_send(CMD_WRITE, REG_ARM_DELAY, list(struct.pack(">I", arm)), False)
            elif kind == "read_cfg":
                v1 = self._do_send(CMD_READ, REG_TWD_MS, [], True)
                time.sleep(0.05)
                v2 = self._do_send(CMD_READ, REG_TRST_MS, [], True)
                time.sleep(0.05)
                v3 = self._do_send(CMD_READ, REG_ARM_DELAY, [], True)
                self.root.after(0, self._update_cfg_ui, v1, v2, v3)
            elif kind == "read_regmap":
                results = []
                for addr, _ in self._regmap_rows:
                    val = self._do_send(CMD_READ, addr, [], True)
                    results.append(val)
                    time.sleep(0.04)
                self.root.after(0, self._update_reg_map_ui, results)
            # Small breathing gap between any two serial transactions
            time.sleep(0.008)

    def _do_send(self, cmd, addr, data_bytes, expect_data):
        """Execute a single UART transaction. Called ONLY from worker thread."""
        if not self._check_conn():
            return None
        frame = build_frame(cmd, addr, data_bytes)
        self.root.after(0, self._log, f"TX: {' '.join(f'{b:02X}' for b in frame)}", "TX")
        try:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            resp = recv_frame(self.ser, timeout=0.2)
            if resp:
                self.root.after(0, self._log, f"RX: {' '.join(f'{b:02X}' for b in resp)}", "RX")
                if expect_data and len(resp) >= 9:
                    return struct.unpack(">I", resp[4:8])[0]
                return resp
            else:
                self.root.after(0, self._log, "RX: Timeout", "ERR")
                return None
        except Exception as e:
            self.root.after(0, self._log, f"Error: {e}", "ERR")
            return None

    def _do_poll(self):
        """Read STATUS register silently (no TX/RX log spam)."""
        try:
            if not self._check_conn():
                return
            frame = build_frame(CMD_STATUS, 0x00)
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            resp = recv_frame(self.ser, timeout=0.15)
            if resp and len(resp) >= 9:
                val = struct.unpack(">I", resp[4:8])[0]
                self.root.after(0, self._update_status_display, val)
        except:
            pass
        finally:
            self._poll_pending = False

    def _do_kick(self):
        """Send KICK command."""
        try:
            if not self._check_conn():
                return
            frame = build_frame(CMD_KICK, 0x00)
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            resp = recv_frame(self.ser, timeout=0.15)
            ok = resp is not None
            self.root.after(0, self._log, f"Kick {'OK' if ok else 'FAILED'}", "RX" if ok else "ERR")
        except:
            pass
        finally:
            self._kick_pending = False

    # ------------------------------------------------------------------ #
    #  PUBLIC API — queue commands, never block the GUI                    #
    # ------------------------------------------------------------------ #
    def _send_cmd_async(self, cmd, addr, data_bytes=[], expect_data=False, callback=None):
        self._cmd_queue.put(("cmd", cmd, addr, data_bytes, expect_data, callback))

    def _write_reg(self, addr, val, callback=None):
        self._send_cmd_async(CMD_WRITE, addr, list(struct.pack(">I", val)), callback=callback)

    def _read_reg(self, addr, callback=None):
        self._send_cmd_async(CMD_READ, addr, expect_data=True, callback=callback)

    # ------------------------------------------------------------------ #
    #  STYLES                                                             #
    # ------------------------------------------------------------------ #
    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background=BG_COLOR)
        style.configure("Panel.TFrame", background=PANEL_BG)
        style.configure("TLabel", background=BG_COLOR, foreground=TEXT_COLOR, font=("Inter", 10))
        style.configure("Panel.TLabel", background=PANEL_BG, foreground=TEXT_COLOR, font=("Inter", 10))
        style.configure("StatusGreen.TLabel", background=PANEL_BG, foreground=ACCENT_GREEN, font=("Inter", 10, "bold"))
        style.configure("StatusRed.TLabel", background=PANEL_BG, foreground=ACCENT_RED, font=("Inter", 10, "bold"))
        style.configure("StatusYellow.TLabel", background=PANEL_BG, foreground=ACCENT_YELLOW, font=("Inter", 10, "bold"))
        style.configure("H1.TLabel", background=BG_COLOR, foreground=TEXT_COLOR, font=("Inter", 16, "bold"))
        style.configure("H2.TLabel", background=PANEL_BG, foreground="#94a3b8", font=("Inter", 11, "bold"))
        style.configure("TButton", font=("Inter", 10, "bold"), padding=6, background=ACCENT_BLUE, foreground="white", borderwidth=0)
        style.map("TButton", background=[("active", "#2563eb"), ("disabled", "#475569")])
        style.configure("Danger.TButton", background=ACCENT_RED)
        style.map("Danger.TButton", background=[("active", "#dc2626")])
        style.configure("Success.TButton", background=ACCENT_GREEN)
        style.map("Success.TButton", background=[("active", "#059669")])
        style.configure("TCheckbutton", background=PANEL_BG, foreground=TEXT_COLOR, font=("Inter", 10))
        style.map("TCheckbutton", background=[("active", PANEL_BG)])

    # ------------------------------------------------------------------ #
    #  UI BUILD                                                           #
    # ------------------------------------------------------------------ #
    def _create_panel(self, parent, title):
        panel = tk.Frame(parent, bg=PANEL_BG, highlightbackground=BORDER_COLOR, highlightthickness=1)
        panel.pack(fill="both", expand=True, padx=8, pady=8)
        ttk.Label(panel, text=title, style="H2.TLabel").pack(anchor="w", padx=15, pady=(15, 10))
        content = tk.Frame(panel, bg=PANEL_BG)
        content.pack(fill="both", expand=True, padx=15, pady=(0, 15))
        return content

    def _build_ui(self):
        # Top Bar
        top_bar = tk.Frame(self.root, bg=BG_COLOR)
        top_bar.pack(fill="x", padx=15, pady=(15, 5))
        ttk.Label(top_bar, text="Watchdog Monitor Pro", style="H1.TLabel").pack(side="left")

        conn_frame = tk.Frame(top_bar, bg=BG_COLOR)
        conn_frame.pack(side="right")
        self.port_var = tk.StringVar(value="COM5")
        self.port_cb = ttk.Combobox(conn_frame, textvariable=self.port_var, width=10,
                                     values=[p.device for p in serial.tools.list_ports.comports()])
        self.port_cb.pack(side="left", padx=5)
        self.conn_btn = ttk.Button(conn_frame, text="Connect", command=self._toggle_connect)
        self.conn_btn.pack(side="left", padx=5)
        self.conn_status = tk.Label(conn_frame, text="● Offline", fg=ACCENT_RED, bg=BG_COLOR, font=("Inter", 10, "bold"))
        self.conn_status.pack(side="left", padx=10)

        # Main Layout (3 columns)
        grid = tk.Frame(self.root, bg=BG_COLOR)
        grid.pack(fill="both", expand=True, padx=7, pady=5)
        col1 = tk.Frame(grid, bg=BG_COLOR); col1.pack(side="left", fill="both", expand=True)
        col2 = tk.Frame(grid, bg=BG_COLOR); col2.pack(side="left", fill="both", expand=True)
        col3 = tk.Frame(grid, bg=BG_COLOR); col3.pack(side="left", fill="both", expand=True)

        # === COLUMN 1: Hardware & Controls ===
        status_panel = self._create_panel(col1, "LIVE STATUS")
        fsm_frame = tk.Frame(status_panel, bg=PANEL_BG)
        fsm_frame.pack(fill="x", pady=(0, 8))
        ttk.Label(fsm_frame, text="CORE STATE:", style="Panel.TLabel", width=14).pack(side="left")
        self.lbl_fsm = ttk.Label(fsm_frame, text="---", style="StatusYellow.TLabel", width=14, anchor="w")
        self.lbl_fsm.pack(side="left", padx=5)

        self.status_labels = {}
        names = [("EN_EFF","System Enable"),("FAULT","Fault Active"),
                 ("ENOUT","ENOUT Pin"),("WDO","WDO Pin"),("KICK_SRC","Last Kick")]
        for key, txt in names:
            f = tk.Frame(status_panel, bg=PANEL_BG)
            f.pack(fill="x", pady=2)
            ttk.Label(f, text=f"{txt}:", style="Panel.TLabel", width=14).pack(side="left")
            lbl = ttk.Label(f, text="---", style="StatusYellow.TLabel", width=14, anchor="w")
            lbl.pack(side="left", padx=5)
            self.status_labels[key] = lbl

        hw_panel = self._create_panel(col1, "HARDWARE CONTROLS")
        led_frame = tk.Frame(hw_panel, bg=PANEL_BG)
        led_frame.pack(fill="x", pady=10)
        self.led_wdo = self._create_led(led_frame, "WDO (Fault)", ACCENT_RED)
        self.led_enout = self._create_led(led_frame, "ENOUT (Ready)", ACCENT_GREEN)

        sw_frame = tk.Frame(hw_panel, bg=PANEL_BG)
        sw_frame.pack(fill="x", pady=15)
        self.en_sw_var = tk.BooleanVar(value=False)
        self.wdi_src_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(sw_frame, text="System Enable (SW)", variable=self.en_sw_var).pack(anchor="w", pady=5)
        ttk.Checkbutton(sw_frame, text="Kick Source: UART", variable=self.wdi_src_var).pack(anchor="w", pady=5)

        btn_frame = tk.Frame(hw_panel, bg=PANEL_BG)
        btn_frame.pack(fill="x", pady=15)
        ttk.Button(btn_frame, text="Apply CTRL", command=self._apply_ctrl).pack(fill="x", pady=3)
        ttk.Button(btn_frame, text="Clear Fault", command=self._clear_fault, style="Danger.TButton").pack(fill="x", pady=3)
        ttk.Button(btn_frame, text="Disable WDG", command=self._disable_wdg).pack(fill="x", pady=3)

        # === COLUMN 2: Registers & Auto-Kick ===
        reg_panel = self._create_panel(col2, "REGISTER CONFIG")
        self.reg_vars = {}
        regs = [("tWD Timeout (ms)", "twd", "1600"),
                ("tRST Delay (ms)", "trst", "200"),
                ("Arming Delay (µs)", "arm", "150")]
        for txt, key, default in regs:
            f = tk.Frame(reg_panel, bg=PANEL_BG)
            f.pack(fill="x", pady=5)
            ttk.Label(f, text=txt, style="Panel.TLabel", width=16).pack(side="left")
            v = tk.StringVar(value=default); self.reg_vars[key] = v
            tk.Entry(f, textvariable=v, bg="#334155", fg="white", insertbackground="white", relief="flat").pack(side="right", fill="x", expand=True)
        ttk.Button(reg_panel, text="Write All", command=self._write_config).pack(fill="x", pady=(10,3))
        ttk.Button(reg_panel, text="Read All", command=self._read_config).pack(fill="x", pady=3)

        # === COLUMN 2: Register Map Viewer ===
        regmap_panel = self._create_panel(col2, "REGISTER MAP VIEWER")

        # Table header
        hdr = tk.Frame(regmap_panel, bg="#273549")
        hdr.pack(fill="x", pady=(0, 4))
        for col_txt, col_w in [("Addr", 5), ("Name", 14), ("Value (dec)", 10), ("Raw (hex)", 10)]:
            ttk.Label(hdr, text=col_txt, style="Panel.TLabel", width=col_w,
                      anchor="center", font=("Consolas", 9, "bold")).pack(side="left", padx=2)

        # Register rows: (addr, label)
        self._regmap_rows = [
            (REG_CTRL,      "CTRL"),
            (REG_TWD_MS,    "tWD_ms"),
            (REG_TRST_MS,   "tRST_ms"),
            (REG_ARM_DELAY, "ARM_DLY"),
            (REG_STATUS,    "STATUS"),
        ]
        self._regmap_dec_vars = []
        self._regmap_hex_vars = []
        for addr, name in self._regmap_rows:
            row = tk.Frame(regmap_panel, bg=PANEL_BG)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=f"0x{addr:02X}", style="Panel.TLabel",
                      width=5, anchor="center", font=("Consolas", 9)).pack(side="left", padx=2)
            ttk.Label(row, text=name, style="Panel.TLabel",
                      width=14, anchor="w", font=("Consolas", 9)).pack(side="left", padx=2)
            dv = tk.StringVar(value="---")
            hv = tk.StringVar(value="---")
            self._regmap_dec_vars.append(dv)
            self._regmap_hex_vars.append(hv)
            tk.Label(row, textvariable=dv, bg="#0f172a", fg=ACCENT_GREEN,
                     font=("Consolas", 9), width=10, anchor="e", relief="flat").pack(side="left", padx=2)
            tk.Label(row, textvariable=hv, bg="#0f172a", fg="#94a3b8",
                     font=("Consolas", 9), width=10, anchor="e", relief="flat").pack(side="left", padx=2)

        ttk.Button(regmap_panel, text="⟳ Refresh All Registers",
                   command=self._refresh_reg_map).pack(fill="x", pady=(8, 0))

        kick_panel = self._create_panel(col2, "HEARTBEAT (AUTO-KICK)")
        kf = tk.Frame(kick_panel, bg=PANEL_BG)
        kf.pack(fill="x", pady=5)
        ttk.Label(kf, text="Interval (s):", style="Panel.TLabel").pack(side="left")
        self.kick_interval = tk.StringVar(value="1.0")
        tk.Entry(kf, textvariable=self.kick_interval, width=8, bg="#334155", fg="white", insertbackground="white", relief="flat").pack(side="left", padx=10)
        self.auto_kick_var = tk.BooleanVar()
        ttk.Checkbutton(kick_panel, text="Enable Auto-Kick", variable=self.auto_kick_var, command=self._toggle_auto_kick).pack(anchor="w", pady=5)
        ttk.Button(kick_panel, text="Send Manual Kick", command=self._send_kick, style="Success.TButton").pack(fill="x", pady=5)

        # === COLUMN 3: Console ===
        cons_panel = self._create_panel(col3, "SYSTEM CONSOLE")
        cons_tools = tk.Frame(cons_panel, bg=PANEL_BG)
        cons_tools.pack(fill="x", pady=(0, 5))
        self.poll_btn = ttk.Button(cons_tools, text="▶ Start Polling", command=self._toggle_polling)
        self.poll_btn.pack(side="left", fill="x", expand=True, padx=(0,5))
        ttk.Button(cons_tools, text="Clear", command=self._clear_log).pack(side="right")

        self.log_box = scrolledtext.ScrolledText(cons_panel, bg="#020617", fg="#10b981",
                                                 font=("Consolas", 15), relief="flat", state="disabled")
        self.log_box.pack(fill="both", expand=True)
        self.log_box.tag_config("TX", foreground="#3b82f6")
        self.log_box.tag_config("RX", foreground="#d946ef")
        self.log_box.tag_config("ERR", foreground="#ef4444")
        self.log_box.tag_config("SYS", foreground="#94a3b8")

    def _create_led(self, parent, text, color):
        f = tk.Frame(parent, bg=PANEL_BG)
        f.pack(fill="x", pady=4)
        canvas = tk.Canvas(f, width=16, height=16, bg=PANEL_BG, highlightthickness=0)
        canvas.pack(side="left", padx=(0, 8))
        circle = canvas.create_oval(2, 2, 14, 14, fill="#334155", outline="")
        ttk.Label(f, text=text, style="Panel.TLabel").pack(side="left")
        return {"canvas": canvas, "circle": circle, "on_color": color}

    def _set_led(self, led, state):
        color = led["on_color"] if state else "#334155"
        led["canvas"].itemconfig(led["circle"], fill=color)

    # ------------------------------------------------------------------ #
    #  CONNECTION                                                         #
    # ------------------------------------------------------------------ #
    def _toggle_connect(self):
        if self.ser and self.ser.is_open:
            self.polling = False
            self.kick_running = False
            # Drain pending commands so worker doesn't touch a closed port
            while not self._cmd_queue.empty():
                try: self._cmd_queue.get_nowait()
                except queue.Empty: break
            time.sleep(0.02)  # let worker finish current op
            self.ser.close()
            self.ser = None
            self.conn_btn.config(text="Connect")
            self.conn_status.config(text="● Offline", fg=ACCENT_RED)
            self.poll_btn.config(text="▶ Start Polling")
            self.auto_kick_var.set(False)
            self._log("Disconnected.", "SYS")
        else:
            try:
                self.ser = serial.Serial(self.port_var.get(), 115200, timeout=0.15)
                time.sleep(0.05)
                self.ser.reset_input_buffer()
                self.conn_btn.config(text="Disconnect")
                self.conn_status.config(text="● Online", fg=ACCENT_GREEN)
                self._log(f"Connected to {self.port_var.get()}", "SYS")
            except Exception as e:
                messagebox.showerror("Error", str(e))

    def _check_conn(self):
        return self.ser and self.ser.is_open

    # ------------------------------------------------------------------ #
    #  ACTIONS — all non-blocking, just enqueue                           #
    # ------------------------------------------------------------------ #
    def _apply_ctrl(self):
        if not self._check_conn(): return
        val = 0
        if self.en_sw_var.get(): val |= CTRL_EN_SW
        if self.wdi_src_var.get(): val |= CTRL_WDI_SRC
        en = self.en_sw_var.get()
        src = 'UART' if self.wdi_src_var.get() else 'HW'
        def cb(r):
            if r is not None:
                self._log(f"Applied CTRL: EN={en}, SRC={src}", "SYS")
        self._write_reg(REG_CTRL, val, callback=cb)

    def _clear_fault(self):
        if not self._check_conn(): return
        val = CTRL_CLR_FLT
        if self.en_sw_var.get(): val |= CTRL_EN_SW
        if self.wdi_src_var.get(): val |= CTRL_WDI_SRC
        def cb(r):
            if r is not None:
                self._log("Sent Clear Fault.", "SYS")
        self._write_reg(REG_CTRL, val, callback=cb)

    def _disable_wdg(self):
        if not self._check_conn(): return
        def cb(r):
            if r is not None:
                self.en_sw_var.set(False)
                self._log("Sent Disable WDG (CTRL=0).", "SYS")
        self._write_reg(REG_CTRL, 0, callback=cb)

    def _send_kick(self):
        if not self._check_conn(): return
        self._cmd_queue.put(("kick",))

    # ------------------------------------------------------------------ #
    #  POLLING — uses a timer-based feeder, not a busy-loop thread        #
    # ------------------------------------------------------------------ #
    def _toggle_polling(self):
        if self.polling:
            self.polling = False
            self.poll_btn.config(text="▶ Start Polling")
        elif self._check_conn():
            self.polling = True
            self.poll_btn.config(text="⏸ Stop Polling")
            self._schedule_poll()

    def _schedule_poll(self):
        """Feed poll commands at 10 Hz via Tk's event loop — zero extra threads."""
        if not self.polling or not self._check_conn():
            return
            
        if not self._poll_pending:
            self._poll_pending = True
            self._cmd_queue.put(("poll",))
            
        self.root.after(100, self._schedule_poll)

    def _update_status_display(self, val):
        self._set_led(self.led_enout, bool(val & ST_ENOUT))
        # WDO is Active Low (Fault when 0). So we turn on RED when val & ST_WDO == 0
        self._set_led(self.led_wdo, not bool(val & ST_WDO))

        en_eff = bool(val & ST_EN_EFF)
        enout = bool(val & ST_ENOUT)
        fault = bool(val & ST_FAULT)

        if not en_eff:
            fsm_text, fsm_style = "DISABLE", "StatusYellow.TLabel"
        elif not enout:
            fsm_text, fsm_style = "ARMING", "StatusYellow.TLabel"
        elif fault:
            fsm_text, fsm_style = "FAULT", "StatusRed.TLabel"
        else:
            fsm_text, fsm_style = "MONITOR", "StatusGreen.TLabel"

        self.lbl_fsm.config(text=fsm_text, style=fsm_style)

        flags = [
            ("EN_EFF", val & ST_EN_EFF, "ACTIVE", "INACTIVE"),
            ("FAULT",  val & ST_FAULT,  "⚠ FAULT", "OK"),
            ("ENOUT",  val & ST_ENOUT,  "HIGH", "LOW"),
            ("WDO",    val & ST_WDO,    "HIGH (OK)", "LOW (FAULT)"),
            ("KICK_SRC", val & ST_KICK_SRC, "SW/UART", "HW/Button"),
        ]
        for key, v, on_txt, off_txt in flags:
            if v:
                st = "StatusRed.TLabel" if key == "FAULT" else "StatusGreen.TLabel"
                self.status_labels[key].config(text=on_txt, style=st)
            else:
                st = "StatusGreen.TLabel" if key == "FAULT" else ("StatusRed.TLabel" if key in ("WDO","EN_EFF") else "StatusYellow.TLabel")
                self.status_labels[key].config(text=off_txt, style=st)

    # ------------------------------------------------------------------ #
    #  AUTO-KICK — timer-based, same pattern as polling                   #
    # ------------------------------------------------------------------ #
    def _toggle_auto_kick(self):
        if self.auto_kick_var.get():
            self.kick_running = True
            self._schedule_kick()
        else:
            self.kick_running = False

    def _schedule_kick(self):
        if not self.kick_running or not self._check_conn():
            return
            
        if not self._kick_pending:
            self._kick_pending = True
            self._cmd_queue.put(("kick",))
            
        try:
            interval_ms = max(100, int(float(self.kick_interval.get()) * 1000))
        except:
            interval_ms = 1000
        self.root.after(interval_ms, self._schedule_kick)

    # ------------------------------------------------------------------ #
    #  CONFIG — enqueue sequential writes/reads                           #
    # ------------------------------------------------------------------ #
    def _write_config(self):
        if not self._check_conn(): return
        try:
            twd = int(self.reg_vars["twd"].get())
            trst = int(self.reg_vars["trst"].get())
            arm = int(self.reg_vars["arm"].get())
        except:
            messagebox.showerror("Error", "Invalid config values")
            return
        self._cmd_queue.put(("write_cfg", twd, trst, arm))

    def _read_config(self):
        if not self._check_conn(): return
        self._cmd_queue.put(("read_cfg",))
        
    def _update_cfg_ui(self, twd, trst, arm):
        if twd is not None: self.reg_vars["twd"].set(str(twd))
        if trst is not None: self.reg_vars["trst"].set(str(trst))
        if arm is not None: self.reg_vars["arm"].set(str(arm & 0xFFFF))

    def _refresh_reg_map(self):
        """Enqueue a full register map read (non-blocking)."""
        if not self._check_conn(): return
        self._cmd_queue.put(("read_regmap",))

    def _update_reg_map_ui(self, results):
        """Update the register map table with fresh values (runs on GUI thread)."""
        for i, val in enumerate(results):
            if val is not None:
                self._regmap_dec_vars[i].set(str(val))
                self._regmap_hex_vars[i].set(f"0x{val:08X}")
            else:
                self._regmap_dec_vars[i].set("Timeout")
                self._regmap_hex_vars[i].set("---")

    # ------------------------------------------------------------------ #
    #  LOG                                                                #
    # ------------------------------------------------------------------ #
    def _log(self, msg, tag="SYS"):
        self.log_box.config(state="normal")
        ts = time.strftime("%H:%M:%S")
        self.log_box.insert("end", f"[{ts}] {msg}\n", tag)
        self.log_box.see("end")
        self.log_box.config(state="disabled")

    def _clear_log(self):
        self.log_box.config(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.config(state="disabled")

if __name__ == "__main__":
    root = tk.Tk()
    app = WatchdogGUI(root)
    root.mainloop()
