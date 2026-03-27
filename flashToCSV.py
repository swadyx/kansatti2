#!/usr/bin/env python3

import csv
import queue
import threading
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: pyserial\n\n"
        "Install it with:\n"
        "    pip install pyserial\n"
    ) from exc


BAUDRATE = 115200
READ_TIMEOUT = 0.1


class SerialWorker:
    def __init__(self, line_callback, status_callback):
        self.ser = None
        self.thread = None
        self.stop_event = threading.Event()
        self.line_callback = line_callback
        self.status_callback = status_callback

    def connect(self, port, baudrate=BAUDRATE):
        if self.ser and self.ser.is_open:
            self.disconnect()

        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=READ_TIMEOUT)

        # Many boards reset when the serial port opens.
        time.sleep(2.0)

        self.stop_event.clear()
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()
        self.status_callback(f"Connected to {port} @ {baudrate} baud")

    def disconnect(self):
        self.stop_event.set()

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass

        self.status_callback("Disconnected")

    def write_line(self, text):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port is not open")

        payload = (text.strip() + "\n").encode("utf-8", errors="replace")
        self.ser.write(payload)
        self.ser.flush()

    def _reader_loop(self):
        partial = b""

        while not self.stop_event.is_set():
            try:
                if not self.ser or not self.ser.is_open:
                    break

                chunk = self.ser.read(4096)
                if not chunk:
                    continue

                partial += chunk

                while b"\n" in partial:
                    raw_line, partial = partial.split(b"\n", 1)
                    line = raw_line.decode("utf-8", errors="replace").rstrip("\r")
                    self.line_callback(line)

            except Exception as exc:
                self.status_callback(f"Serial read error: {exc}")
                break


class FlightDataApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Flight Data Tool")
        self.geometry("980x680")

        self.serial_worker = SerialWorker(
            self._on_serial_line_thread,
            self._on_status_thread
        )
        self.ui_queue = queue.Queue()

        self.mode = None  # None, "dump", "erase"
        self.csv_output_path = None
        self.csv_file = None
        self.csv_writer = None
        self.csv_rows_written = 0

        self._build_ui()
        self._refresh_ports()

        self.after(50, self._process_ui_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="Port:").grid(row=0, column=0, sticky="w")

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            top,
            textvariable=self.port_var,
            width=28,
            state="readonly"
        )
        self.port_combo.grid(row=0, column=1, sticky="w", padx=(6, 10))

        ttk.Button(top, text="Refresh", command=self._refresh_ports).grid(
            row=0, column=2, padx=(0, 10)
        )

        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=(0, 10))

        ttk.Label(top, text="Baud:").grid(row=0, column=4, sticky="w")

        self.baud_var = tk.StringVar(value=str(BAUDRATE))
        self.baud_entry = ttk.Entry(top, textvariable=self.baud_var, width=10)
        self.baud_entry.grid(row=0, column=5, sticky="w")

        action = ttk.LabelFrame(self, text="Actions", padding=10)
        action.pack(fill="x", padx=10, pady=(0, 10))

        self.dump_btn = ttk.Button(
            action,
            text="Dump to CSV",
            command=self.start_dump,
            state="disabled"
        )
        self.dump_btn.grid(row=0, column=0, padx=(0, 10), pady=2)

        self.erase_btn = ttk.Button(
            action,
            text="Erase Saved Log",
            command=self.start_erase,
            state="disabled"
        )
        self.erase_btn.grid(row=0, column=1, padx=(0, 10), pady=2)

        self.send_ack_btn = ttk.Button(
            action,
            text="Send ACK",
            command=lambda: self._send_manual("ACK"),
            state="disabled"
        )
        self.send_ack_btn.grid(row=0, column=2, padx=(0, 10), pady=2)

        self.send_nack_btn = ttk.Button(
            action,
            text="Send NACK",
            command=lambda: self._send_manual("NACK"),
            state="disabled"
        )
        self.send_nack_btn.grid(row=0, column=3, padx=(0, 10), pady=2)

        self.send_y_btn = ttk.Button(
            action,
            text="Send y",
            command=lambda: self._send_manual("y"),
            state="disabled"
        )
        self.send_y_btn.grid(row=0, column=4, padx=(0, 10), pady=2)

        self.send_n_btn = ttk.Button(
            action,
            text="Send n",
            command=lambda: self._send_manual("n"),
            state="disabled"
        )
        self.send_n_btn.grid(row=0, column=5, padx=(0, 10), pady=2)

        status_frame = ttk.Frame(self, padding=(10, 0, 10, 10))
        status_frame.pack(fill="x")

        self.status_var = tk.StringVar(value="Not connected")
        ttk.Label(status_frame, textvariable=self.status_var).pack(anchor="w")

        summary_frame = ttk.Frame(self, padding=(10, 0, 10, 10))
        summary_frame.pack(fill="x")

        self.summary_var = tk.StringVar(value="Rows written: 0")
        ttk.Label(summary_frame, textvariable=self.summary_var).pack(anchor="w")

        console_frame = ttk.LabelFrame(self, text="Serial Console", padding=10)
        console_frame.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        self.console = tk.Text(console_frame, wrap="none", height=30)
        self.console.pack(side="left", fill="both", expand=True)

        yscroll = ttk.Scrollbar(console_frame, orient="vertical", command=self.console.yview)
        yscroll.pack(side="right", fill="y")

        self.console.configure(yscrollcommand=yscroll.set)

    def _set_connected_ui(self, connected):
        self.connect_btn.configure(text="Disconnect" if connected else "Connect")
        state = "normal" if connected else "disabled"

        for btn in [
            self.dump_btn,
            self.erase_btn,
            self.send_ack_btn,
            self.send_nack_btn,
            self.send_y_btn,
            self.send_n_btn,
        ]:
            btn.configure(state=state)

    def _append_console(self, text):
        self.console.insert("end", text + "\n")
        self.console.see("end")

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports

        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set("")

    def _toggle_connection(self):
        if self.serial_worker.ser and self.serial_worker.ser.is_open:
            self.serial_worker.disconnect()
            self._set_connected_ui(False)
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("No port selected", "Select a serial port first.")
            return

        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid baud", "Baud rate must be an integer.")
            return

        try:
            self.serial_worker.connect(port, baud)
            self._set_connected_ui(True)
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc))
            self._set_connected_ui(False)

    def _send_manual(self, text):
        try:
            self.serial_worker.write_line(text)
            self._append_console(f">>> {text}")
        except Exception as exc:
            messagebox.showerror("Send failed", str(exc))

    def start_dump(self):
        if not (self.serial_worker.ser and self.serial_worker.ser.is_open):
            messagebox.showerror("Not connected", "Connect to the board first.")
            return

        default_name = f"flight_dump_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

        path = filedialog.asksaveasfilename(
            title="Save CSV As",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )

        if not path:
            return

        try:
            self.csv_output_path = Path(path)
            self.csv_file = open(self.csv_output_path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_rows_written = 0
            self.mode = "dump"
            self.summary_var.set("Rows written: 0")
        except Exception as exc:
            messagebox.showerror("File error", f"Could not open output file:\n{exc}")
            self._cleanup_csv()
            return

        self._append_console("=== Starting DUMP workflow ===")
        self._append_console("Waiting for sketch prompts. The app will answer automatically.")
        self.status_var.set(f"Dump armed. Output file: {self.csv_output_path}")

    def start_erase(self):
        if not (self.serial_worker.ser and self.serial_worker.ser.is_open):
            messagebox.showerror("Not connected", "Connect to the board first.")
            return

        ok = messagebox.askyesno(
            "Confirm erase",
            "This will tell the board to erase the saved flight log.\n\nContinue?"
        )
        if not ok:
            return

        self.mode = "erase"
        self._append_console("=== Starting ERASE workflow ===")
        self._append_console("Waiting for sketch prompts. The app will answer automatically.")
        self.status_var.set("Erase armed")

    def _cleanup_csv(self):
        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass

        self.csv_file = None
        self.csv_writer = None

    def _handle_line(self, line):
        self._append_console(line)

        stripped = line.strip()
        lower = stripped.lower()

        if stripped.startswith("t_ms,state,temp_C,pressure_Pa,"):
            if self.mode == "dump" and self.csv_writer:
                header = [x.strip() for x in stripped.split(",")]
                self.csv_writer.writerow(header)
                self.csv_file.flush()
            return

        if self._looks_like_csv_row(stripped):
            if self.mode == "dump" and self.csv_writer:
                row = [x.strip() for x in stripped.split(",")]
                self.csv_writer.writerow(row)
                self.csv_rows_written += 1
                self.summary_var.set(f"Rows written: {self.csv_rows_written}")
                self.csv_file.flush()
                self._safe_send("ACK")
            return

        if "# Done" in stripped or "Done (reached end page)" in stripped:
            if self.mode == "dump":
                self.status_var.set(
                    f"Dump complete. Saved {self.csv_rows_written} rows to {self.csv_output_path}"
                )
                self._cleanup_csv()
                self.mode = None
            return

        if "Erase complete." in stripped:
            if self.mode == "erase":
                self.status_var.set("Erase complete")
                self.mode = None
            return

        if self.mode == "dump":
            self._handle_dump_prompts(lower)
        elif self.mode == "erase":
            self._handle_erase_prompts(lower)

    def _handle_dump_prompts(self, lower):
        if "first confirmation:" in lower:
            self._safe_send("n")
            return

        if "second confirmation:" in lower:
            self._safe_send("n")
            return

        if "third confirmation:" in lower:
            self._safe_send("n")
            return

        if "dump saved flight data as csv?" in lower:
            self._safe_send("y")
            return

        if "type ack or nack" in lower:
            self._safe_send("ACK")
            return

    def _handle_erase_prompts(self, lower):
        if "first confirmation:" in lower:
            self._safe_send("y")
            return

        if "second confirmation:" in lower:
            self._safe_send("y")
            return

        if "third confirmation:" in lower:
            self._safe_send("y")
            return

        if "dump saved flight data as csv?" in lower:
            self._safe_send("n")
            return

    def _looks_like_csv_row(self, line):
        if line.count(",") < 10:
            return False

        first = line.split(",", 1)[0].strip()
        return first.isdigit()

    def _safe_send(self, text):
        try:
            self.serial_worker.write_line(text)
            self._append_console(f">>> {text}")
        except Exception as exc:
            self.status_var.set(f"Send failed: {exc}")

    def _on_serial_line_thread(self, line):
        self.ui_queue.put(("line", line))

    def _on_status_thread(self, status):
        self.ui_queue.put(("status", status))

    def _process_ui_queue(self):
        try:
            while True:
                kind, payload = self.ui_queue.get_nowait()

                if kind == "line":
                    self._handle_line(payload)
                elif kind == "status":
                    self.status_var.set(payload)
                    self._append_console(f"[status] {payload}")
                    connected = bool(self.serial_worker.ser and self.serial_worker.ser.is_open)
                    self._set_connected_ui(connected)

        except queue.Empty:
            pass

        self.after(50, self._process_ui_queue)

    def _on_close(self):
        try:
            self.serial_worker.disconnect()
        except Exception:
            pass

        self._cleanup_csv()
        self.destroy()


if __name__ == "__main__":
    app = FlightDataApp()
    app.mainloop()
