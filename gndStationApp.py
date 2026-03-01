import sys
import time
import queue
import threading
from dataclasses import dataclass

import pygame
import serial


def serial_reader(ser: serial.Serial, out_q: "queue.Queue[str]", stop_event: threading.Event):
    ser.timeout = 0.05
    buf = b""
    while not stop_event.is_set():
        try:
            chunk = ser.read(256)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip(b"\r")
                    if not line:
                        continue
                    try:
                        out_q.put_nowait(line.decode(errors="replace"))
                    except queue.Full:
                        pass
            else:
                time.sleep(0.005)
        except serial.SerialException:
            break


# Expected: T,<seq>,ax_g,<v>,...,thrust,<v>,m1_us,<v>,m2_us,<v>,m3_us,<v>,m4_us,<v>
def parse_telem_line(line: str):
    line = line.strip()
    if not line.startswith("T,"):
        return None
    parts = line.split(",")
    if len(parts) < 4:
        return None

    out = {"seq": parts[1].strip()}
    i = 2
    while i + 1 < len(parts):
        k = parts[i].strip()
        v = parts[i + 1].strip()
        if k:
            out[k] = v
        i += 2
    return out


@dataclass
class Button:
    label: str
    cmd: str
    rect: pygame.Rect

    def draw(self, surf, font, mouse_pos, mouse_down):
        hovered = self.rect.collidepoint(mouse_pos)
        bg = (45, 45, 50) if not hovered else (70, 70, 80)
        if mouse_down and hovered:
            bg = (95, 95, 110)
        pygame.draw.rect(surf, bg, self.rect, border_radius=10)
        pygame.draw.rect(surf, (120, 120, 140), self.rect, width=2, border_radius=10)

        text = font.render(self.label, True, (235, 235, 240))
        surf.blit(
            text,
            (self.rect.centerx - text.get_width() // 2,
             self.rect.centery - text.get_height() // 2),
        )

    def hit(self, pos) -> bool:
        return self.rect.collidepoint(pos)


def draw_tile(surf, rect, title, unit, value, font_title, font_unit, font_value):
    pygame.draw.rect(surf, (26, 26, 30), rect, border_radius=12)
    pygame.draw.rect(surf, (80, 80, 95), rect, width=2, border_radius=12)

    t = font_title.render(title, True, (220, 220, 235))
    u = font_unit.render(unit, True, (170, 170, 185))
    v = font_value.render(value, True, (245, 245, 250))

    surf.blit(t, (rect.x + 10, rect.y + 8))
    if unit:
        surf.blit(u, (rect.x + 10 + t.get_width() + 10, rect.y + 10))
    surf.blit(v, (rect.x + 10, rect.y + 34))


def main():
    if len(sys.argv) < 2:
        print("Usage: python ground_ui.py <COM_PORT> [BAUD]")
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) >= 3 else 115200

    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except serial.SerialException as e:
        print(f"Could not open serial {port}: {e}")
        sys.exit(1)

    def send_cmd(cmd: str):
        try:
            ser.write((cmd + "\n").encode())
        except serial.SerialException:
            pass

    q_lines: queue.Queue[str] = queue.Queue(maxsize=400)
    stop_event = threading.Event()
    t = threading.Thread(target=serial_reader, args=(ser, q_lines, stop_event), daemon=True)
    t.start()

    pygame.init()
    pygame.display.set_caption("LoRa Ground UI")
    W, H = 1250, 760
    screen = pygame.display.set_mode((W, H))
    clock = pygame.time.Clock()

    font_btn = pygame.font.SysFont("consolas", 20)
    font_title = pygame.font.SysFont("consolas", 30, bold=True)
    font_tile_title = pygame.font.SysFont("consolas", 18, bold=True)
    font_tile_unit = pygame.font.SysFont("consolas", 16)
    font_tile_value = pygame.font.SysFont("consolas", 28, bold=True)
    font_small = pygame.font.SysFont("consolas", 16)

    # Buttons
    bx, by = 20, 20
    bw, bh = 180, 55
    gap = 12

    buttons = []

    def add_button(label, cmd, row, col):
        x = bx + col * (bw + gap)
        y = by + row * (bh + gap)
        buttons.append(Button(label, cmd, pygame.Rect(x, y, bw, bh)))

    add_button("LED ON", "led_on", 0, 0)
    add_button("LED OFF", "led_off", 0, 1)
    add_button("BLINK FAST", "blink_fast", 1, 0)
    add_button("BLINK SLOW", "blink_slow", 1, 1)

    add_button("FLY (ARM)", "fly", 0, 2)
    add_button("STOP (SPACE)", "stop", 1, 2)

    add_button("THRUST + (↑)", "thrust_up", 0, 3)
    add_button("THRUST - (↓)", "thrust_down", 1, 3)

    # Field definitions: (key_in_serial, display_name, unit)
    FIELDS = [
        ("seq",     "seq",     ""),
        ("ax_g",    "ax",      "g"),
        ("ay_g",    "ay",      "g"),
        ("az_g",    "az",      "g"),
        ("gx_dps",  "gx",      "°/s"),
        ("gy_dps",  "gy",      "°/s"),
        ("gz_dps",  "gz",      "°/s"),
        ("roll",    "roll",    "°"),
        ("pitch",   "pitch",   "°"),
        ("yaw",     "yaw",     "°"),
        ("lipo_V",  "lipo",    "V"),
        ("thrust",  "thrust",  "0–1"),
        # NEW motor commanded PWM (microseconds)
        ("m1_us",   "m1",      "µs"),
        ("m2_us",   "m2",      "µs"),
        ("m3_us",   "m3",      "µs"),
        ("m4_us",   "m4",      "µs"),
    ]

    latest = {k: "--" for k, _, _ in FIELDS}
    latest_raw = ""
    last_telem_ms = 0

    running = True
    while running:
        mouse_pos = pygame.mouse.get_pos()
        mouse_down = pygame.mouse.get_pressed(num_buttons=3)[0]

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                for b in buttons:
                    if b.hit(event.pos):
                        send_cmd(b.cmd)
                        break

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    send_cmd("stop")
                elif event.key == pygame.K_UP:
                    send_cmd("thrust_up")
                elif event.key == pygame.K_DOWN:
                    send_cmd("thrust_down")
                elif event.key == pygame.K_ESCAPE:
                    running = False

        try:
            while True:
                line = q_lines.get_nowait()
                latest_raw = line
                parsed = parse_telem_line(line)
                if parsed:
                    for k, _, _ in FIELDS:
                        if k in parsed:
                            latest[k] = parsed[k]
                    last_telem_ms = pygame.time.get_ticks()
        except queue.Empty:
            pass

        screen.fill((18, 18, 20))

        title = font_title.render("Ground Station UI", True, (240, 240, 245))
        screen.blit(title, (20, 150))

        for b in buttons:
            b.draw(screen, font_btn, mouse_pos, mouse_down)

        age_ms = pygame.time.get_ticks() - last_telem_ms if last_telem_ms else 999999
        screen.blit(font_small.render(f"Serial: {port} @ {baud}   Telemetry age: {age_ms} ms",
                                      True, (180, 180, 195)), (20, 190))
        screen.blit(font_small.render("Keys: SPACE=STOP, ↑/↓=thrust, ESC=quit",
                                      True, (160, 160, 175)), (20, 210))

        panel = pygame.Rect(20, 240, W - 40, H - 260)
        pygame.draw.rect(screen, (22, 22, 25), panel, border_radius=16)
        pygame.draw.rect(screen, (70, 70, 85), panel, width=2, border_radius=16)

        cols = 4
        tile_w = (panel.width - 20 - (cols - 1) * 14) // cols
        tile_h = 90
        start_x = panel.x + 10
        start_y = panel.y + 10

        for idx, (key, disp, unit) in enumerate(FIELDS):
            r = idx // cols
            c = idx % cols
            x = start_x + c * (tile_w + 14)
            y = start_y + r * (tile_h + 14)
            rect = pygame.Rect(x, y, tile_w, tile_h)
            draw_tile(screen, rect, disp, f"({unit})" if unit else "", str(latest.get(key, "--")),
                      font_tile_title, font_tile_unit, font_tile_value)

        raw_area = pygame.Rect(panel.x + 10, panel.bottom - 50, panel.width - 20, 40)
        pygame.draw.rect(screen, (26, 26, 30), raw_area, border_radius=10)
        pygame.draw.rect(screen, (80, 80, 95), raw_area, width=2, border_radius=10)
        raw_txt = (latest_raw.strip()[:180] if latest_raw else "(waiting for serial...)")
        screen.blit(font_small.render("raw:", True, (170, 170, 185)), (raw_area.x + 10, raw_area.y + 10))
        screen.blit(font_small.render(raw_txt, True, (200, 200, 210)), (raw_area.x + 55, raw_area.y + 10))

        pygame.display.flip()
        clock.tick(60)

    stop_event.set()
    time.sleep(0.05)
    try:
        ser.close()
    except Exception:
        pass
    pygame.quit()


if __name__ == "__main__":
    main()
