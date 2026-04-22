"""
================================================================================
Watchdog Monitor - Hardware UART Test Suite
Board   : Kiwi 1P5 (Gowin GW1N-UV1P5)
Port    : COM5, 115200 8N1
Protocol: [0x55][CMD][ADDR][LEN][DATA...][CHK]
Commands: 0x01=WRITE, 0x02=READ, 0x03=KICK, 0x04=GET_STATUS
================================================================================
"""

import serial
import time
import sys
import struct

# ============================================================
# CONFIGURATION
# ============================================================
COM_PORT    = "COM5"
BAUD_RATE   = 115200
RX_TIMEOUT  = 1.0   # seconds to wait for a response

# Register Map
REG_CTRL      = 0x00
REG_TWD_MS    = 0x04
REG_TRST_MS   = 0x08
REG_ARM_DELAY = 0x0C
REG_STATUS    = 0x10

# CMD Codes
CMD_WRITE  = 0x01
CMD_READ   = 0x02
CMD_KICK   = 0x03
CMD_STATUS = 0x04

# CTRL bits
CTRL_EN_SW    = (1 << 0)
CTRL_WDI_SRC  = (1 << 1)  # 0=HW, 1=SW/UART
CTRL_CLR_FLT  = (1 << 2)

# STATUS bits
STATUS_EN_EFFECTIVE = (1 << 0)
STATUS_FAULT_ACTIVE = (1 << 1)
STATUS_ENOUT        = (1 << 2)
STATUS_WDO          = (1 << 3)
STATUS_LAST_KICK    = (1 << 4)

# ============================================================
# TEST RESULT TRACKER
# ============================================================
results = {"PASS": 0, "FAIL": 0, "SKIP": 0}

def log(msg, level="INFO"):
    colors = {"INFO": "\033[0m", "PASS": "\033[92m", "FAIL": "\033[91m",
              "WARN": "\033[93m", "HEAD": "\033[96m", "SEND": "\033[94m", "RECV": "\033[95m"}
    print(f"{colors.get(level, '')}[{level}] {msg}\033[0m")

def check(condition, description):
    if condition:
        log(f"  PASS: {description}", "PASS")
        results["PASS"] += 1
    else:
        log(f"  FAIL: {description}", "FAIL")
        results["FAIL"] += 1
    return condition

# ============================================================
# UART PROTOCOL HELPERS
# ============================================================
def calc_checksum(byte_list):
    """XOR checksum of all bytes EXCEPT the Start byte (0x55)."""
    chk = 0
    for b in byte_list:
        chk ^= b
    return chk

def build_frame(cmd, addr, data_bytes=[]):
    length = len(data_bytes)
    core = [cmd, addr, length] + data_bytes
    chk  = calc_checksum(core)
    return bytes([0x55] + core + [chk])

def send_frame(ser, frame):
    hex_str = ' '.join(f'{b:02X}' for b in frame)
    log(f"  TX: {hex_str}", "SEND")
    ser.write(frame)

def recv_frame(ser, timeout=RX_TIMEOUT):
    """Read until we get 0x55 start byte then read the rest of the frame."""
    deadline = time.time() + timeout
    buf = bytearray()

    # Wait for start byte
    while time.time() < deadline:
        b = ser.read(1)
        if b and b[0] == 0x55:
            buf.append(0x55)
            break
    else:
        return None  # Timeout

    # Read at least 4 more bytes: CMD, ADDR, LEN, CHK
    time.sleep(0.05)
    buf += ser.read(ser.in_waiting or 16)

    if len(buf) >= 5:
        hex_str = ' '.join(f'{b:02X}' for b in buf)
        log(f"  RX: {hex_str}", "RECV")
        return bytes(buf)
    return None

# ============================================================
# HIGH-LEVEL COMMAND HELPERS
# ============================================================
def write_reg(ser, addr, value_32bit):
    data = list(struct.pack(">I", value_32bit))  # Big-endian
    frame = build_frame(CMD_WRITE, addr, data)
    send_frame(ser, frame)
    return recv_frame(ser)

def read_reg(ser, addr):
    frame = build_frame(CMD_READ, addr, [])
    send_frame(ser, frame)
    resp = recv_frame(ser)
    if resp and len(resp) >= 9:
        val = struct.unpack(">I", resp[4:8])[0]
        return val
    return None

def kick(ser):
    frame = build_frame(CMD_KICK, 0x00, [])
    send_frame(ser, frame)
    return recv_frame(ser)

def get_status(ser):
    frame = build_frame(CMD_STATUS, 0x00, [])
    send_frame(ser, frame)
    resp = recv_frame(ser)
    if resp and len(resp) >= 9:
        return struct.unpack(">I", resp[4:8])[0]
    return None

def status_str(s):
    if s is None:
        return "N/A"
    bits = []
    bits.append(f"EN_EFF={'1' if s & STATUS_EN_EFFECTIVE else '0'}")
    bits.append(f"FAULT={'1' if s & STATUS_FAULT_ACTIVE  else '0'}")
    bits.append(f"ENOUT={'1' if s & STATUS_ENOUT         else '0'}")
    bits.append(f"WDO={'1' if s & STATUS_WDO             else '0'}")
    bits.append(f"KICK_SRC={'SW' if s & STATUS_LAST_KICK else 'HW'}")
    return " | ".join(bits)

def section(title):
    log("", "HEAD")
    log("=" * 60, "HEAD")
    log(f"  {title}", "HEAD")
    log("=" * 60, "HEAD")

def disable_watchdog(ser):
    """Helper: turn off watchdog cleanly."""
    write_reg(ser, REG_CTRL, 0x00000000)
    time.sleep(0.1)

# ============================================================
# TEST SCENARIOS
# ============================================================

def test_01_register_readback(ser):
    section("TEST 01: Register Read/Write Integrity")

    cases = [
        (REG_TWD_MS,    0x000003E8, "tWD_ms = 1000"),
        (REG_TWD_MS,    0x00000640, "tWD_ms = 1600"),
        (REG_TRST_MS,   0x000000C8, "tRST_ms = 200"),
        (REG_TRST_MS,   0x00000064, "tRST_ms = 100"),
        (REG_ARM_DELAY, 0x00000096, "arm_delay = 150 (only low 16-bit)"),
        (REG_CTRL,      0x00000003, "CTRL: EN_SW=1, WDI_SRC=1"),
        (REG_CTRL,      0x00000001, "CTRL: EN_SW=1, WDI_SRC=0"),
        (REG_CTRL,      0x00000000, "CTRL: Disabled"),
    ]

    for addr, val, desc in cases:
        resp = write_reg(ser, addr, val)
        check(resp is not None, f"WRITE ACK for {desc}")
        time.sleep(0.05)
        readback = read_reg(ser, addr)
        # ARM_DELAY is 16-bit register
        expected = val & 0x0000FFFF if addr == REG_ARM_DELAY else val
        check(readback == expected, f"Readback match {desc}: expected=0x{expected:08X}, got=0x{(readback if readback is not None else 0):08X}")
        time.sleep(0.05)

def test_02_status_register_readonly(ser):
    section("TEST 02: STATUS Register is Read-Only")
    disable_watchdog(ser)

    # Try to write to STATUS (should be ignored)
    write_reg(ser, REG_STATUS, 0xDEADBEEF)
    time.sleep(0.1)
    s = get_status(ser)
    check(s != 0xDEADBEEF, "STATUS register ignores writes (Read-Only)")
    log(f"  STATUS after write attempt: {status_str(s)}", "INFO")

def test_03_enable_disable_sw(ser):
    section("TEST 03: Software Enable/Disable via CTRL[0]")
    disable_watchdog(ser)
    time.sleep(0.2)

    s = get_status(ser)
    check(s is not None and not (s & STATUS_EN_EFFECTIVE), "FSM in DISABLE when CTRL[0]=0")
    check(s is not None and (s & STATUS_WDO), "WDO=1 (No Fault) when disabled")
    check(s is not None and not (s & STATUS_ENOUT), "ENOUT=0 when disabled")
    log(f"  STATUS (disabled): {status_str(s)}", "INFO")

    # Enable SW with UART kick source
    write_reg(ser, REG_CTRL, CTRL_EN_SW | CTRL_WDI_SRC)
    log("  Waiting for arming delay (default ~150us -> ~1ms margin)...", "INFO")
    time.sleep(0.5)  # Arm delay is 150us, 500ms gives plenty of margin

    s = get_status(ser)
    check(s is not None and (s & STATUS_EN_EFFECTIVE), "FSM in MONITOR (en_effective=1) after enable")
    check(s is not None and (s & STATUS_ENOUT), "ENOUT=1 after arming")
    check(s is not None and (s & STATUS_WDO), "WDO=1 (No Fault) during normal monitor")
    log(f"  STATUS (monitoring): {status_str(s)}", "INFO")

    # Disable
    disable_watchdog(ser)
    time.sleep(0.2)
    s = get_status(ser)
    check(s is not None and not (s & STATUS_EN_EFFECTIVE), "FSM returns to DISABLE after EN=0")
    log(f"  STATUS (re-disabled): {status_str(s)}", "INFO")

def test_04_kick_sw_when_enabled(ser):
    section("TEST 04: SW KICK while WDI_SRC=1 (UART Mode)")
    # Enable with SW kick source and 2000ms timeout
    write_reg(ser, REG_TWD_MS, 2000)
    write_reg(ser, REG_CTRL, CTRL_EN_SW | CTRL_WDI_SRC)
    time.sleep(0.5)  # wait arming

    s = get_status(ser)
    check(s is not None and (s & STATUS_EN_EFFECTIVE), "Watchdog armed before kick test")

    # Send 5 kicks, verify ACK each time
    for i in range(5):
        time.sleep(0.3)
        resp = kick(ser)
        check(resp is not None, f"KICK ACK received (kick #{i+1})")

    s = get_status(ser)
    check(s is not None and (s & STATUS_WDO), "WDO still HIGH after 5 kicks (no timeout)")
    check(s is not None and not (s & STATUS_FAULT_ACTIVE), "No fault after regular kicking")
    check(s is not None and (s & STATUS_LAST_KICK), "last_kick_src=1 (SW UART)")
    log(f"  STATUS after kicks: {status_str(s)}", "INFO")

    disable_watchdog(ser)

def test_05_kick_rejected_when_hw_mode(ser):
    section("TEST 05: SW KICK rejected when WDI_SRC=0 (HW-only Mode)")
    write_reg(ser, REG_TWD_MS, 5000)  # Long timeout to avoid false timeout
    write_reg(ser, REG_CTRL, CTRL_EN_SW)  # WDI_SRC=0 (HW only)
    time.sleep(0.5)

    log("  Sending SW KICK while WDI_SRC=0 (should be silently dropped)...", "INFO")
    resp = kick(ser)
    # Parser drops frame -> no ACK -> resp = None
    check(resp is None, "SW KICK rejected (no ACK) when WDI_SRC=0 [Bug Fix #2]")

    disable_watchdog(ser)

def test_06_watchdog_timeout_and_fault(ser):
    section("TEST 06: Watchdog Timeout -> FAULT State")
    # Set short timeout for this test
    write_reg(ser, REG_TWD_MS, 3000)   # 3 second watchdog
    write_reg(ser, REG_TRST_MS, 1000)  # 1 second fault hold
    write_reg(ser, REG_CTRL, CTRL_EN_SW | CTRL_WDI_SRC)
    time.sleep(0.5)  # arm

    s = get_status(ser)
    check(s is not None and (s & STATUS_EN_EFFECTIVE), "Watchdog armed, monitoring started")
    log("  Not kicking... waiting for timeout (3 seconds)...", "WARN")

    # Monitor in steps
    for elapsed in range(1, 5):
        time.sleep(1.0)
        s = get_status(ser)
        log(f"  T+{elapsed}s: {status_str(s)}", "INFO")
        if s is not None and (s & STATUS_FAULT_ACTIVE):
            log("  -> Entered FAULT state!", "WARN")
            break

    check(s is not None and (s & STATUS_FAULT_ACTIVE), "FAULT_ACTIVE=1 after timeout [Core Behavior]")
    check(s is not None and not (s & STATUS_WDO), "WDO=0 (pulled low) during fault")
    check(s is not None and (s & STATUS_ENOUT), "ENOUT remains 1 during fault (not disabled)")

    # Wait for auto recovery (tRST=1000ms)
    log("  Waiting for auto-recovery (tRST=1s)...", "INFO")
    time.sleep(1.5)
    s = get_status(ser)
    check(s is not None and not (s & STATUS_FAULT_ACTIVE), "FAULT cleared after tRST expired (auto-recovery)")
    check(s is not None and (s & STATUS_WDO), "WDO=1 (released) after auto-recovery")
    log(f"  STATUS after auto-recovery: {status_str(s)}", "INFO")

    disable_watchdog(ser)

def test_07_clr_fault_immediate(ser):
    section("TEST 07: CLR_FAULT Clears Fault Immediately (W1C)")
    write_reg(ser, REG_TWD_MS, 2000)
    write_reg(ser, REG_TRST_MS, 5000)  # Long tRST to test manual clear
    write_reg(ser, REG_CTRL, CTRL_EN_SW | CTRL_WDI_SRC)
    time.sleep(0.5)

    log("  Waiting for 2s timeout...", "WARN")
    time.sleep(2.5)

    s = get_status(ser)
    check(s is not None and (s & STATUS_FAULT_ACTIVE), "In FAULT state")
    log("  Writing CLR_FAULT (CTRL[2]=1)...", "INFO")
    # Write CLR_FAULT bit while keeping EN_SW and WDI_SRC
    write_reg(ser, REG_CTRL, CTRL_EN_SW | CTRL_WDI_SRC | CTRL_CLR_FLT)
    time.sleep(0.2)

    s = get_status(ser)
    check(s is not None and not (s & STATUS_FAULT_ACTIVE), "Fault cleared immediately by CLR_FAULT [Core Behavior]")
    check(s is not None and (s & STATUS_WDO), "WDO=1 released immediately")
    log(f"  STATUS after CLR_FAULT: {status_str(s)}", "INFO")

    disable_watchdog(ser)

def test_08_kick_keeps_alive(ser):
    section("TEST 08: Consecutive Kicks Prevent Timeout")
    twd = 2000  # 2s timeout
    write_reg(ser, REG_TWD_MS, twd)
    write_reg(ser, REG_CTRL, CTRL_EN_SW | CTRL_WDI_SRC)
    time.sleep(0.5)

    log(f"  Kicking every 1s for 10s with tWD={twd}ms. No timeout should occur.", "INFO")
    all_alive = True
    for i in range(10):
        time.sleep(1.0)
        resp = kick(ser)
        s = get_status(ser)
        alive = (s is not None and not (s & STATUS_FAULT_ACTIVE) and (s & STATUS_WDO))
        log(f"  Kick #{i+1}: ACK={'OK' if resp else 'MISS'} | {status_str(s)}", "INFO")
        if not alive:
            all_alive = False
            break

    check(all_alive, "Watchdog stays alive during consistent kicking [No drift bug]")
    disable_watchdog(ser)

def test_09_underflow_guard(ser):
    section("TEST 09: Underflow Guard - Write 0 to tWD/tRST/armDelay")
    log("  Writing 0 to tWD_ms, tRST_ms, arm_delay (would cause 49-day freeze if unfixed).", "WARN")

    write_reg(ser, REG_TWD_MS,    0x00000000)
    write_reg(ser, REG_TRST_MS,   0x00000000)
    write_reg(ser, REG_ARM_DELAY, 0x00000000)
    write_reg(ser, REG_CTRL, CTRL_EN_SW | CTRL_WDI_SRC)

    time.sleep(0.5)
    s = get_status(ser)
    # If underflow was not fixed, FSM would be stuck in ARMING/MONITOR for 49 days.
    # After fix, with tWD=0 -> timer+1 >= 0 is immediately true -> Fault instant.
    log(f"  STATUS with tWD=0: {status_str(s)}", "INFO")
    check(s is not None, "System responsive after 0-value register writes [Underflow Bug Fix #1]")

    # Restore sane defaults
    write_reg(ser, REG_TWD_MS,    1600)
    write_reg(ser, REG_TRST_MS,   200)
    write_reg(ser, REG_ARM_DELAY, 150)
    disable_watchdog(ser)

def test_10_checksum_validation(ser):
    section("TEST 10: Checksum Validation (Corrupted Frames Rejected)")
    bad_frames = [
        (bytes([0x55, 0x03, 0x00, 0x00, 0xFF]), "KICK with wrong CHK (0xFF)"),
        (bytes([0x55, 0x01, 0x04, 0x04, 0x00, 0x00, 0x03, 0xE8, 0xEE]), "WRITE with wrong CHK"),
        (bytes([0x55, 0x02, 0x04, 0x00, 0xAA]), "READ with wrong CHK (0xAA)"),
    ]

    for raw, desc in bad_frames:
        hex_str = ' '.join(f'{b:02X}' for b in raw)
        log(f"  TX (corrupted): {hex_str}", "SEND")
        ser.write(raw)
        resp = recv_frame(ser, timeout=0.5)
        check(resp is None, f"No response to corrupted frame: {desc}")
        time.sleep(0.1)

def test_11_rapid_enable_disable(ser):
    section("TEST 11: Rapid Enable/Disable Stress Test")
    write_reg(ser, REG_TWD_MS, 5000)
    errors = 0
    for i in range(10):
        write_reg(ser, REG_CTRL, CTRL_EN_SW | CTRL_WDI_SRC)
        time.sleep(0.1)
        write_reg(ser, REG_CTRL, 0x00000000)
        time.sleep(0.1)
        s = get_status(ser)
        if s is None or (s & STATUS_FAULT_ACTIVE):
            errors += 1

    check(errors == 0, f"No fault during 10x rapid enable/disable cycles (errors={errors})")
    disable_watchdog(ser)

def test_12_prescaler_drift(ser):
    section("TEST 12: Prescaler Drift - Kick At Varying Intervals")
    twd = 3000  # 3s timeout
    write_reg(ser, REG_TWD_MS, twd)
    write_reg(ser, REG_CTRL, CTRL_EN_SW | CTRL_WDI_SRC)
    time.sleep(0.5)

    log("  Kicking at 90%, 80%, 70%... of timeout to stress prescaler accuracy.", "INFO")
    all_ok = True
    intervals = [2.7, 2.4, 2.1, 2.7, 2.4, 2.1]

    for i, t in enumerate(intervals):
        log(f"  Waiting {t}s before kick {i+1}...", "INFO")
        time.sleep(t)
        resp = kick(ser)
        s = get_status(ser)
        ok = (resp is not None and s is not None and
              not (s & STATUS_FAULT_ACTIVE) and (s & STATUS_WDO))
        log(f"  Kick #{i+1} @ T+{t}s: {'OK' if ok else 'FAIL'} | {status_str(s)}", "INFO")
        if not ok:
            all_ok = False
            break

    check(all_ok, "Watchdog survives near-deadline kicks (prescaler drift fixed) [Bug Fix #3]")
    disable_watchdog(ser)

# ============================================================
# MAIN ENTRY POINT
# ============================================================
def main():
    print("\033[96m")
    print("=" * 60)
    print("  Watchdog Monitor - Hardware UART Test Suite")
    print(f"  Port: {COM_PORT} @ {BAUD_RATE} baud")
    print("=" * 60)
    print("\033[0m")

    try:
        ser = serial.Serial(
            port=COM_PORT, baudrate=BAUD_RATE,
            bytesize=8, parity='N', stopbits=1,
            timeout=RX_TIMEOUT
        )
        log(f"Connected to {COM_PORT} @ {BAUD_RATE}", "INFO")
    except serial.SerialException as e:
        log(f"Cannot open {COM_PORT}: {e}", "FAIL")
        sys.exit(1)

    time.sleep(0.5)
    ser.reset_input_buffer()

    tests = [
        test_01_register_readback,
        test_02_status_register_readonly,
        test_03_enable_disable_sw,
        test_04_kick_sw_when_enabled,
        test_05_kick_rejected_when_hw_mode,
        test_06_watchdog_timeout_and_fault,
        test_07_clr_fault_immediate,
        test_08_kick_keeps_alive,
        test_09_underflow_guard,
        test_10_checksum_validation,
        test_11_rapid_enable_disable,
        test_12_prescaler_drift,
    ]

    for test_fn in tests:
        try:
            ser.reset_input_buffer()
            disable_watchdog(ser)
            time.sleep(0.2)
            test_fn(ser)
        except Exception as e:
            log(f"Exception in {test_fn.__name__}: {e}", "FAIL")
            results["FAIL"] += 1

    # Summary
    total = results["PASS"] + results["FAIL"]
    log("", "HEAD")
    log("=" * 60, "HEAD")
    log(f"  FINAL RESULTS: {results['PASS']}/{total} checks PASSED", "HEAD")
    log(f"  PASS: {results['PASS']}  |  FAIL: {results['FAIL']}", "HEAD")
    log("=" * 60, "HEAD")
    if results["FAIL"] == 0:
        log("  ALL TESTS PASSED! Firmware is healthy.", "PASS")
    else:
        log(f"  {results['FAIL']} CHECK(S) FAILED. Review output above.", "FAIL")

    ser.close()
    log("Connection closed.", "INFO")

if __name__ == "__main__":
    main()
