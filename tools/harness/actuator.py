"""Host-side driver for the actuator, speaking the CAN protocol in docs/protocol.md.

Transport is the simulator's UDP encapsulation (16-byte datagrams). Swapping
`UdpBus` for a python-can bus is the only change needed to drive real hardware
through a USB-CAN adapter.
"""
import os
import socket
import struct
import subprocess
import threading
import time

# --- Protocol constants (mirror core/include/can_proto.h) -------------------
ID_ESTOP = 0x001
ID_STATUS = 0x080
ID_TELEM = 0x180
ID_CMD = 0x200
ID_SIM_LOAD = 0x7F0  # simulator only

MODE_IDLE, MODE_VELOCITY, MODE_POSITION = 0, 1, 2
STATE_DISABLED, STATE_ENABLED, STATE_FAULT = 0, 1, 2
FAULT_TIMEOUT, FAULT_OVERCURRENT, FAULT_ESTOP = 0x01, 0x02, 0x04
FLAG_CLEAR_FAULT = 0x01

VEL_CMD_SCALE = 1e-3   # rad/s per LSB
POS_SCALE = 1e-4       # rad per LSB
VEL_TLM_SCALE = 0.02   # rad/s per LSB
CUR_SCALE = 1e-3       # A per LSB

UDP_FRAME = struct.Struct("<IB3x8s")
CMD = struct.Struct("<BBBxi")
TELEM = struct.Struct("<ihh")
STATUS = struct.Struct("<BBBBI")


def encode_cmd(mode, setpoint, seq, flags=0):
    return CMD.pack(mode, flags, seq & 0xFF, setpoint)


def decode_telem(data):
    pos, vel, cur = TELEM.unpack(data[:8])
    return {"pos": pos * POS_SCALE, "vel": vel * VEL_TLM_SCALE, "current": cur * CUR_SCALE}


def decode_status(data):
    state, faults, mode, last_seq, uptime = STATUS.unpack(data[:8])
    return {"state": state, "faults": faults, "mode": mode, "last_seq": last_seq, "uptime_ms": uptime}


class UdpBus:
    """A CAN 'bus' made of localhost UDP sockets."""

    def __init__(self, peer_port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.1)
        self.port = self.sock.getsockname()[1]
        self.peer = ("127.0.0.1", peer_port)

    def send(self, can_id, data, dlc=None):
        data = bytes(data)
        dlc = len(data) if dlc is None else dlc
        self.sock.sendto(UDP_FRAME.pack(can_id, dlc, data.ljust(8, b"\0")), self.peer)

    def recv(self):
        try:
            buf = self.sock.recv(64)
        except socket.timeout:
            return None
        if len(buf) != UDP_FRAME.size:
            return None
        can_id, dlc, data = UDP_FRAME.unpack(buf)
        return can_id, dlc, data[:dlc]

    def close(self):
        self.sock.close()


def _free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def _host_sim_cmd(node, rx_port, tx_port):
    binary = os.environ.get("ACTUATOR_SIM", os.path.join(REPO_ROOT, "build", "actuator_sim"))
    return [binary, "--node", str(node), "--rx-port", str(rx_port), "--tx-port", str(tx_port)]


def _renode_cmd(node, rx_port, tx_port):
    """Run the real STM32 firmware image in Renode, CAN bridged to UDP."""
    if node != 1:
        raise ValueError("firmware-sim.elf is built for node 1 (BOARD_NODE_ID)")
    renode = os.environ.get("RENODE", "renode")
    elf = os.path.abspath(os.environ.get("ACTUATOR_ELF", os.path.join(REPO_ROOT, "build", "stm32", "firmware-sim.elf")))
    uart_log = os.path.join(REPO_ROOT, "build", "stm32", f"uart-{rx_port}.log")
    # Renode resolves relative paths against its own directory, so pass absolute ones.
    cmds = [
        f"$elf=@{elf}",
        f"$rx_port={rx_port}",
        f"$tx_port={tx_port}",
        f"$uart_log=@{uart_log}",
        f"include @{os.path.join(REPO_ROOT, 'renode', 'actuator.resc')}",
    ]
    args = [renode, "--disable-gui", "--plain", "-P", "-1"]
    for c in cmds:
        args += ["-e", c]
    return args


class Actuator:
    """Starts a device under test and talks to it like a host controller would.

    ACTUATOR_BACKEND=host   (default) the native simulator, build/actuator_sim
    ACTUATOR_BACKEND=renode the STM32 firmware image running in Renode
    """

    def __init__(self, node=1):
        self.node = node
        self.backend = os.environ.get("ACTUATOR_BACKEND", "host")
        sim_port = _free_udp_port()
        self.bus = UdpBus(sim_port)
        make_cmd = _renode_cmd if self.backend == "renode" else _host_sim_cmd
        self.proc = subprocess.Popen(
            make_cmd(node, sim_port, self.bus.port),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self._lock = threading.Lock()
        self._running = True
        self.telem = None
        self.status = None
        self.telem_log = []  # (t, telem dict)
        self.foreign_frames = 0
        self._seq = 0
        self._hold = None
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx.start()
        self._tx = threading.Thread(target=self._hold_loop, daemon=True)
        self._tx.start()
        # The device is up once it reports status (10 Hz). Renode needs a few
        # seconds to start .NET, compile the UDP bridge and boot the firmware.
        try:
            self.wait_status(lambda s: True, timeout=60.0 if self.backend == "renode" else 3.0)
        except AssertionError:
            self.close()
            raise

    # --- receive -----------------------------------------------------------
    def _rx_loop(self):
        while self._running:
            frame = self.bus.recv()
            if frame is None:
                continue
            can_id, dlc, data = frame
            now = time.monotonic()
            with self._lock:
                if can_id == ID_TELEM + self.node and dlc == 8:
                    self.telem = decode_telem(data)
                    self.telem_log.append((now, self.telem))
                elif can_id == ID_STATUS + self.node and dlc == 8:
                    self.status = decode_status(data)
                else:
                    self.foreign_frames += 1

    def wait_status(self, pred, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                s = self.status
            if s is not None and pred(s):
                return s
            time.sleep(0.01)
        raise AssertionError(f"status condition not met within {timeout}s; last status: {self.status}")

    def wait_telem(self, pred, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                t = self.telem
            if t is not None and pred(t):
                return t
            time.sleep(0.01)
        raise AssertionError(f"telemetry condition not met within {timeout}s; last telemetry: {self.telem}")

    def telem_window(self, seconds):
        """Collect telemetry for `seconds` and return the samples."""
        start = time.monotonic()
        time.sleep(seconds)
        with self._lock:
            return [t for (ts, t) in self.telem_log if ts >= start]

    # --- transmit ----------------------------------------------------------
    def command(self, mode, setpoint=0, flags=0, node=None):
        self._seq = (self._seq + 1) & 0xFF
        node = self.node if node is None else node
        self.bus.send(ID_CMD + node, encode_cmd(mode, setpoint, self._seq, flags))
        return self._seq

    def _hold_loop(self):
        # Commands double as a heartbeat: re-send the held command at 50 Hz.
        while self._running:
            hold = self._hold
            if hold is not None:
                self.command(*hold)
            time.sleep(0.02)

    def hold_velocity(self, rad_s):
        self._hold = (MODE_VELOCITY, round(rad_s / VEL_CMD_SCALE))

    def hold_position(self, rad):
        self._hold = (MODE_POSITION, round(rad / POS_SCALE))

    def release(self):
        """Stop sending commands (the node's watchdog will notice)."""
        self._hold = None
        time.sleep(0.03)  # let an in-flight send finish

    def idle(self):
        self.release()
        return self.command(MODE_IDLE)

    def clear_fault(self):
        self.release()
        return self.command(MODE_IDLE, flags=FLAG_CLEAR_FAULT)

    def estop(self):
        self.bus.send(ID_ESTOP, b"")

    def set_load(self, newton_m):
        self.bus.send(ID_SIM_LOAD, struct.pack("<i", round(newton_m / 1e-4)))

    def close(self):
        self._hold = None
        self._running = False
        self.proc.terminate()
        try:
            self.proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
        self._rx.join(timeout=1)
        self._tx.join(timeout=1)
        self.bus.close()
