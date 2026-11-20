"""Black-box tests: drive the firmware over CAN exactly as a robot's main
computer would, with the plant simulated. Each test gets a fresh process.

Run with `make hil` (builds the simulator first)."""
import statistics
import time
import unittest

from actuator import (
    FAULT_ESTOP,
    FAULT_TIMEOUT,
    ID_CMD,
    MODE_POSITION,
    MODE_VELOCITY,
    STATE_DISABLED,
    STATE_ENABLED,
    STATE_FAULT,
    Actuator,
    encode_cmd,
)


class ActuatorHilTest(unittest.TestCase):
    def setUp(self):
        self.act = Actuator(node=1)

    def tearDown(self):
        self.act.close()

    def test_boots_disabled_and_reports_status(self):
        s1 = self.act.wait_status(lambda s: True, 1.0)
        self.assertEqual(s1["state"], STATE_DISABLED)
        self.assertEqual(s1["faults"], 0)
        s2 = self.act.wait_status(lambda s: s["uptime_ms"] > s1["uptime_ms"], 1.0)
        self.assertGreater(s2["uptime_ms"], s1["uptime_ms"])

    def test_velocity_tracking(self):
        self.act.hold_velocity(100.0)
        self.act.wait_status(lambda s: s["state"] == STATE_ENABLED, 0.5)
        time.sleep(0.5)
        vels = [t["vel"] for t in self.act.telem_window(0.3)]
        self.assertGreater(len(vels), 20)
        self.assertAlmostEqual(statistics.mean(vels), 100.0, delta=2.0)
        self.assertLess(max(abs(v - 100.0) for v in vels), 5.0)

    def test_position_move(self):
        self.act.hold_position(3.0)
        self.act.wait_telem(lambda t: abs(t["pos"] - 3.0) < 0.01 and abs(t["vel"]) < 1.0, 2.0)
        samples = self.act.telem_window(0.2)
        self.assertTrue(all(abs(t["pos"] - 3.0) < 0.01 for t in samples))

    def test_command_timeout_latches_fault(self):
        self.act.hold_velocity(50.0)
        self.act.wait_telem(lambda t: t["vel"] > 45.0, 1.0)
        self.act.release()  # host goes silent
        s = self.act.wait_status(lambda s: s["state"] == STATE_FAULT, 0.5)
        self.assertTrue(s["faults"] & FAULT_TIMEOUT)
        # Motor is no longer driven: it coasts down (friction only, so slowly).
        t0 = self.act.telem["vel"]
        time.sleep(0.3)
        self.assertLess(self.act.telem["vel"], t0)

    def test_estop_then_recover(self):
        self.act.hold_velocity(50.0)
        self.act.wait_telem(lambda t: t["vel"] > 45.0, 1.0)
        self.act.estop()
        s = self.act.wait_status(lambda s: s["state"] == STATE_FAULT, 0.3)
        self.assertTrue(s["faults"] & FAULT_ESTOP)
        # Still sending velocity commands, but the fault is latched.
        time.sleep(0.3)
        self.assertEqual(self.act.status["state"], STATE_FAULT)

        self.act.clear_fault()
        self.act.wait_status(lambda s: s["state"] == STATE_DISABLED and s["faults"] == 0, 0.5)
        self.act.hold_velocity(-80.0)
        self.act.wait_telem(lambda t: abs(t["vel"] + 80.0) < 2.0, 1.5)

    def test_rejects_load_disturbance(self):
        self.act.hold_velocity(100.0)
        self.act.wait_telem(lambda t: abs(t["vel"] - 100.0) < 2.0, 1.0)
        self.act.set_load(0.2)  # ~4 A of load torque
        time.sleep(0.6)
        vels = [t["vel"] for t in self.act.telem_window(0.3)]
        self.assertAlmostEqual(statistics.mean(vels), 100.0, delta=3.0)
        currents = [t["current"] for t in self.act.telem_window(0.1)]
        self.assertGreater(statistics.mean(currents), 3.0)  # it's working harder

    def test_ignores_other_nodes(self):
        for _ in range(15):
            self.act.command(MODE_VELOCITY, 100_000, node=2)
            time.sleep(0.02)
        self.assertEqual(self.act.status["state"], STATE_DISABLED)
        self.assertLess(abs(self.act.telem["vel"]), 0.5)

    def test_malformed_commands_are_dropped(self):
        bad_mode = bytearray(encode_cmd(MODE_VELOCITY, 100_000, 1))
        bad_mode[0] = 7
        short = encode_cmd(MODE_VELOCITY, 100_000, 2)[:4]
        for _ in range(10):
            self.act.bus.send(ID_CMD + 1, bad_mode)
            self.act.bus.send(ID_CMD + 1, short)
            time.sleep(0.02)
        self.assertEqual(self.act.status["state"], STATE_DISABLED)

    def test_status_echoes_last_sequence_number(self):
        seq = self.act.command(MODE_POSITION, 0)
        s = self.act.wait_status(lambda s: s["last_seq"] == seq, 0.5)
        self.assertEqual(s["mode"], MODE_POSITION)


if __name__ == "__main__":
    unittest.main()
