#!/usr/bin/env python3
# Test an toàn bộ fuzzy trên KHỚP 5 (wrist_rotate) — và chỉ khớp đó.
# - Chỉ wrist_rotate chuyển sang PWM; 4 khớp còn lại GIỮ position mode (firmware giữ trọng lực).
# - Luật fuzzy = đúng fuzzy_type1.c của project (qua ctypes) -> cùng mặt phẳng điều khiển với node.
# - Pha 1: giữ pose gốc (regulation). Pha 2: bước tham chiếu +STEP (tracking). Cuối: zero + torque off + trả position mode.
import ctypes, csv, os, math, time, signal, datetime
import matplotlib
matplotlib.use("Agg")           # lưu PNG, không bật cửa sổ khi chạy
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.srv import OperatingModes, TorqueEnable

JOINT     = "wrist_rotate"
KE        = 0.2
KED       = 0.0005
KU        = 700.0
UMAX      = 600.0          # cap an toàn wrist_rotate (hardware ±885)
LOOP_HZ   = 100.0
STEP      = 2.0            # rad, lệch tham chiếu ở pha 2 (~115°)
HOLD_S    = 3.0            # pha 1: giữ pose gốc
RUN_S     = 16.0           # tổng thời gian chạy
WATCHDOG  = 0.2            # s: nếu joint_states cũ hơn -> zero PWM
LIM_LO, LIM_HI = -3.0, 3.0 # giới hạn mềm wrist_rotate (rad)
RUNS_DIR  = "runs"          # thư mục lưu CSV + PNG mỗi lần chạy (A/B tuning)

class Stop(Exception): pass

class FuzzyJoint5(Node):
    def __init__(self):
        super().__init__("fuzzy_joint5_test")
        self.lib = ctypes.CDLL("/tmp/fuzzy_type1.so")
        self.f = self.lib.fuzzy_type1_eval
        self.f.restype = ctypes.c_float
        self.f.argtypes = [ctypes.c_float, ctypes.c_float]

        self.cli_mode = self.create_client(OperatingModes, "/rx150/set_operating_modes")
        self.cli_tq = self.create_client(TorqueEnable, "/rx150/torque_enable")
        for c in (self.cli_mode, self.cli_tq):
            if not c.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"service {c.srv_name} không sẵn sàng")

        self.pub = self.create_publisher(JointSingleCommand, "/rx150/commands/joint_single", 10)
        self.sub = self.create_subscription(JointState, "/rx150/joint_states",
                                            self.on_js, qos_profile_sensor_data)
        self.pos = None; self.vel = 0.0; self.js_stamp = None; self.ref0 = None
        self.last_print = 0.0
        self.log = []                 # [(t, phase, ref, pos, vel, e, u), ...]

    def on_js(self, msg):
        if JOINT not in msg.name:
            return
        i = msg.name.index(JOINT)
        self.pos = float(msg.position[i])
        self.vel = float(msg.velocity[i])
        self.js_stamp = time.monotonic()
        if self.ref0 is None:
            self.ref0 = self.pos
            self.get_logger().info(f"ref0 (pose gốc {JOINT}) = {self.ref0:+.4f} rad")

    def _call(self, client, req):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        return fut.result()

    def set_mode(self, mode, pt="time", pv=0, pa=0):
        r = OperatingModes.Request()
        r.cmd_type = "single"; r.name = JOINT; r.mode = mode
        r.profile_type = pt; r.profile_velocity = pv; r.profile_acceleration = pa
        return self._call(self.cli_mode, r)

    def set_torque(self, on):
        r = TorqueEnable.Request()
        r.cmd_type = "single"; r.name = JOINT; r.enable = bool(on)
        return self._call(self.cli_tq, r)

    def cmd(self, u):
        m = JointSingleCommand(); m.name = JOINT; m.cmd = float(u); self.pub.publish(m)

    def zero(self):
        self.cmd(0.0)

    def run(self):
        # chờ đo lần đầu
        t = time.monotonic()
        while self.ref0 is None and time.monotonic() - t < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.ref0 is None:
            self.get_logger().error("không nhận joint_states -> hủy (không chạm vào motor)")
            return
        if not (LIM_LO <= self.ref0 <= LIM_HI):
            self.get_logger().warn(f"ref0 {self.ref0:.3f} sát giới hạn -> vẫn chạy cẩn thận")

        self.set_mode("pwm")
        self.set_torque(True)
        self.get_logger().info(f"{JOINT} -> PWM, torque on. Bắt đầu: giữ {HOLD_S}s rồi bước +{STEP}rad")

        ref_hold = self.ref0
        ref_step = max(LIM_LO, min(LIM_HI, self.ref0 + STEP))
        t0 = time.monotonic()
        rate_dt = 1.0 / LOOP_HZ
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                now = time.monotonic()
                el = now - t0
                if el > RUN_S:
                    break
                ref = ref_hold if el < HOLD_S else ref_step
                # watchdog
                if self.js_stamp is None or (now - self.js_stamp) > WATCHDOG:
                    self.zero()
                    p = self.pos if self.pos is not None else 0.0
                    self.log.append((el, "WD", ref, p, self.vel, ref - p, 0.0))
                    if now - self.last_print > 1.0:
                        self.get_logger().error("stale joint_states -> zero PWM")
                        self.last_print = now
                    time.sleep(rate_dt); continue
                e = ref - self.pos
                ed = -self.vel
                en = max(-1.0, min(1.0, KE * e))
                edn = max(-1.0, min(1.0, KED * ed))
                un = float(self.f(en, edn))
                u = max(-UMAX, min(UMAX, un * KU))
                self.cmd(u)
                phase = "HOLD" if el < HOLD_S else "STEP"
                self.log.append((el, phase, ref, self.pos, self.vel, e, u))
                if now - self.last_print > 0.5:
                    self.get_logger().info(
                        f"[{phase}] t={el:5.2f}s ref={ref:+.3f} pos={self.pos:+.3f} "
                        f"e={e:+.3f} vel={self.vel:+.3f} u={u:+6.1f}")
                    self.last_print = now
                time.sleep(rate_dt)
        except (KeyboardInterrupt, Stop):
            pass
        finally:
            self.cleanup()
            self.save_results()

    def cleanup(self):
        try:
            self.zero(); time.sleep(0.05); self.zero()
            pos_now = self.pos if self.pos is not None else 0.0
            self.set_torque(False)
            self.set_mode("position")
            # đặt Goal_Position = vị trí hiện tại trước khi bật torque -> không bị nhảy
            for _ in range(5):
                self.cmd(pos_now); time.sleep(0.02)
            self.set_torque(True)
            self.get_logger().info(
                f"done: zero PWM + torque off + position mode @ {pos_now:+.3f} rad (không nhảy)")
        except Exception as ex:
            self.get_logger().error(f"cleanup lỗi: {ex}")

    def save_results(self):
        if not self.log:
            self.get_logger().warn("không có dữ liệu log -> bỏ qua plot")
            return
        os.makedirs(RUNS_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = f"{JOINT}_Ke{KE}_Ked{KED}_Ku{KU}_step{STEP}_{ts}"
        csv_path = os.path.join(RUNS_DIR, tag + ".csv")
        png_path = os.path.join(RUNS_DIR, tag + ".png")
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "phase", "ref", "pos", "vel", "e", "u"])
            for row in self.log:
                w.writerow([f"{x:.6f}" if isinstance(x, float) else x for x in row])
        t   = [r[0] for r in self.log]
        ref = [r[2] for r in self.log]
        pos = [r[3] for r in self.log]
        vel = [r[4] for r in self.log]
        err = [r[5] for r in self.log]
        u   = [r[6] for r in self.log]
        fig, ax = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
        ax[0].plot(t, ref, "k--", lw=1.4, label="ref")
        ax[0].plot(t, pos, lw=1.6, label="pos")
        ax[0].set_ylabel("góc (rad)"); ax[0].legend(loc="right"); ax[0].grid(alpha=.3)
        ax[0].set_title(f"{JOINT} | Ke={KE} Ked={KED} Ku={KU} step=+{STEP}rad  "
                        f"(đạt {pos[-1]:+.3f}, thiếu {ref[-1]-pos[-1]:+.3f} rad)")
        ax[1].plot(t, err, color="tab:orange"); ax[1].axhline(0, color="k", lw=.5)
        ax[1].set_ylabel("lỗi e (rad)"); ax[1].grid(alpha=.3)
        ax[2].plot(t, u, color="tab:green")
        ax[2].axhline(UMAX, ls=":", color="r"); ax[2].axhline(-UMAX, ls=":", color="r")
        ax[2].set_ylabel("PWM u"); ax[2].grid(alpha=.3)
        ax[3].plot(t, vel, color="tab:purple"); ax[3].axhline(0, color="k", lw=.5)
        ax[3].set_ylabel("vận tốc (rad/s)"); ax[3].set_xlabel("thời gian (s)"); ax[3].grid(alpha=.3)
        for a in ax:
            a.axvline(HOLD_S, ls="--", color="gray", lw=.8)
        fig.tight_layout(); fig.savefig(png_path, dpi=110); plt.close(fig)
        self.get_logger().info(f"đã lưu:\n  {csv_path}\n  {png_path}")


def main():
    rclpy.init()
    node = FuzzyJoint5()
    def _stop(signum, frame):
        raise Stop()
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        node.run()
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
