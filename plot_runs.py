#!/usr/bin/env python3
# Overlay nhiều lần chạy (trong runs/) để so sánh A/B khi tune gain.
# Dùng:
#   python3 plot_runs.py                 # overlay tất cả *.csv trong runs/
#   python3 plot_runs.py runs/a.csv ...  # overlay chỉ các file chỉ định
import sys, os, glob, csv
import matplotlib.pyplot as plt

RUNS = "runs"

def load(path):
    t = []; ref = []; pos = []; u = []; vel = []
    with open(path) as f:
        for r in csv.DictReader(f):
            t.append(float(r["t"])); ref.append(float(r["ref"]))
            pos.append(float(r["pos"])); u.append(float(r["u"])); vel.append(float(r["vel"]))
    return t, ref, pos, u, vel

def main():
    files = sys.argv[1:] or sorted(glob.glob(os.path.join(RUNS, "*.csv")))
    if not files:
        print("Không có CSV trong runs/. Chạy test_joint5.py trước.")
        return
    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for p in files:
        t, ref, pos, u, vel = load(p)
        label = os.path.splitext(os.path.basename(p))[0]
        ax[0].plot(t, pos, label=label)
        ax[1].plot(t, u)
        ax[2].plot(t, vel)
    t, ref, pos, u, vel = load(files[0])
    ax[0].plot(t, ref, "k--", lw=1, label="ref")
    ax[0].set_ylabel("pos (rad)"); ax[0].legend(fontsize=7, loc="right"); ax[0].grid(alpha=.3)
    ax[1].set_ylabel("PWM u"); ax[1].grid(alpha=.3)
    ax[2].set_ylabel("vel (rad/s)"); ax[2].grid(alpha=.3); ax[2].set_xlabel("thời gian (s)")
    fig.suptitle(f"So sánh {len(files)} lần chạy (A/B tuning)")
    fig.tight_layout()
    out = os.path.join(RUNS, "overlay.png")
    fig.savefig(out, dpi=110)
    plt.show()
    print(f"overlay -> {out}")

if __name__ == "__main__":
    main()
