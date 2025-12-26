import csv
import os
import re
import time
from typing import Dict, List, Sequence, Tuple

from dobot_api import DobotApiDashboard


POSE_COLUMNS_CANONICAL = ("x", "y", "z", "rx", "ry", "rz")


# =========================
# 直接在这里修改配置即可运行
# =========================

# 机械臂控制器 IP（和 UI 里 Connect 输入框一致）
ROBOT_IP = "111.111.130.198"

# Dashboard 端口（和 UI 里 Dashboard Port 一致）
DASH_PORT = 29999

# CSV 目录与文件：
# - CSV_PATH 留空字符串 ""：自动选择 DATA_DIR 下最新的 csv
# - 或者写死为具体路径，例如："data/record_20251224_163601.csv"
DATA_DIR = "data"
CSV_PATH = ""

# 额外的直线速度比例（0..100），可进一步降速
VELL_RATIO = 20

# 连续运动（轨迹更顺滑）：
# - BLEND_R_MM > 0：相邻 MovL 之间使用圆弧过渡半径 r（单位 mm），轨迹会更连续
# - CP_RATIO > 0：使用平滑过渡比例 cp（0..100），与 r 互斥（r 优先）
# 建议优先用 r（例如 2~10mm），更直观。
BLEND_R_MM = 2
CP_RATIO = 0

# 指令发送间隔（秒）。不再用“按距离等待”，而是小间隔持续下发，让控制器自己做连续过渡。
SEND_INTERVAL_S = 0.04

# 每 N 行回放一次（1=数据全使用进行回放，2=每隔一行，降低点数）
STEP = 2


def _find_latest_csv(data_dir: str) -> str:
    if not os.path.isdir(data_dir):
        raise FileNotFoundError(f"data_dir not found: {data_dir}")

    candidates: List[str] = []
    for name in os.listdir(data_dir):
        if name.lower().endswith(".csv"):
            candidates.append(os.path.join(data_dir, name))

    if not candidates:
        raise FileNotFoundError(f"no csv files under: {data_dir}")

    candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return candidates[0]


def _build_header_map(fieldnames: Sequence[str]) -> Dict[str, str]:
    header_map: Dict[str, str] = {}
    for raw in fieldnames:
        if raw is None:
            continue
        key = raw.strip().lower()
        if key and key not in header_map:
            header_map[key] = raw
    return header_map


def read_poses_from_csv(csv_path: str) -> List[Tuple[float, float, float, float, float, float]]:
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"csv not found: {csv_path}")

    poses: List[Tuple[float, float, float, float, float, float]] = []
    with open(csv_path, "r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("csv has no header row")

        header_map = _build_header_map(reader.fieldnames)
        missing = [k for k in POSE_COLUMNS_CANONICAL if k not in header_map]
        if missing:
            available = ", ".join(reader.fieldnames)
            raise ValueError(
                f"missing columns: {missing}. Available columns: {available}"
            )

        cols = [header_map[k] for k in POSE_COLUMNS_CANONICAL]
        for row in reader:
            try:
                values = [float((row.get(col) or "").strip()) for col in cols]
            except Exception:
                continue
            poses.append((values[0], values[1], values[2], values[3], values[4], values[5]))

    if not poses:
        raise ValueError("no valid pose rows read from csv")
    return poses


_FLOAT_RE = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")


def _parse_result_code(reply: str) -> int:
    m = re.search(r"-?\d+", reply or "")
    return int(m.group(0)) if m else 0


def _parse_pose_from_getpose_reply(reply: str) -> Tuple[float, float, float, float, float, float]:
    nums = [float(x) for x in _FLOAT_RE.findall(reply or "")]
    if len(nums) >= 7:
        vals = nums[1:7]
    elif len(nums) >= 6:
        vals = nums[-6:]
    else:
        raise ValueError(f"GetPose reply parse failed: {reply!r}")
    return float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3]), float(vals[4]), float(vals[5])


def _wait_until_near_pose(
    dash: DobotApiDashboard,
    target: Tuple[float, float, float, float, float, float],
    *,
    pos_tol_mm: float,
    rot_tol_deg: float,
    poll_s: float,
    timeout_s: float,
) -> None:
    start = time.time()
    while True:
        reply = dash.GetPose()
        try:
            cur = _parse_pose_from_getpose_reply(reply)
        except Exception:
            cur = None

        if cur is not None:
            dx = cur[0] - target[0]
            dy = cur[1] - target[1]
            dz = cur[2] - target[2]
            pos_err = (dx * dx + dy * dy + dz * dz) ** 0.5
            rot_err = max(abs(cur[3] - target[3]), abs(cur[4] - target[4]), abs(cur[5] - target[5]))
            if pos_err <= pos_tol_mm and rot_err <= rot_tol_deg:
                return

        if time.time() - start >= timeout_s:
            raise TimeoutError("wait pose timeout")
        time.sleep(poll_s)


def main() -> int:
    if STEP < 1:
        raise ValueError("STEP must be >= 1")
    if not ROBOT_IP or ROBOT_IP.strip() == "":
        raise ValueError("ROBOT_IP is empty. Please set ROBOT_IP at top of file.")

    csv_path = CSV_PATH.strip() or _find_latest_csv(DATA_DIR)
    poses = read_poses_from_csv(csv_path)

    dash = None
    try:
        dash = DobotApiDashboard(ROBOT_IP, int(DASH_PORT))
        dash.EnableRobot()

        try:
            dash.VelL(int(VELL_RATIO))
        except Exception:
            pass

        params = {}
        if BLEND_R_MM and int(BLEND_R_MM) > 0:
            params["r"] = int(BLEND_R_MM)
        elif CP_RATIO and int(CP_RATIO) > 0:
            params["cp"] = int(CP_RATIO)

        print(f"CSV: {csv_path}")
        print(f"Robot IP: {ROBOT_IP} | DashPort: {DASH_PORT}")
        print(f"Total poses: {len(poses)} | step={STEP}")
        print(
            f"Mode: r={params.get('r', 0)} | cp={params.get('cp', 0)}"
        )

        for idx in range(0, len(poses), STEP):
            x, y, z, rx, ry, rz = poses[idx]
            reply = dash.MovL(
                x,
                y,
                z,
                rx,
                ry,
                rz,
                0,
                **params,
            )
            code = _parse_result_code(reply)
            if code < 0:
                try:
                    dash.Stop()
                except Exception:
                    pass
                raise RuntimeError(f"MovL rejected (code={code}) at row {idx + 1}")

            time.sleep(float(SEND_INTERVAL_S))

        print("Replay done.")
        return 0
    except KeyboardInterrupt:
        print("Interrupted by user.")
        try:
            if dash is not None:
                dash.Stop()
        except Exception:
            pass
        return 130
    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        try:
            if dash is not None:
                dash.close()
                try:
                    dash.socket_dobot = 0
                except Exception:
                    pass
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
