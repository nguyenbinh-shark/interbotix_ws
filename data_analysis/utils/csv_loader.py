"""
csv_loader — Load và validate file CSV dữ liệu điều khiển.

Hỗ trợ phát hiện tự động danh sách khớp từ header CSV,
không hardcode tên khớp cụ thể.
"""

import sys
import re
import pandas as pd


# Tên khớp mặc định (rx150 arm) — dùng khi không detect được từ CSV.
DEFAULT_JOINTS = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']


def load_csv(filepath, t_start=None, t_end=None):
    """Load file CSV, filter theo khoảng thời gian nếu cần.

    Args:
        filepath: Đường dẫn file CSV.
        t_start: Thời gian bắt đầu (giây), None = từ đầu.
        t_end: Thời gian kết thúc (giây), None = đến cuối.

    Returns:
        pandas DataFrame đã filter.
    """
    df = pd.read_csv(filepath)
    if 'timestamp' not in df.columns:
        print('ERROR: file CSV thiếu cột "timestamp"', file=sys.stderr)
        sys.exit(1)
    if t_start is not None:
        df = df[df['timestamp'] >= t_start]
    if t_end is not None:
        df = df[df['timestamp'] <= t_end]
    df = df.reset_index(drop=True)
    return df


def detect_joints(df, suffix='_pos'):
    """Phát hiện danh sách khớp từ tên cột CSV.

    Tìm các cột kết thúc bằng `suffix` (mặc định '_pos'),
    trích xuất tên khớp phía trước.

    Args:
        df: DataFrame đã load.
        suffix: Hậu tố cột dùng để detect (mặc định '_pos').

    Returns:
        List[str] tên khớp, hoặc DEFAULT_JOINTS nếu không detect được.
    """
    pattern = re.compile(rf'^(.+){re.escape(suffix)}$')
    joints = []
    for col in df.columns:
        m = pattern.match(col)
        if m:
            name = m.group(1)
            # Bỏ qua các cột ref_pos (có dạng joint_ref_pos)
            if name.endswith('_ref'):
                continue
            joints.append(name)
    return joints if joints else list(DEFAULT_JOINTS)


def get_data_summary(df, joints):
    """In tóm tắt dữ liệu.

    Args:
        df: DataFrame.
        joints: List tên khớp.

    Returns:
        Dict chứa thông tin tóm tắt.
    """
    duration = df['timestamp'].max() - df['timestamp'].min()
    n_rows = len(df)
    freq = n_rows / max(duration, 0.01)
    return {
        'n_rows': n_rows,
        'duration': duration,
        'frequency': freq,
        'joints': joints,
    }
