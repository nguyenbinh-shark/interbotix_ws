"""
plot_styles — Cấu hình màu sắc, style cho đồ thị.

Tập trung quản lý style ở đây để dễ thay đổi khi
thêm bộ điều khiển mới hoặc đổi phong cách.
"""

import matplotlib.pyplot as plt

# Bảng màu cho từng khớp (tuỳ chỉnh ở đây)
JOINT_COLORS = {
    'waist':        '#e74c3c',
    'shoulder':     '#3498db',
    'elbow':        '#2ecc71',
    'wrist_angle':  '#f39c12',
    'wrist_rotate': '#9b59b6',
}

# Màu mặc định cho khớp không có trong bảng (dùng khi đổi robot)
_FALLBACK_COLORS = [
    '#1abc9c', '#e67e22', '#2c3e50', '#c0392b', '#7f8c8d',
    '#16a085', '#d35400', '#8e44ad', '#27ae60', '#2980b9',
]


def get_joint_color(joint_name, index=0):
    """Lấy màu cho khớp. Nếu không có trong bảng, dùng fallback theo index."""
    if joint_name in JOINT_COLORS:
        return JOINT_COLORS[joint_name]
    return _FALLBACK_COLORS[index % len(_FALLBACK_COLORS)]


def setup_style():
    """Cấu hình style matplotlib chung."""
    plt.rcParams.update({
        'figure.facecolor': 'white',
        'axes.facecolor': '#fafafa',
        'axes.grid': True,
        'grid.alpha': 0.3,
        'grid.linestyle': '--',
        'font.size': 9,
        'axes.titlesize': 11,
        'axes.labelsize': 9,
        'xtick.labelsize': 8,
        'ytick.labelsize': 8,
        'legend.fontsize': 7,
        'figure.max_open_warning': 0,
    })
