"""
path_postprocessor.py
=====================
Hậu xử lý quỹ đạo A* trước khi truyền sang Controller.

SỬA LỖI 5 — Phân mảnh tiền xử lý:
    Bản gốc: Hàm preconditionPath() nằm lạc trong mainHelper.py cùng với
             code giao tiếp mạng (async aiohttp, robot HTTP POST).
    Bản mới: Đóng gói vào đúng module planner, tách biệt hoàn toàn.

Thuật toán: Giữ nguyên 100% so với bản gốc.
    - Duyệt mảng waypoints theo cặp liên tiếp.
    - Nếu hai điểm liên tiếp có cùng theta (góc hướng) và tag=0 (đang đi):
        xóa điểm trước (collinear redundant), tăng pop_count.
    - Sau tối đa pop_threshold=3 lần xóa liên tiếp, buộc giữ điểm tiếp theo
        để không mất cấu trúc đường cong.
    - Điểm có tag=1 (pallet pickup) không bao giờ bị xóa.
"""


def precondition_path(path: list) -> list:
    """
    Lọc bỏ tối đa 3 waypoint collinear liên tiếp cùng góc theta,
    giảm mật độ điểm cho spline interpolation của Controller.

    Parameters
    ----------
    path : list
        Danh sách waypoints: [[x, y, theta, time, tag], ...]
        tag: 0=đang đi, 1=tại pallet (pickup), 2=tại goal (drop-off)

    Returns
    -------
    list
        Danh sách waypoints đã được lọc (in-place modification + return).
    """
    i = 0
    pop_count = 0
    pop_threshold = 3

    while i < len(path) - 1:
        # Truyền tag pallet pickup lên điểm trước để Controller nhận đúng tín hiệu
        if path[i + 1][4] == 1:
            path[i][4] = 1

        # Điều kiện xóa điểm i:
        #   - Cùng theta với điểm tiếp theo (collinear)
        #   - Chưa vượt quá pop_threshold lần xóa liên tiếp
        #   - Điểm i không phải pickup point (tag != 0 thì không xóa)
        if (path[i][2] == path[i + 1][2]    # cùng góc hướng
                and pop_count < pop_threshold
                and path[i][4] == 0):         # đang ở chế độ driving
            path.pop(i)
            pop_count += 1
        else:
            i += 1
            pop_count = 0  # Reset đếm khi gặp điểm rẽ hoặc đặc biệt

    return path
