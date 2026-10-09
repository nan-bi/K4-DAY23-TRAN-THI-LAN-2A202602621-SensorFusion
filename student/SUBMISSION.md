# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Trần Thị Lan
- MSSV: 02621
- Email: 2211090020@studenthuph.edu.vn
- Link repo (fork): https://github.com/nan-bi/K4-Track4-Day23-Sensor-Fusion-Student-Tran-Thi-Lan-02621
- Commit hash nộp (`git rev-parse HEAD`):

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`:
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`:
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`:
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`:
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss:

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`):
   - **LiDAR**: Đo trực tiếp vị trí không gian 3D của xe $z = [x, y, z]^T \in \mathbb{R}^3$ (đơn vị: mét). Ma trận hiệp phương sai nhiễu $R = \text{diag}(\sigma_x^2, \sigma_y^2, \sigma_z^2) \in \mathbb{R}^{3 \times 3}$. Mô hình đo $h(x)$ là tuyến tính, ma trận Jacobian $H = [I_3 \mid 0_{3 \times 3}] \in \mathbb{R}^{3 \times 6}$.
   - **Camera**: Đo toạ độ pixel 2D trên mặt phẳng ảnh $z = [u, v]^T \in \mathbb{R}^2$ (đơn vị: pixel). Ma trận hiệp phương sai nhiễu $R = \text{diag}(\sigma_u^2, \sigma_v^2) \in \mathbb{R}^{2 \times 2}$. Mô hình đo $h(x)$ là phi tuyến (phép chiếu pinhole perspective projection), ma trận Jacobian $H \in \mathbb{R}^{2 \times 6}$ được tính bằng đạo hàm chuỗi (chain rule) kết hợp ma trận xoay $R_{veh \to sens}$.

2. Vì sao cần gating Mahalanobis trước khi gán?
   - Khoảng cách Mahalanobis $d^2 = \gamma^T S^{-1} \gamma$ chuẩn hoá khoảng cách sai lệch dựa trên ma trận hiệp phương sai $S = H P H^T + R$, phản ánh đúng độ bất định của cả trạng thái dự báo và phép đo thay vì chỉ dùng khoảng cách Euclid hình học.
   - Cổng kiểm định $\chi^2$ (chi-square gate) loại bỏ các phép đo nằm ngoài vùng phân phối xác suất tin cậy của track (loại bỏ outliers, đo không thuộc về track), giúp thuật toán gán greedy chỉ xét các ứng viên hợp lệ, tránh gán sai (false association) và giảm khối lượng tính toán.

3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.
   - Pipeline là **track-then-fuse**: Hệ thống duy trì duy nhất **một** danh sách track. Mỗi frame Waymo, EKF thực hiện **predict một lần** cho toàn bộ track, sau đó lần lượt gán và cập nhật EKF nối tiếp theo từng cảm biến: **AssocL** (LiDAR) $\to$ **AssocC** (Camera). Camera chỉ tinh chỉnh trạng thái EKF mà không tạo track riêng.
   - Trên log `fusion-run-lab`, mỗi frame được xử lý theo đúng trình tự: 1 lượt `predict` $\to$ `AssocL` (gán lidar, EKF update, quản lý track) $\to$ `AssocC` (gán camera 2D, EKF update) trên cùng tập track đó.

4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?
   - Khi camera bị lệch calibration (lệch extrinsic xoay/tịnh tiến hoặc intrinsic tiêu cự/tâm ảnh), toạ độ pixel dự báo $h(x)$ bị lệch có hệ thống (systematic bias/offset) so với toạ độ đo $z$.
   - Triệu chứng: Innovation $\gamma = z - h(x)$ có kỳ vọng khác 0 (non-zero mean residual, xuất hiện bias cố định thay vì dao động quanh 0). Khoảng cách Mahalanobis $d^2$ tăng cao khiến phép đo camera dễ bị cổng $\chi^2$ từ chối (bị gating loại bỏ), hoặc nếu được gán thì EKF update sẽ kéo trạng thái track lệch khỏi ground-truth, làm RMSE tracking tăng lên.

5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng? Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.
   - Cần truyền `sensor` tường minh vì khi `meas_list` rỗng, không thể suy ra loại cảm biến từ dữ liệu đo. Lượt LiDAR dù không có đo vẫn phải chạy `manage_tracks` để thực hiện suy giảm điểm (score decay) cho các track trong FOV bị miss và xóa các track cạn điểm.
   - LiDAR đo trực tiếp vị trí 3D không gian nên có độ tin cậy hình học cao, dùng để khởi tạo, tính điểm tồn tại, xác nhận và xóa track. Camera trong lab chỉ đo góc 2D (mất thông tin độ sâu tuyệt đối) nên chỉ dùng để tinh chỉnh trạng thái $x, P$ trong EKF update, không can thiệp vòng đời track để tránh sinh ghost track hay xóa nhầm track.

6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.
   - **Điều kiện xác nhận (Confirmation)**: Track có `score > confirmed_threshold` (0.8) sẽ chuyển sang trạng thái `"confirmed"`.
   - **Giữ confirmed sau miss**: Khi track đã ở trạng thái `"confirmed"`, nếu gặp một frame miss trong FOV thì `score` bị trừ $1/\text{window}$ (tức $-1/6$), nhưng trạng thái vẫn được **giữ nguyên `"confirmed"`** (không bị hạ về tentative).
   - **Điều kiện xóa track (Deletion)**: Track bị xóa nếu thoả mãn bất kỳ điều kiện nào sau:
     1. Phương sai vị trí ngang $P_{xx} > \text{max\_P}$ (9.0) hoặc $P_{yy} > \text{max\_P}$ (9.0); HOẶC
     2. Track `"confirmed"` có `score < delete_threshold` (0.6); HOẶC
     3. Track chưa confirmed (`"initialized"` hoặc `"tentative"`) có `score \le 0.0`.

## Bonus (không bắt buộc)

- Không

## Khai báo sử dụng AI (bắt buộc)

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Antigravity AI Coding Assistant
- Dùng cho phần nào (hàm, câu hỏi, debug): Hỗ trợ cài đặt môi trường, hoàn thiện các hàm EKF, Association, Camera Fusion, Track Management (Parts E–H) theo tài liệu kỹ thuật, và trả lời câu hỏi lý thuyết.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Chạy toàn bộ bộ kiểm thử tự động `pytest student/tests` (pass 128/128 tests), đối chiếu công thức ma trận EKF, cổng chi2 và vòng đời track theo docs/HUONG_DAN_KY_THUAT.md.

## Checklist nộp

- [ ] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [ ] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [ ] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [ ] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [ ] Đã điền đủ file này, gồm khai báo AI
- [ ] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [ ] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
