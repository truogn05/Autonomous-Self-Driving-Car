# My World — sa bàn giao thông Webots R2025a

Mở `worlds/my_world.wbt` bằng Webots R2025a. Nhấn Run để chạy chu kỳ đèn.
World chuẩn bị cho robot 2WD khoảng 0,30 × 0,22 m; xe ô tô mẫu kích thước thật
như BMW X5 không vừa làn 0,60 m này.

## Môi trường

- Nền 54 × 36 m, phạm vi x=[-30,24], y=[-18,18], ENU; chạy bên phải.
- 29 đoạn đường, đường bao khép kín và các ô phố liên thông.
- Các ngã ba T, hai ngã tư (ngã tư `SIGNAL_CROSS` có đèn), một ngã ba Y phía Nam.
  Có 19 junction native, tương ứng 22 nút/cổng trong map; bốn cổng vòng xuyến dùng chung ID `ROUNDABOUT`.
  Mỗi đường có `startJunction`/`endJunction`, mỗi junction có `connectedRoadIDs` đối ứng.
  Bó vỉa junction theo biên ngoài, chừa mở toàn bộ miệng đường.
- Cột đường Tây ở x=-27, cột ô phố ở x=-17; hai đoạn ngang trên/dưới dài 6,8 m và 5,8 m.
  `BLOCK_MID_TUNNEL` nối ngang dưới cầu `RECONNECTED_BRIDGE`, có thêm 3 m đường phẳng ở mỗi đầu.
  Mặt hầm z=-0,55 m và mặt cầu z=0,008 m; hai mối nối cầu là junction logic riêng.
- Vòng xuyến bốn cửa phía Đông: đảo bán kính 2,4 m, mép ngoài 4 m;
  hai làn đi ngược chiều kim đồng hồ, biển nhường đường ở các cửa vào.
- Đường thường rộng 2,40 m, bốn làn 0,60 m: hai làn mỗi chiều. Vạch trắng đứt phân làn cùng chiều, vạch vàng đôi phân cách hai chiều, dùng `RoadLine` như `city.wbt`. Góc cua và đường chữ S nằm ở đường bao/nhánh nội bộ.
- Vạch dừng, mũi tên, vạch sang đường, biển STOP/nhường đường/hướng vòng xuyến.
- Biển `0.5 m/s` là biển minh họa cho sa bàn robot, không phải biển giao thông chuẩn.
- Đoạn `NORTH_TRAINING` hiện là đường bốn làn đều, dùng `StraightRoadSegment` như các đoạn thẳng khác.
  Hộp `TRAINING_OBSTACLE` vẫn có thể di chuyển để thử đổi làn/đổi tuyến; không còn hình học mở rộng bất đối xứng.
- Vạch dừng và vạch sang đường dùng texture qua `startLine`/`endLine`, không có `Shape` vẽ vạch ở world.
  Hình học nội bộ của PROTO Webots vẫn dùng Shape theo thiết kế upstream.
- Vạch dừng phủ hai làn đi vào tất cả giao lộ T/X/Y; cửa vòng xuyến không có vạch dừng.
- `OuterBarrier-1` dùng `CrashBarrier` chạy kín theo mép ngoài sa bàn, chừa thông các nhánh giao lộ.
  Thanh chắn được hạ xuống: collider cao 0,02–0,42 m, khoảng cách cột 0,6 m, phù hợp robot nhỏ.
- Bố cục được giãn 2 lần để thêm làn mà vẫn giữ bề rộng làn, bán kính cua và khoảng cách giữa đường.
  Biển và đèn chỉ được dời theo lề mới; logic điều khiển không đổi.
- `Roundabout` local đã có hai làn vòng; bốn đường tiếp cận đều có bốn làn (2+2).
  Controller xe cần nhường đường/hợp làn ở cửa vào; chưa mô phỏng ưu tiên nhiều xe cùng vào.


Đã nối lại hai đầu cụt trong ô phố Tây, nối nhánh Bắc vòng xuyến lên đường bao,
và bỏ các nhánh thừa nhô ra mép trên/trái của ảnh tham khảo. Không có đường cụt.

## Đặt robot của bạn

- Điểm đầu: `translation -26.1 -10.6 ...`, heading `+pi/2` (Bắc).
- Điểm đích: khoảng `(20.1, 6)`, heading `-pi/2` (Nam).
- Cao độ spawn tùy bán kính bánh và gốc tọa độ robot; mặt nền vật lý ở z=0.
- Ô xanh/cam ngoài lề đánh dấu vùng đầu/đích, không phải vật cản trên làn.
- Robot và controller tự lái chưa được thêm vào world. Biển, vạch và dữ liệu map
  không tự áp luật giao thông lên robot; controller xe phải xử lý chúng.

## File và chỉnh sửa

| File | Vai trò |
|---|---|
| `worlds/my_world.wbt` | World dùng chính, mở trực tiếp trong Webots |
| `worlds/my_world.before_four_lanes.wbt` | Bản sao trước khi chuyển sang bốn làn |
| `worlds/my_world.original.wbt` | Bản sao world ban đầu trước khi sinh môi trường |
| `tools/build_world.py`, `tools/sync_world.py` | Đồng bộ junction, bó vỉa, barrier và map từ world đã lưu |
| `map.json` | Hình học đường, junction, làn có hướng, điểm đầu/đích |
| `map.svg` | Sơ đồ nhìn từ trên, có ID để tra trong map |
| `controllers/traffic_signals/traffic_signals.py` | Một đồng hồ chung cho bốn hướng đèn |
| `controllers/world_inspector/world_inspector.py` | Kiểm tra mô phỏng; mặc định chỉ chờ |
| `protos/vendor/` | PROTO R2025a và texture đã tải về, giữ license trong từng file |
| `validation/` | Báo cáo và ảnh kiểm tra; xem hash để biết đúng phiên bản |

Chạy `python webots/tools/build_world.py` từ gốc repository để đồng bộ world/map.
World đã lưu là nguồn hình học: lệnh giữ hầm/cầu và các đối tượng chỉnh trong GUI,
chỉ dựng lại junction, bó vỉa junction, barrier và dữ liệu làn. Sao lưu trước khi đổi bố cục lớn.
Không cần sinh lại để mở world. `make_signs.py` chỉ cần khi muốn tạo lại texture biển,
dùng Pillow và font Arial của Windows. Generator dùng thêm Shapely (đã dùng trong kiểm tra hình học)
để offset biên barrier và loại bỏ chỗ tự cắt tại đoạn cong/giao lộ.

Để mở offline, world chính tham chiếu các PROTO local trong `protos/vendor`.
`tools/fetch_assets.py` dùng khi cần lấy lại asset, không cần chạy mỗi lần mở world.
Các PROTO dùng tham chiếu tài nguyên local. Riêng `Road.proto` có hai sửa lỗi cục bộ:
nhóm texture `startLine`/`endLine` theo đúng làn hiện tại, và cho vạch bổ sung kế thừa
độ rộng của vạch đã cấu hình khi tăng `numberOfLanes`. Giữ các sửa này khi tải lại asset.
Nguồn: https://github.com/cyberbotics/webots/tree/R2025a/projects/objects/road/protos

### Tải danh sách PROTO road, traffic, cây và xe

Chạy từ gốc repository (chỉ cần Python, không cần cài thêm thư viện):

```powershell
python webots/tools/download_protos.py
```

Script tải 73 PROTO R2025a: 13 road, 29 traffic, 9 cây và 22 xe. File đã có được bỏ qua
để giữ các chỉnh sửa cục bộ; nếu tải bị lỗi, chạy lại để tiếp tục.
Thư mục đích giữ nguyên cấu trúc upstream:

```text
webots/protos/vendor/projects/
├── objects/
│   ├── road/protos/*.proto
│   ├── traffic/protos/*.proto
│   └── trees/protos/*.proto
└── vehicles/protos/
    ├── abstract/*.proto
    ├── bmw/*.proto
    ├── citroen/*.proto
    ├── generic/*.proto
    ├── lincoln/*.proto
    ├── mercedes_benz/*.proto
    ├── range_rover/*.proto
    ├── tesla/*.proto
    └── toyota/*.proto
```

Script này chỉ tải các file `.proto` trong danh sách, giữ nguyên nội dung và license
upstream; không tải texture, mesh hay PROTO phụ thuộc. Các model có tham chiếu tương đối
đến tài nguyên chưa tải cần bổ sung các tài nguyên đó trước khi dùng trong world.

## Đèn và làn

Chu kỳ 26 s: Đông–Tây xanh 10 s → vàng 2 s → tất cả đỏ 1 s →
Bắc–Nam xanh 10 s → vàng 2 s → tất cả đỏ 1 s. Đèn bắt đầu hoạt động khi Run.
Xe rẽ vẫn phải nhường xe đối diện/người đi bộ; đây không phải pha rẽ trái riêng.

Trong `map.json`, `_F1`/`_F2` là hai làn chiều thuận, `_B1`/`_B2` chiều ngược;
1 là làn sát tim, 2 là làn sát lề. Mỗi làn tách thành `_entry`/`_exit` với vùng
chuyển làn trên đường (`CHANGE_*`), trước khi đến giao lộ. Tại giao lộ, đi thẳng giữ
thứ tự làn, rẽ trái từ làn trong và rẽ phải từ làn ngoài; không tự thêm quay đầu.
Các mục có `junction` là đường nối; `successors` chứa ID đi tiếp. Vòng xuyến dùng `RING_*` một chiều.
`roads[].elevations` lưu cao độ của từng waypoint, phân biệt hầm và cầu khi đường cắt nhau trên hình chiếu XY.
`speed_limit_m_s` chỉ là tham số map, không tự hãm xe trong mô phỏng.

Các node đường có `name` rõ ràng và không khai báo DEF riêng. ID trong map giữ nguyên để tra cứu.
Một số node khác như `SIGNAL_SYSTEM` vẫn cần DEF để controller kiểm tra tham chiếu.

Nền đỡ xe ở các đường thường; hầm/cầu có collider mặt đường riêng.
Lề, đảo giữa, cột biển/đèn và hộp thử có collider riêng. Cách này tránh hai mặt đỡ
chồng nhau gây rung bánh. Chưa có NPC, người đi bộ động, nhà hoặc cây che tầm nhìn.

## Kiểm tra

```powershell
python webots/tools/build_world.py
python webots/tools/check_geometry.py
node webots/tools/check_road_template.cjs
python webots/tools/verify_junctions.py --webots E:/ProgramE/Webots/msys64/mingw64/bin/webots.exe
```

- Generator kiểm tra làn nối đúng đầu mút, toàn bộ mạng có hướng liên thông hai chiều
  về khả năng đi đến nhau, không có nhánh cụt, tọa độ trong nền.
- `check_geometry.py` dùng Shapely kiểm tra vùng quét hình tròn bán kính
  0,186 m của 628 đoạn làn/đường nối nằm trong mặt đường. Không kiểm điều khiển động học.
- Kiểm tra thêm vạch dừng trên từng hướng vào, không có vạch dừng ở vòng xuyến,
  barrier khép kín, không tự cắt và không lấn mặt đường/giao lộ.
- `check_road_template.cjs` thực thi JavaScript thực tế trong `Road.proto`, kiểm tra
  độ rộng vạch khi đổi 4/5/6/7 làn trên đường thẳng/cong và bề rộng texture đầu/cuối đường.
  Chỉnh số làn trong GUI không tự cập nhật `map.json` hay vị trí vạch vàng;
  mạng đường dùng cho điều hướng vẫn là cấu hình 4 làn (2+2).
- `verify_junctions.py` sao chép project vào Temp, chạy Webots thu nhỏ với port riêng;
  kiểm tra toàn bộ road/junction và ba probe va chạm hầm/cầu, xuất hai ảnh rồi thoát.
  Report có nonce/hash xác nhận đúng lần chạy; không đóng phiên Webots đang dùng.
  `verify_world.py` là verifier cũ với probe cố định, không dùng làm gate cho bố cục mới.
- `runtime.json` và `geometry.json` ghi hash của world. Một báo cáo cũ không xác nhận
  bản world mới. Ảnh `town-overview.png` là camera toàn cảnh; `robot-eye-sign.png`
  kiểm tra biển ở độ cao camera 0,15 m.

Đây là kiểm chứng môi trường, không phải kiểm chứng xe đã tự lái thành công.

## Trạng thái hiện tại

Đã kiểm tra hình học 29 đường, 19 junction, 50 miệng đường, 554 tam giác và 628 đoạn làn.
Webots R2025a đã mở bản sao mới thành công; ba probe hầm/cầu PASS.
Xem `validation/geometry.json`, `validation/junction-runtime.json` và hai ảnh `junction-*.png`.
Đây là kiểm tra môi trường và vật thử tĩnh, chưa kiểm chứng robot chạy qua toàn bộ tuyến.
