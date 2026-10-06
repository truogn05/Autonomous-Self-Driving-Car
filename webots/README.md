# My World — sa bàn giao thông Webots R2025a

Mở `worlds/my_world.wbt` bằng Webots R2025a. Nhấn Run để chạy chu kỳ đèn.
World chuẩn bị cho robot 2WD khoảng 0,30 × 0,22 m; xe ô tô mẫu kích thước thật
như BMW X5 không vừa làn 0,60 m này.

## Môi trường

- Nền 96 × 78 m, phạm vi x=[-48,48], y=[-30,48], ENU; chạy bên phải.
- 55 đoạn đường, gồm 26 đoạn mới mở rộng ra ngoài nền cũ; đường bao khép kín và các ô phố liên thông.
- Các ngã ba T và ngã tư; `SIGNAL_CROSS` có đèn. Các nút đường bao phía Bắc/Tây/Nam
  đã nối thêm nhánh ra phần mở rộng.
  Có 33 junction native, tương ứng 36 nút/cổng logic trong map; bốn cổng vòng xuyến dùng chung ID `ROUNDABOUT`.
  Mỗi đường có `startJunction`/`endJunction`, mỗi junction có `connectedRoadIDs` đối ứng.
  Bó vỉa junction theo biên ngoài, chừa mở toàn bộ miệng đường.
- Cột đường Tây ở x=-35, cột ô phố ở x=-23; các đoạn ngang trên/dưới đã được kéo dài.
  `BLOCK_MID_TUNNEL` nối ngang dưới cầu `RECONNECTED_BRIDGE` tại x=-23; cầu dài 5,6 m.
  Mặt hầm z=-0,55 m và mặt cầu z=0,008 m; hai mối nối cầu là junction logic riêng.
- Vòng xuyến bốn cửa phía Đông: đảo bán kính 2,4 m, mép ngoài 4,8 m;
  bốn làn đi ngược chiều kim đồng hồ, biển nhường đường ở các cửa vào.
- Đường thường rộng 2,40 m, bốn làn 0,60 m: hai làn mỗi chiều.
  Trục giữa và các nhánh chính rộng 4,80 m, tổng cộng tám làn: bốn làn mỗi chiều.
  Vạch trắng đứt phân làn cùng chiều, vạch vàng đôi phân cách hai chiều, dùng `RoadLine` như `city.wbt`.
  Góc cua và đường chữ S nằm ở đường bao/nhánh nội bộ.
- Hai đường chữ S phía Bắc `NORTH_WINDING_W`/`NORTH_WINDING_E` có mặt đường cao thêm
  0,60/0,70 m so với mặt đường thường; dốc lớn nhất lần lượt 14,47%/14,61%, có collider mặt đường riêng.
- Vạch dừng, mũi tên, vạch sang đường, biển STOP/nhường đường/hướng vòng xuyến.
- Biển `0.5 m/s` là biển minh họa cho sa bàn robot, không phải biển giao thông chuẩn.
- Đoạn `NORTH_TRAINING` hiện là đường bốn làn đều, dùng `StraightRoadSegment` như các đoạn thẳng khác.
  Hộp `TRAINING_OBSTACLE` vẫn có thể di chuyển để thử đổi làn/đổi tuyến; không còn hình học mở rộng bất đối xứng.
- Vạch dừng và vạch sang đường dùng texture qua `startLine`/`endLine`, không có `Shape` vẽ vạch ở world.
  Hình học nội bộ của PROTO Webots vẫn dùng Shape theo thiết kế upstream.
- Vạch dừng phủ toàn bộ làn đi vào các giao lộ T/X/Y; cửa vòng xuyến không có vạch dừng.
- `OuterBarrier-1` dùng `CrashBarrier` chạy kín theo mép ngoài sa bàn, chừa thông các nhánh giao lộ.
  Thanh chắn được hạ xuống: collider cao 0,02–0,42 m, khoảng cách cột 0,6 m, phù hợp robot nhỏ.
- Bố cục được kéo giãn và bổ sung đường để mở rộng sa bàn; bề rộng mỗi làn vẫn là 0,60 m.
  Biển và đèn chỉ được dời theo lề mới; logic điều khiển không đổi.
- `Roundabout` local có bốn làn vòng; bốn đường tiếp cận có tám làn (4+4).
  Controller xe cần nhường đường/hợp làn ở cửa vào; chưa mô phỏng ưu tiên nhiều xe cùng vào.


Đã nối lại hai đầu cụt trong ô phố Tây, nối nhánh Bắc vòng xuyến lên đường bao,
và bỏ các nhánh thừa nhô ra mép trên/trái của ảnh tham khảo. Không có đường cụt.

## Đặt robot của bạn

- Điểm đầu: `translation -34.1 -13.8 ...`, heading `+pi/2` (Bắc).
- Điểm đích: khoảng `(34.1, 6)`, heading `-pi/2` (Nam).
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
| `tools/expand_world.py` | Script mở rộng bố cục một lần; không chạy lại trong quy trình đồng bộ thường ngày |
| `map.json` | Hình học đường, junction, làn có hướng, điểm đầu/đích |
| `map.svg` | Sơ đồ nhìn từ trên, có ID để tra trong map |
| `controllers/traffic_signals/traffic_signals.py` | Một đồng hồ chung cho bốn hướng đèn |
| `controllers/world_inspector/world_inspector.py` | Kiểm tra mô phỏng; mặc định chỉ chờ |
| `protos/vendor/` | PROTO R2025a và texture đã tải về, giữ license trong từng file |
| `validation/` | Báo cáo và ảnh kiểm tra; xem hash để biết đúng phiên bản |

Chạy `python webots/tools/build_world.py` từ gốc repository để đồng bộ world/map.
World đã lưu là nguồn hình học: lệnh giữ hầm/cầu và các đối tượng chỉnh trong GUI,
chỉ dựng lại junction, bó vỉa junction, barrier và dữ liệu làn. Sao lưu trước khi đổi bố cục lớn.
`tools/sync_world.py` đọc world hiện tại; `build_world.py` gọi quy trình đồng bộ này.
`expand_world.py` đã dùng để tạo bố cục mở rộng, không phải lệnh cần chạy mỗi lần chỉnh world.
Không cần sinh lại để mở world. `make_signs.py` chỉ cần khi muốn tạo lại texture biển,
dùng Pillow và font Arial của Windows. Generator dùng thêm Shapely (đã dùng trong kiểm tra hình học)
để offset biên barrier và loại bỏ chỗ tự cắt tại đoạn cong/giao lộ.

Để mở offline, world chính tham chiếu các PROTO local trong `protos/vendor`.
`tools/fetch_assets.py` dùng khi cần lấy lại asset, không cần chạy mỗi lần mở world.
Các PROTO dùng tham chiếu tài nguyên local. Riêng `Road.proto` có hai sửa lỗi cục bộ:
nhóm texture `startLine`/`endLine` theo đúng làn hiện tại, và cho vạch bổ sung kế thừa
độ rộng của vạch đã cấu hình khi tăng `numberOfLanes`. Giữ các sửa này khi tải lại asset.
`StraightRoadSegment.proto` còn chuyển tiếp ID, junction hai đầu và số làn chiều thuận
vào `Road` bên trong; giữ các binding này để metadata vẫn đúng.
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

Trong `map.json`, `_F1` đến `_F2` hoặc `_F4` là các làn chiều thuận, `_B1` đến `_B2` hoặc `_B4` chiều ngược,
tùy đường có bốn hay tám làn. Làn 1 sát tim; làn có chỉ số lớn nhất sát lề. Mỗi làn tách thành `_entry`/`_exit` với vùng
chuyển làn trên đường (`CHANGE_*`), trước khi đến giao lộ. Tại giao lộ, đi thẳng giữ
thứ tự làn, rẽ trái từ làn trong và rẽ phải từ làn ngoài; không tự thêm quay đầu.
Các mục có `junction` là đường nối; `successors` chứa ID đi tiếp. Vòng xuyến dùng `RING_*` một chiều.
`roads[].elevations` lưu cao độ của từng waypoint, phân biệt hầm và cầu khi đường cắt nhau trên hình chiếu XY.
`speed_limit_m_s` chỉ là tham số map, không tự hãm xe trong mô phỏng.

Các node đường có `name` rõ ràng và không khai báo DEF riêng. ID trong map giữ nguyên để tra cứu.
Một số node khác như `SIGNAL_SYSTEM` vẫn cần DEF để controller kiểm tra tham chiếu.

Nền đỡ xe ở các đường thường; hầm/cầu và hai đường chữ S cao phía Bắc có collider mặt đường riêng.
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
  0,186 m của 1578 đoạn làn/đường nối nằm trong mặt đường. Không kiểm điều khiển động học.
- Kiểm tra thêm vạch dừng trên từng hướng vào, không có vạch dừng ở vòng xuyến,
  barrier khép kín, không tự cắt và không lấn mặt đường/giao lộ.
- `check_road_template.cjs` thực thi JavaScript thực tế trong `Road.proto`, kiểm tra
  độ rộng vạch khi đổi 4/5/6/7/8 làn trên đường thẳng/cong, bề rộng texture đầu/cuối đường và binding junction.
  Chỉnh số làn trong GUI không tự cập nhật `map.json` hay vị trí vạch vàng;
  chạy lại lệnh đồng bộ để map phản ánh cấu hình bốn/tám làn đã lưu.
- `verify_junctions.py` sao chép project vào Temp, chạy Webots thu nhỏ với port riêng;
  kiểm tra toàn bộ road/junction và năm probe va chạm: hai ở hầm, một trên cầu,
  hai tại đỉnh đường chữ S phía Bắc; xuất hai ảnh rồi thoát.
  Report có nonce/hash xác nhận đúng lần chạy; không đóng phiên Webots đang dùng.
  `verify_world.py` là verifier cũ với probe cố định, không dùng làm gate cho bố cục mới.
- `junction-runtime.json` và `geometry.json` ghi hash của world. Chỉ dùng báo cáo có hash
  khớp world hiện tại. `junction-overview.png` là toàn cảnh theo bounds mới;
  `junction-west-detail.png` là ảnh chi tiết phía Tây. `runtime.json` và ảnh từ verifier cũ
  không xác nhận bản world mới.

Đây là kiểm chứng môi trường, không phải kiểm chứng xe đã tự lái thành công.

## Trạng thái hiện tại

Kiểm tra tĩnh hiện bao phủ 55 đường, 33 junction, 102 miệng đường, 2594 tam giác và 1578 đoạn làn.
Verifier runtime kiểm tra metadata native và năm vật thử tĩnh trên hầm, cầu và hai đường Bắc.
Xem `validation/geometry.json`, `validation/junction-runtime.json` và hai ảnh `junction-*.png`.
Đây là kiểm tra môi trường và vật thử tĩnh, chưa kiểm chứng robot chạy qua toàn bộ tuyến.
