# viz_pkg - 簡單 ROS 平面顯示套件

最簡單的 ROS 套件，用於在 rviz 中顯示平面。

## 功能特色

- ✅ **簡單配置文件**: 預先定義位置與大小
- ✅ **手動輸入**: 支援指令行參數輸入
- ✅ **動態更新**: 透過 ROS topic 更新位置
- ✅ **正確的平面建立**: 以位置為中心，x,y 方向作為 a,b 大小，z 方向固定厚度 0.005m

## 使用方法

### 1. 基本啟動
```bash
roslaunch viz_pkg plane_visualizer.launch
```

### 2. 自訂參數啟動
```bash
roslaunch viz_pkg plane_visualizer.launch plane_a:=3.0 plane_b:=2.0 plane_x:=1.0 plane_y:=2.0 plane_z:=0.5
```

### 3. 手動輸入 (指令行)
```bash
# 只設定位置
rosrun viz_pkg plane_visualizer.py 1.0 2.0 0.5

# 設定位置和尺寸
rosrun viz_pkg plane_visualizer.py 1.0 2.0 0.5 3.0 2.0
```

### 4. 動態位置更新
```bash
rostopic pub /plane_pose geometry_msgs/PoseStamped "header:
  frame_id: 'world'
pose:
  position: {x: 1.0, y: 2.0, z: 0.5}"
```

### 5. 測試功能
```bash
rosrun viz_pkg test_simple.py
```

## 配置文件

配置文件位於 `config/plane_config.yaml`：
```yaml
position:
  x: 0.0
  y: 0.0
  z: 0.0

size:
  a: 2.0
  b: 1.5
```

## 參數說明

- `plane_a`: X 方向大小 (預設: 2.0m)
- `plane_b`: Y 方向大小 (預設: 1.5m)
- `plane_x`: X 座標 (預設: 0.0m)
- `plane_y`: Y 座標 (預設: 0.0m)
- `plane_z`: Z 座標 (預設: 0.0m)

## ROS 主題

- `/plane_marker`: 發布平面標記 (visualization_msgs/Marker)
- `/plane_pose`: 接收位置更新 (geometry_msgs/PoseStamped)

## 重要特性

### 平面建立方式
- **位置為中心**: 輸入的 (x,y,z) 座標為平面的中心點
- **尺寸定義**: a 為 X 方向大小，b 為 Y 方向大小
- **固定厚度**: Z 方向厚度固定為 0.005m (不可改變)
- **座標系**: 使用 "world" 座標系

### 使用 rostopic 指令：
```bash
rostopic pub /plane_pose geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position: {x: 1.0, y: 2.0, z: 0.5}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

## 參數說明

- `frame_id`: 座標系 (預設: "map")
- `plane_width`: 平面寬度 a (預設: 2.0)
- `plane_height`: 平面高度 b (預設: 1.5)
- `plane_thickness`: 平面厚度 (預設: 0.05)
- `plane_x`: 初始 X 座標 (預設: 0.0)
- `plane_y`: 初始 Y 座標 (預設: 0.0)
- `plane_z`: 初始 Z 座標 (預設: 0.0)

## ROS 主題

### 發布的主題：
- `/plane_marker` (visualization_msgs/Marker): 平面標記資訊

### 訂閱的主題：
- `/plane_pose` (geometry_msgs/PoseStamped): 接收新的平面位置

## 檔案結構

```
viz_pkg/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── README.md
├── scripts/
│   ├── plane_visualizer.py      # 主要節點
│   └── test_plane_position.py   # 測試腳本
├── launch/
│   ├── plane_visualizer.launch  # 完整啟動檔案
│   └── plane_only.launch        # 僅節點啟動檔案
├── rviz/
│   └── plane_visualization.rviz # rviz 設定檔
└── src/
    └── viz_pkg/
        └── __init__.py
```

## 故障排除

1. 如果看不到平面，請檢查：
   - rviz 中的 Fixed Frame 是否設為 "map"
   - Marker 顯示是否已啟用
   - 平面位置是否在視野範圍內

2. 如果節點無法啟動，請檢查：
   - 是否已正確編譯套件
   - 是否已載入環境變數 (source devel/setup.bash)
   - Python 腳本是否有執行權限

## 範例使用情境

1. 顯示一個 3×2 的平面在位置 (1, 2, 0.5)：
```bash
roslaunch viz_pkg plane_visualizer.launch plane_width:=3.0 plane_height:=2.0 plane_x:=1.0 plane_y:=2.0 plane_z:=0.5
```

2. 動態移動平面到新位置：
```bash
rosrun viz_pkg test_plane_position.py 5.0 3.0 1.0
```
