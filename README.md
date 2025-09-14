# viz_pkg - STL 環境載入與碰撞物件工具

提供將 STL 模型載入至 MoveIt PlanningScene（碰撞物件）與 RViz（Marker 顯示），支援多檔載入、檔名自動生成 id、一次性載入後自動結束。

## 功能
- 載入單/多個 .stl
- 以檔名自動生成唯一 object_id（重複自動加 _2、_3…）
- MoveIt 碰撞物件 + RViz Marker 顯示（topic: /env_marker）
- 預設 Marker 顏色：淺灰 [0.8, 0.8, 0.8, 0.4]，可由參數覆蓋
- 一次性載入 3 秒後自動結束

資源路徑支援：
- package URL，例如 package://viz_pkg/modol/3dcp_ws.stl
- 套件內相對路徑，例如 modol/3dcp_ws.stl（亦相容舊路徑 config/）

## 使用方法

### 1) 單一檔案
```bash
rosrun viz_pkg viz_obj.py _mesh_path:=modol/3dcp_ws.stl
# 或：rosrun viz_pkg viz_obj.py _mesh_path:=package://viz_pkg/modol/3dcp_ws.stl
```

### 2) 多個檔案
- 逗號分隔：
```bash
rosrun viz_pkg viz_obj.py "_mesh_paths:=modol/a.stl,modol/b.stl"
```
- YAML list（搭配 roslaunch）：
```xml
<node pkg="viz_pkg" type="viz_obj.py" name="viz_loader" output="screen">
  <param name="mesh_paths" value="['package://viz_pkg/modol/a.stl','package://viz_pkg/modol/b.stl']"/>
</node>
```

### 3) 指令工具（逐個參數）
```bash
rosrun viz_pkg load_models.py package://viz_pkg/modol/a.stl package://viz_pkg/modol/b.stl
```

### 4) 設定顏色（RViz Marker）
```bash
rosrun viz_pkg viz_obj.py _marker_color:='[1.0, 0.2, 0.2, 0.6]'
```

## 清除 / 查詢
- 移除全部：
```bash
rosrun viz_pkg clear_env.py _object_id:=你的id
rosrun viz_pkg clear_env.py _remove_all:=true
```
- 依 id 移除：
```bash
rosrun viz_pkg remove_models.py table shelf_2
```
- 列出目前 MoveIt world objects：
```bash
rosrun viz_pkg list_models.py
```

## 備註
- 預設 scale 為 [0.001, 0.001, 0.001]（mm → m）
- 所有物件共用 frame_id/pose/scale；若需每物件獨立設定，可擴充 YAML 方案
- 若需讓 PlanningScene 本身顯示為自訂顏色（非預設綠色），可加發 ObjectColor 設定（可依需求加工具）
