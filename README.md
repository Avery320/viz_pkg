# viz_pkg — STL 環境載入、MoveIt 碰撞物件、RViz 可視化與 PlanningScene 轉發

提供將 STL 模型載入至 MoveIt PlanningScene（碰撞物件）與 RViz（Marker 顯示），支援多檔載入、檔名自動生成 id、一次性載入後自動結束。

## 架構
- 核心模組（src/viz_pkg/src）
  - rviz_marker.py：RViz Marker 週期性發佈（MESH_RESOURCE）
  - collision_obj.py：MoveIt PlanningSceneInterface 新增/移除碰撞物件
  - obj_loader.py：整合載入、參數處理、建立 Marker 與 Collision 物件
- 腳本（src/viz_pkg/scripts）
  - viz_obj.py：常駐節點；載入 STL、持續發 /env_marker、轉發 PlanningScene 至 /planning_scene/scene（latched），啟動即呼叫 /get_planning_scene 發佈初始快照
  - clear_env.py：清除 MoveIt 物件與 RViz 標記（支援全部/單一 id）
  - list_models.py：列出 MoveIt world 物件 id
  - sphere_marker.py：發佈測試用球體 Marker（選用）

## Topics
- RViz Marker：/env_marker（visualization_msgs/Marker，type=MESH_RESOURCE，週期性）
- PlanningScene：/planning_scene/scene（moveit_msgs/PlanningScene，latched；啟動時即送出完整快照，之後持續轉發 /move_group/monitored_planning_scene）

## 使用方式
- 啟動常駐載入器（單一檔）
```bash
rosrun viz_pkg viz_obj.py _mesh_path:=modol/3dcp_ws.stl
# 或 package URL：_mesh_path:=package://viz_pkg/modol/3dcp_ws.stl
```
- 多檔（逗號分隔）
```bash
rosrun viz_pkg viz_obj.py "_mesh_paths:=modol/a.stl,modol/b.stl"
```
- 位置（CLI x y z，非 ROS 參數）
```bash
rosrun viz_pkg viz_obj.py 0.2 0.0 0.1
```

### 主要參數（viz_obj.py）
- ~mesh_path：單一路徑；支援套件相對路徑或 package:// URI
- ~mesh_paths：多檔（逗號字串或 list）
- ~frame_id：（預設 world）
- ~scale：（預設 [0.001, 0.001, 0.001]，mm→m）
- ~marker_color：（預設 [0.8, 0.8, 0.8, 0.4]）
- ~marker_use_embedded_materials：（預設 false）
- ~marker_period：（預設 0.1 秒）

## 清除 / 查詢

```bash
rosrun viz_pkg clear_env.py _object_id:=你的id # 移除單一物件
rosrun viz_pkg clear_env.py _object_id:=all # 移除全部
```

- 列出 MoveIt world 物件：
```bash
rosrun viz_pkg list_models.py
```

## 備註
- 支援路徑：套件相對（如 modol/xxx.stl）與 package:// URI
- 若無 MoveIt（moveit_commander 缺失），系統仍可發 RViz Marker，但不會新增碰撞物件
- 預設所有物件共用 frame_id/pose/scale；若需每物件獨立設定，可擴充 YAML/參數結構
