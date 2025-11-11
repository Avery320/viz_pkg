# viz_pkg — STL 環境載入、MoveIt 碰撞物件、RViz 可視化與 PlanningScene 轉發

提供將 STL 模型載入至 MoveIt PlanningScene（碰撞物件）與 RViz（Marker 顯示），支援多檔載入、檔名自動生成 id、一次性載入後自動結束。

## 架構
- 核心模組（src/viz_pkg/src）
  - rviz_marker.py：RViz Marker 週期性發佈（MESH_RESOURCE 與 TRIANGLE_LIST）
  - mesh_loader.py：STL 三角形載入工具（trimesh / numpy-stl / ASCII）
  - collision_obj.py：MoveIt PlanningSceneInterface 新增/移除碰撞物件
  - obj_loader.py：整合載入、參數處理、建立 Marker 與 Collision 物件
- 腳本（src/viz_pkg/scripts）
  - viz_obj.py：常駐節點；載入 STL、持續發 /env_marker、轉發 PlanningScene 至 /planning_scene/scene（latched），啟動即呼叫 /get_planning_scene 發佈初始快照
  - clear_env.py：清除 MoveIt 物件與 RViz 標記（支援全部/單一 id）
  - list_models.py：列出 MoveIt world 物件 id
  - sphere_marker.py：發佈測試用球體 Marker（選用）

## Topics
- RViz Marker（依 ~marker_mode）：
  - resource：/MarkerRes/<id>（type=10 MESH_RESOURCE；可用 ~marker_topic 覆寫為單一自訂 topic，如 /env_marker）
  - triangles：/MarkerTri/<id>（type=11 TRIANGLE_LIST）
  - both：同時發佈上述兩種 topic
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
- 只發布 Marker（不加入碰撞），並自訂單一 topic（resource 模式）
```bash
rosrun viz_pkg viz_obj.py _mesh_path:=modol/3dcp_ws.stl _publish_markers:=true _add_collision:=false _marker_mode:=resource _marker_topic:=/env_marker
```
- 同時發佈 MESH_RESOURCE 與 TRIANGLE_LIST（雙軌）
```bash
rosrun viz_pkg viz_obj.py _mesh_path:=modol/3dcp_ws.stl _publish_markers:=true _add_collision:=false _marker_mode:=both
# 預設 topic：/MarkerRes/<id> 與 /MarkerTri/<id>
```
- 只加入碰撞（不發 Marker）
```bash
rosrun viz_pkg viz_obj.py _mesh_path:=modol/3dcp_ws.stl _publish_markers:=false _add_collision:=true
```

## 清除 / 查詢

```bash
rosrun viz_pkg clear_env.py _object_id:=你的id   # 移除單一物件
rosrun viz_pkg clear_env.py _object_id:=all     # 移除全部
```

說明：
- 指令會同時清除 MoveIt 世界物件與 RViz 標記兩條軌：/MarkerRes/<id>（ns=<id>）、/MarkerTri/<id>（ns=<id>_tri）。
- 當 _object_id:=all 時，會自動掃描目前 ROS master 上所有 /MarkerRes/* 與 /MarkerTri/* 的 visualization_msgs/Marker topic，逐一送出 DELETEALL。
- 若你的 resource 軌有使用自訂 ~marker_topic，請搭配 _topic 覆寫以一併清除。

- 列出 MoveIt world 物件：
```bash
rosrun viz_pkg list_models.py
```

## 主要參數
- ~mesh_path：單一路徑；支援套件相對路徑或 package:// URI
- ~mesh_paths：多檔（逗號字串或 list）
- ~frame_id：（預設 world）
- ~scale：（預設 [0.001, 0.001, 0.001]，mm→m）
- ~publish_markers：（預設 true）是否發布 RViz Marker
- ~add_collision：（預設 true）是否加入 MoveIt 碰撞物件
- ~marker_mode：（resource｜triangles｜both；預設 resource）
- ~marker_topic：（字串，選填）只覆寫 resource 模式下的 topic；未提供時預設 /MarkerRes/<id>
- ~marker_latched：（預設 true）Marker publisher 是否使用 latched（晚訂閱者也能拿到最後一筆）
- ~marker_use_embedded_materials：（預設 false）
- ~marker_period：（預設 0.1 秒；僅在週期性發布時有意義）

## triangles 模式相依性與注意事項
- 若 STL 為 binary 格式，建議安裝 numpy-stl（或 trimesh）以解析；否則僅 ASCII STL 能載入，可能導致 TRIANGLE_LIST 無幾何顯示。
- 安裝建議：
  - pip3 install numpy-stl
- TRIANGLE_LIST 訊息量較大；建議搭配：
  - _marker_period:=1.0 或更慢
  - ~marker_latched:=true（讓晚訂閱者也能拿到最後一筆）