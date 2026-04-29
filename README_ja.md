# MoveIt Trace Motion Planner

MoveIt2 Humble 向けの Trace motion planner 実装です。  
この repository は ROS 2 / MoveIt2 の実行環境を含まず、別の ROS workspace で build / test する前提です。

## 現在の位置づけ

これは論文手法の完全再現ではなく、MoveIt2 planning plugin として Trace motion を検証するための実装です。

現在実装済み:

- `planning_interface::PlannerManager` plugin
- planner id: `TraceMotion`
- start / goal のロボット形状から Trace waypoint を生成
- final pose goal は MoveIt 標準 IK
- intermediate waypoint は position + direction IK
- intermediate IK 失敗時は MoveIt full-pose IK に fallback
- UR5e / Franka Research 3 用 example launch
- Shift motion は設定のみ、default off

未実装:

- Shift motion
- prioritized IK
- null-space IK
- path pruning
- spline smoothing
- OMPL fallback

## 参考文献

Masanori Sekiguchi and Naoyuki Takesue, "Motion Planning for Redundant Articulated Robots Based on Geometrical Properties of the Whole Robot Body in Initial and Goal Configuration", Journal of the Robotics Society of Japan, 40(2), 154-161, 2022.  
https://doi.org/10.7210/jrsj.40.154

## 論文寄り default と実用寄り option

現在の default は論文寄りです。

```yaml
trace_planner:
  waypoint_source:
    add_offset_waypoints: false

  search:
    enable_shortcut_search: false
```

この設定では、基本的に論文の `G1, I1, G2, I2...` の順で waypoint を増やす累積 Trace path を試します。  
offset 候補や shortcut 的な探索は使いません。

UR5e で障害物回避を通しやすくしたい場合は、実用寄り option を有効にします。

```yaml
trace_planner:
  waypoint_source:
    add_offset_waypoints: true
    offset_distance: 0.08

  search:
    enable_shortcut_search: true
```

この設定では、各 Trace waypoint の周辺に deterministic な offset 候補を追加し、単一 waypoint や start/goal waypoint pair も試します。  
これは論文そのものではありませんが、UR5e のような 6DOF ロボットで検証しやすくするための実験的な補助です。

UR5e example では、この実用寄り option を default にしています。

```yaml
trace_planner:
  waypoint_source:
    add_offset_waypoints: true
    offset_distance: 0.12

  search:
    enable_shortcut_search: true
```

`offset_distance` を `0.08` から `0.12` にすると、各 Trace waypoint の周辺候補がより外側に生成されます。  
箱の近くをかすめる候補ではなく、箱から少し離れた候補も試せるため、UR5e のように候補数と冗長性が少ないロボットでは成功率が上がることがあります。  
一方で、offset を大きくしすぎると軌道が大回りになったり、IK が解けにくくなる可能性があります。

## 論文手法との差分

現在の実装は論文の考え方を取り入れていますが、完全再現ではありません。

近い点:

- start / goal のロボット全体形状から waypoint 候補を作る
- `G1, I1, G2, I2...` の順で Trace waypoint を使う
- 関節空間のランダム探索は使わない
- intermediate waypoint では position + direction IK を使う
- Shift motion は本体ではなく optional 扱い

主な違い:

- waypoint は MoveIt の link transform から生成しており、論文の「各関節位置 + 各リンク方向」を厳密に再現しているわけではありません。
- direction IK は tip link の Z 軸方向を使った DLS numerical IK です。論文中の方向誤差表現に近づけていますが、優先度付き IK ではありません。
- intermediate IK では、position + direction IK が失敗した場合に MoveIt full-pose IK へ fallback します。これは実用上の補助で、論文そのものの手順ではありません。
- UR5e example では offset waypoint と shortcut search を default で有効にしています。これは 6DOF ロボットで検証しやすくするための実験的補助で、論文手法そのものではありません。
- Shift motion、prioritized IK、null-space IK はまだ未実装です。
- path pruning / smoothing はまだ未実装です。
- 解発見保証はありません。これは論文でも今後の課題として述べられている点ですが、現実装は Shift motion が無い分さらに失敗しやすいです。

したがって、この repository の現状は次の位置づけです。

```text
Trace motion inspired MoveIt2 plugin prototype
```

論文再現度を上げる次の候補:

1. waypoint 生成を「関節位置 + リンク方向」にさらに寄せる
2. direction IK の誤差・Jacobian を論文の方向誤差により忠実にする
3. Shift motion の joint sampling 版を追加する
4. FR3 / 7DOF で評価する
5. prioritized IK / null-space IK を追加する

## UR5e で失敗しやすい理由

UR5e は 6DOF なので、7DOF ロボットより冗長性が少ないです。  
論文では 7DOF ロボットを主に使い、Trace motion で避けられない場合に Shift motion / prioritized IK を併用します。

この実装では Shift motion がまだないため、UR5e で障害物を置くと以下のように失敗しやすいです。

```text
Trace planner failed: collision-free path not found
```

特に、start state または goal state が collision している場合は避けられません。  
Trace planner の前提は基本的に次です。

```text
start state: collision-free
goal state: collision-free
start -> goal の途中を Trace waypoint で避ける
```

## ログの読み方

例:

```text
Trace planner exhausted accumulated waypoint candidates:
attempted=12, ik_failures=7, collision_failures=5,
start_candidates=6, goal_candidates=6, max_waypoints=12
```

意味:

- `attempted`: 実際に試した waypoint 数
- `ik_failures`: intermediate IK が解けなかった数
- `collision_failures`: IK は解けたが path collision した数
- `start_candidates`: start 形状から作った waypoint 候補数
- `goal_candidates`: goal 形状から作った waypoint 候補数
- `max_waypoints`: 今回試せる waypoint 上限

論文寄り default では UR5e の候補はだいたい start 6 + goal 6 です。  
offset を有効にすると候補数は増えます。

## 実機なし UR5e

Terminal 1:

```bash
ros2 launch moveit_trace_motion_planner trace_ur5e_fake_hardware.launch.py
```

Terminal 2:

```bash
ros2 launch moveit_trace_motion_planner trace_ur5e.launch.py ur_type:=ur5e launch_rviz:=true
```

RViz MotionPlanning panel で以下を選びます。

```text
Planning Group: ur_manipulator
Planning Pipeline: trace_motion_planner
Planner: TraceMotion
```

## FR3

Franka Research 3 は 7DOF なので、Trace motion の評価対象としては UR5e より自然です。

```bash
ros2 launch moveit_trace_motion_planner trace_fr3.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

Shift motion はまだ default off / 未実装ですが、将来的な評価対象は FR3 の方が適しています。

## よくある警告

以下は今回の planning plugin 検証では基本的に無視できます。

```text
Action server: /recognize_objects not available
No 3D sensor plugin(s) defined for octomap updates
namespace collision has occurred with plugin factory
```

以下は `/joint_states` の timestamp / fake hardware 側の問題です。

```text
latest received state has time 0.000000
couldn't receive full current joint state within 1s
```

この場合は fake hardware を先に起動し、`/joint_states` が流れていることを確認してから MoveIt を起動してください。

```bash
ros2 topic echo /joint_states --once
```
