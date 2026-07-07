# Waypoint Tools

F1TENTH 차량의 `map -> base_link` TF를 이용하여 waypoint를 기록하고, RViz에서 waypoint marker를 시각화하기 위한 도구입니다.

## Main Files

```text
src/
├── map_waypoints_logger.py
└── waypoint_marker_publisher.py
```

## Role in the Pipeline

```text
Localization / TF
        ↓
Waypoint Recording
        ↓
Waypoint CSV
        ↓
Pure Pursuit Path Tracking
```

기록된 waypoint CSV들은 저장소 루트의 `resources/waypoints`에 분리했습니다.
