# Pure Pursuit Path Tracking

F1TENTH 차량이 미리 기록된 waypoint를 따라 주행하도록 구현한 ROS 2 기반 Pure Pursuit controller입니다.

## Demo

### Competition Track

[![Pure Pursuit Competition](../../media/demos/pure_pursuit_competition.gif)](../../media/demos/pure_pursuit_competition.mp4)

### Practice Track

[![Pure Pursuit Practice](../../media/demos/pure_pursuit_practice.gif)](../../media/demos/pure_pursuit_practice.mp4)

## Core Logic

- `map -> base_link` TF 기반 차량 위치 추정
- 현재 차량 위치에서 가장 가까운 waypoint 탐색
- 속도 기반 dynamic lookahead 계산
- lookahead target point를 local frame으로 변환
- Pure Pursuit steering angle 계산
- 경로 곡률 및 조향각 기반 target speed 계산
- RViz marker를 통한 waypoint, target point, steering arc 시각화

## Main Code

```text
pure_pursuit/
├── pure_pursuit_node.py
└── waypoints.csv
```

실행:

```bash
ros2 run pure_pursuit pure_pursuit_node
```

## Notes

원본 실험 코드에서 최종적으로 사용한 Pure Pursuit controller만 남기고 중복 실험 파일은 제거했습니다. 기존 절대경로 waypoint 설정은 패키지 내부 `waypoints.csv`를 참조하도록 정리했습니다.
