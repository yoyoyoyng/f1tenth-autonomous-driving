# Gap Following / Disparity Extension

LiDAR scan 기반 reactive obstacle avoidance 프로젝트입니다. F1TENTH 차량 폭과 safety margin을 고려하여 장애물 주변을 확장하고, 가장 안전한 gap을 선택해 `/drive` 명령을 생성합니다.

## Demo

### Obstacle Avoidance

[![Gap Following Obstacle](../../media/demos/gap_follow_obstacle.gif)](../../media/demos/gap_follow_obstacle.mp4)

### Track Driving

[![Gap Following Track](../../media/demos/gap_follow_track.gif)](../../media/demos/gap_follow_track.mp4)

## Core Logic

- `/scan` LiDAR 데이터 전처리
- 전방 FOV 영역만 사용
- nearest obstacle 기준 safety bubble 적용
- disparity extension으로 장애물 경계 확장
- 가장 넓은 safe gap 중심 선택
- 조향 smoothing, deadzone, rate limit 적용
- 전방 거리와 조향각 기반 속도 제어
- RViz marker로 best point와 bubble point 시각화

## Main Code

```text
gap_following/
└── gap_follow_node.py
```

실행:

```bash
ros2 run gap_following gap_follow_node
```

## Notes

맵 이미지는 이 프로젝트 폴더에 넣지 않았습니다. Gap Following은 map을 사용하지 않는 LiDAR 기반 reactive driving 알고리즘이기 때문에, 공통 map 데이터는 저장소 루트의 `resources/maps`에 분리했습니다.
