# Algorithm Notes

## Pure Pursuit

Pure Pursuit는 현재 차량 위치에서 전방 lookahead 거리만큼 떨어진 target waypoint를 선택하고, 차량 좌표계 기준 target point를 이용해 조향각을 계산합니다. 본 프로젝트에서는 속도에 따라 lookahead를 동적으로 늘리고, 경로 곡률과 현재 조향각을 함께 사용해 속도를 제한했습니다.

## Gap Following

Gap Following은 LiDAR scan에서 안전하게 주행 가능한 공간을 찾는 reactive 알고리즘입니다. 본 프로젝트에서는 disparity extension과 safety bubble을 함께 사용하여 장애물 경계를 보수적으로 확장하고, 선택된 gap 중심을 향해 조향했습니다.

## Localization

AMCL 기반 localization을 사용하여 map 좌표계에서 차량 pose를 추정했습니다. F1TENTH는 omnidirectional platform이 아니므로 `DifferentialMotionModel`을 사용하도록 설정했습니다.
