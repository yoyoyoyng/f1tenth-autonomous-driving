# Nav2 Minimal Stack

F1TENTH 실험에서 localization과 global planning을 검증하기 위해 사용한 Nav2 경량 구성입니다.

## Components

- `map_server`
- `amcl`
- `planner_server`
- `lifecycle_manager`
- global costmap only launch

## Important Configuration

F1TENTH는 omnidirectional platform이 아니므로 AMCL motion model은 다음과 같이 정리했습니다.

```yaml
robot_model_type: "nav2_amcl::DifferentialMotionModel"
```

맵 데이터는 프로젝트별 폴더가 아니라 저장소 루트의 `resources/maps`에 보관합니다.

## Files

```text
config/
├── nav2_params.yaml
└── mapper_params_online_async.yaml
launch/
└── global_costmap_only_4nodes.launch.py
scripts/
└── logger_node_amcl.py
```

## Demo

[![Localization Demo](../../media/demos/localization_demo.gif)](../../media/demos/localization_demo.mp4)
