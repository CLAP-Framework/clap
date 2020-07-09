# Usage

## Carla with ros bridge
1. Run docker or local binary. Export port to 2000, 2001
1. Start roscore separately
1. Go to ros bridge repo, source `devel/setup.bash` and launch `src/driver/simulators/carla/carla_adapter/launch/server.launch`
1. Go to this repo, source `devel/setup.bash` and launch `src/driver/simulators/carla/carla_adapter/scripts/use_bridge/main.launch`

## Carla with scenario_runner
1. Run docker or local binary. Export port to 2000, 2001
1. Start roscore separately
1. Set `TEAM_CODE_ROOT` to `src/driver/simulators/carla/carla_adapter/scripts/use_srunner`
1. Run srunner command: `python ${ROOT_SCENARIO_RUNNER}/srunner/challenge/challenge_evaluator_routes.py --scenarios=${ROOT_SCENARIO_RUNNER}/srunner/challenge/all_towns_traffic_scenarios1_3_4.json --agent=${TEAM_CODE_ROOT}/ZZZAgent.py`

> See `zzz/src/driver/simulators/carla/carla_adapter/scripts/server.tmuxp.yaml` for details
