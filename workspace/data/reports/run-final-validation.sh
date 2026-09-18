#!/bin/bash
set -u
W=/home/astribot/TowerGO/workspace
R="$W/data/reports"
C="$W/src/astribot_navigation/scripts/container.sh"
while kill -0 104313 2>/dev/null; do sleep 5; done
BASE_IMAGE=public.ecr.aws/docker/library/ros:humble-ros-base bash "$C" build > "$R/build-final.log" 2>&1 || exit 1
mkdir -p "$R/development/pre-grid-padding"
for s in room corridor obstacles; do mv "$R/acceptance-$s" "$R/development/pre-grid-padding/"; done
mv "$R/public-api-check" "$R/development/pre-grid-padding/"
mv "$R/replay-check" "$R/development/pre-grid-padding/"
cp "$R/IMPLEMENTATION_STATUS.md" "$R/development/pre-grid-padding/"
for s in room corridor obstacles; do
 bash "$C" run python3 /towergo/src/astribot_navigation/scripts/run_suite.py --scenario "$s" --rate 2 --output "/data/reports/acceptance-$s" > "$R/acceptance-$s-console.log" 2>&1 &
done
bash "$C" run bash /towergo/src/astribot_navigation/scripts/test_unit.sh > "$R/unit-container.log" 2>&1 &
wait
bash "$C" run python3 /towergo/src/astribot_navigation/scripts/audit_maps.py /data/reports > "$R/map-audit-console.log" 2>&1
bash "$C" run python3 /towergo/src/astribot_navigation/scripts/test_public_api.py --map /data/reports/acceptance-room/mapping-room-front-42-clean/map.yaml --output /data/reports/public-api-check > "$R/public-api-console.log" 2>&1 &
bash "$C" run python3 /towergo/src/astribot_navigation/scripts/check_replay.py /data/bags/room-input --output /data/reports/replay-check > "$R/replay-console.log" 2>&1 &
wait
python3 "$W/src/astribot_navigation/scripts/summarize.py" "$R"
