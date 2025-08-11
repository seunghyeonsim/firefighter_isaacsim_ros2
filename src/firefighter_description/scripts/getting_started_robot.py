# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # start the simulation app, with GUI open

import sys
import os

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path

# Import our custom FirefighterArticulation class
from firefighter_articulation import FirefighterArticulation

# preparing the scene
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Setup the scene using the new setup_scene function
my_world, firefighter = FirefighterArticulation.setup_scene()

print("\n=== FIREFIGHTER ROBOT COMPREHENSIVE TEST ===")

# Get all flipper positions
print("Current flipper positions:")
flipper_positions = firefighter.get_all_flipper_positions()
for name, pos in flipper_positions.items():
    print(f"  {name}: {pos}")

# ===== 1. 플리퍼 순서대로 들었다 내리기 =====
print("\n=== 1. 플리퍼 순서대로 들었다 내리기 ===")

print("1.1. 왼쪽 앞 플리퍼 들기...")
firefighter.set_front_left_flipper_velocity(2.0)
for j in range(30):
    my_world.step(render=True)
firefighter.set_front_left_flipper_velocity(0.0)
for j in range(10):
    my_world.step(render=True)

print("1.2. 오른쪽 앞 플리퍼 들기...")
firefighter.set_front_right_flipper_velocity(2.0)
for j in range(30):
    my_world.step(render=True)
firefighter.set_front_right_flipper_velocity(0.0)
for j in range(10):
    my_world.step(render=True)

print("1.3. 왼쪽 뒤 플리퍼 들기...")
firefighter.set_rear_left_flipper_velocity(2.0)
for j in range(30):
    my_world.step(render=True)
firefighter.set_rear_left_flipper_velocity(0.0)
for j in range(10):
    my_world.step(render=True)

print("1.4. 오른쪽 뒤 플리퍼 들기...")
firefighter.set_rear_right_flipper_velocity(2.0)
for j in range(30):
    my_world.step(render=True)
firefighter.set_rear_right_flipper_velocity(0.0)
for j in range(10):
    my_world.step(render=True)

print("1.5. 모든 플리퍼 내리기...")
firefighter.set_all_flippers_velocity(-2.0)
for j in range(40):
    my_world.step(render=True)
firefighter.set_all_flippers_velocity(0.0)
for j in range(20):
    my_world.step(render=True)

# ===== 2. 내린 상태에서 주행 테스트 =====
print("\n=== 2. 플리퍼 내린 상태에서 주행 테스트 ===")

print("2.1. 직진...")
firefighter.move_forward_differential(speed=0.3)
for j in range(50):
    my_world.step(render=True)

print("2.2. 정지...")
firefighter.stop_differential()
for j in range(20):
    my_world.step(render=True)

print("2.3. 후진...")
firefighter.move_backward_differential(speed=0.3)
for j in range(50):
    my_world.step(render=True)

print("2.4. 정지...")
firefighter.stop_differential()
for j in range(20):
    my_world.step(render=True)

print("2.5. 좌회전...")
firefighter.turn_left_differential(speed=1.0)
for j in range(50):
    my_world.step(render=True)

print("2.6. 정지...")
firefighter.stop_differential()
for j in range(20):
    my_world.step(render=True)

print("2.7. 우회전...")
firefighter.turn_right_differential(speed=1.0)
for j in range(50):
    my_world.step(render=True)

print("2.8. 정지...")
firefighter.stop_differential()
for j in range(20):
    my_world.step(render=True)

# ===== 3. 플리퍼 -3만큼 내리기 =====
print("\n=== 3. 플리퍼 -3만큼 내리기 ===")

print("3.1. 모든 플리퍼 -3.0으로 내리기...")
firefighter.set_all_flippers_velocity(-3.0)
for j in range(60):
    my_world.step(render=True)

print("3.2. 플리퍼 정지...")
firefighter.set_all_flippers_velocity(0.0)
for j in range(30):
    my_world.step(render=True)

# ===== 4. 플리퍼 내린 상태에서 주행 동작 =====
print("\n=== 4. 플리퍼 내린 상태에서 주행 동작 ===")

print("4.1. 직진...")
firefighter.move_forward_differential(speed=0.4)
for j in range(60):
    my_world.step(render=True)

print("4.2. 정지...")
firefighter.stop_differential()
for j in range(20):
    my_world.step(render=True)

print("4.3. 좌회전...")
firefighter.turn_left_differential(speed=1.2)
for j in range(60):
    my_world.step(render=True)

print("4.4. 정지...")
firefighter.stop_differential()
for j in range(20):
    my_world.step(render=True)

print("4.5. 우회전...")
firefighter.turn_right_differential(speed=1.2)
for j in range(60):
    my_world.step(render=True)

print("4.6. 정지...")
firefighter.stop_differential()
for j in range(20):
    my_world.step(render=True)

print("4.7. 후진...")
firefighter.move_backward_differential(speed=0.4)
for j in range(60):
    my_world.step(render=True)

print("4.8. 최종 정지...")
firefighter.stop_differential()
firefighter.set_all_flippers_velocity(0.0)
for j in range(30):
    my_world.step(render=True)

print("\n🎉 모든 테스트 완료! 🎉")
print("테스트 내용:")
print("1. 플리퍼 순서대로 들었다 내리기")
print("2. 플리퍼 내린 상태에서 주행 테스트")
print("3. 플리퍼 -3만큼 내리기")
print("4. 플리퍼 내린 상태에서 주행 동작")
print("Press Ctrl+C to exit...")

# Keep the simulation running
try:
    while True:
        my_world.step(render=True)
except KeyboardInterrupt:
    print("Exiting simulation...")
    simulation_app.close()
