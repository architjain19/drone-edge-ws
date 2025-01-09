#!/usr/bin/env python3

import asyncio
from mavsdk import System
import mavsdk.mission_raw


async def main():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    # Import the QGroundControl mission file
    print("-- Importing mission from QGC .plan file")
    mission_file_path = "./bfl-mundhwa.plan"
    mission_data = await drone.mission_raw.import_qgroundcontrol_mission(mission_file_path)

    print(f"Imported {len(mission_data.mission_items)} mission items "
          f"and {len(mission_data.rally_items)} rally items.")

    # Upload the mission
    print("-- Uploading mission to the drone")
    await drone.mission_raw.upload_mission(mission_data.mission_items)

    # Upload rally points (if any)
    if mission_data.rally_items:
        print("-- Uploading rally points")
        await drone.mission_raw.upload_rally_points(mission_data.rally_items)

    # Wait for global position estimate
    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    # Arm the drone
    print("-- Arming the drone")
    await drone.action.arm()

    # Start the mission
    print("-- Starting the mission")
    await drone.mission_raw.start_mission()

    # Monitor mission progress
    await monitor_mission_progress(drone)


async def monitor_mission_progress(drone):
    """Monitor and display mission progress"""
    async for mission_progress in drone.mission_raw.mission_progress():
        print(f"Mission progress: {mission_progress.current}/{mission_progress.total}")

        if mission_progress.current == mission_progress.total:
            print("-- Mission complete!")
            break


if __name__ == "__main__":
    asyncio.run(main())
