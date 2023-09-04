import asyncio
import time
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

import math
import influxdb_client
from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from pymavlink import mavutil

token="X"
org = "Y"
url = "http://localhost:8086"
write_client = InfluxDBClient(url=url, token=token, org=org)
bucket="Z"
write_api = write_client.write_api(write_options=SYNCHRONOUS)

# -----------------------------------------------------------------------------------------------------------

async def flight_time(start_time, drone):
    async for _ in drone.telemetry.in_air():
        end = time.time()
        fly_time = int(end - start_time)
        #print(f"Flight Time (s): {fly_time}")
        p = influxdb_client.Point("HUMS").field("Flight Time", fly_time)
        write_api.write(bucket=bucket, org=org, record=p)

async def print_battery(drone):
    async for battery in drone.telemetry.battery():
        batP= battery.remaining_percent*100
        #print(f"Battery (%): {batP}")
        p = influxdb_client.Point("HUMS").field("Battery (%)", batP)
        write_api.write(bucket=bucket, org=org, record=p)

        BatV= battery.voltage_v
        #print(f"Battery (V): {BatV}")
        p = influxdb_client.Point("HUMS").field("Battery (V)", BatV)
        write_api.write(bucket=bucket, org=org, record=p)

async def imu(drone):
    async for imu in drone.telemetry.imu():
        #print(f"Temperature (C): {imu.temperature_degc}")
        p = influxdb_client.Point("HUMS").field("Temp", imu.temperature_degc)
        write_api.write(bucket=bucket, org=org, record=p)

        #print(f"Front Accel (m/s2): {imu.acceleration_frd.forward_m_s2}")
        p = influxdb_client.Point("HUMS").field("Front Accel", imu.acceleration_frd.forward_m_s2)
        write_api.write(bucket=bucket, org=org, record=p)

        #print(f"Right Accel (m/s2): {imu.acceleration_frd.right_m_s2}")
        p = influxdb_client.Point("HUMS").field("Right Accel", imu.acceleration_frd.right_m_s2)
        write_api.write(bucket=bucket, org=org, record=p)

        #print(f"Down Accel (m/s2): {imu.acceleration_frd.down_m_s2}")
        p = influxdb_client.Point("HUMS").field("Down Accel", imu.acceleration_frd.down_m_s2)
        write_api.write(bucket=bucket, org=org, record=p)

        accel = math.sqrt((imu.acceleration_frd.forward_m_s2*imu.acceleration_frd.forward_m_s2)+(imu.acceleration_frd.right_m_s2*imu.acceleration_frd.right_m_s2)+(imu.acceleration_frd.down_m_s2*imu.acceleration_frd.down_m_s2))
        #print(f"Accel Module (m/2s): {accel}")
        p = influxdb_client.Point("HUMS").field("Accel Module", accel)
        write_api.write(bucket=bucket, org=org, record=p)

async def air_pressure(drone):
    async for scaled_pressure in drone.telemetry.scaled_pressure():
        #print(f"Air Pressure (hPa): {scaled_pressure.absolute_pressure_hpa}")
        p = influxdb_client.Point("HUMS").field("Air Pressure", scaled_pressure.absolute_pressure_hpa)
        write_api.write(bucket=bucket, org=org, record=p)

async def ned_velocity(drone):
    async for velocity_ned in drone.telemetry.velocity_ned():
        #print(f"North Velocity (m/s): {velocity_ned.north_m_s}")
        nv= velocity_ned.north_m_s
        p = influxdb_client.Point("HUMS").field("North Velocity", velocity_ned.north_m_s)
        write_api.write(bucket=bucket, org=org, record=p)

        #print(f"East Velocity (m/s): {velocity_ned.east_m_s}")
        ne= velocity_ned.east_m_s
        p = influxdb_client.Point("HUMS").field("East Velocity", velocity_ned.east_m_s)
        write_api.write(bucket=bucket, org=org, record=p)

        #print(f"Down Velocity (m/s): {velocity_ned.north_m_s}")
        nd= velocity_ned.down_m_s
        p = influxdb_client.Point("HUMS").field("Down Velocity", velocity_ned.down_m_s)
        write_api.write(bucket=bucket, org=org, record=p)

        velocity = math.sqrt((velocity_ned.north_m_s*velocity_ned.north_m_s)+(velocity_ned.east_m_s*velocity_ned.east_m_s)+(velocity_ned.down_m_s*velocity_ned.down_m_s))
        #print(f"Velocity Module (m/s): {velocity}")
        p = influxdb_client.Point("HUMS").field("Velocity Module", velocity)
        write_api.write(bucket=bucket, org=org, record=p)

async def rawgps(drone):
    async for raw_gps in drone.telemetry.raw_gps():
        #print(f"Velocity (m/s): {raw_gps.velocity_m_s}")
        p = influxdb_client.Point("HUMS").field("Velocity", raw_gps.velocity_m_s)
        write_api.write(bucket=bucket, org=org, record=p)

        #print(f"Altitude (m): {raw_gps.absolute_altitude_m}")
        p = influxdb_client.Point("HUMS").field("Altitude", raw_gps.absolute_altitude_m)
        write_api.write(bucket=bucket, org=org, record=p)

        #print(f"Latitude (deg): {raw_gps.latitude_deg}")
        p = influxdb_client.Point("HUMS").field("Latitude", raw_gps.latitude_deg)
        write_api.write(bucket=bucket, org=org, record=p)

        #print(f"Longitude (deg): {raw_gps.longitude_deg}")
        p = influxdb_client.Point("HUMS").field("Longitude", raw_gps.longitude_deg)
        write_api.write(bucket=bucket, org=org, record=p)


async def fixedwingmetrics(drone):
    async for fixedwing_metrics in drone.telemetry.fixedwing_metrics():
        #print(f"Air Speed (m/s): {fixedwing_metrics.airspeed_m_s}")
        p = influxdb_client.Point("HUMS").field("Air Speed (m/s)", fixedwing_metrics.airspeed_m_s)
        write_api.write(bucket=bucket, org=org, record=p)

        #print(f"Throttle Percentage (%): {fixedwing_metrics.throttle_percentage}")
        throttle= fixedwing_metrics.throttle_percentage * 100
        p = influxdb_client.Point("HUMS").field("Throttle (%)", throttle)
        write_api.write(bucket=bucket, org=org, record=p)

        #print(f"Climb Rate (m/s): {fixedwing_metrics.climb_rate_m_s}")
        p = influxdb_client.Point("HUMS").field("Climb Rate (m/s)", fixedwing_metrics.climb_rate_m_s)
        write_api.write(bucket=bucket, org=org, record=p)

async def actuators(drone):
    async for actuator_output_status in drone.telemetry.actuator_output_status():
        print(f"Actuator output: {actuator_output_status.active}")
        p = influxdb_client.Point("HUMS").field("Actuator Output Status", actuator_output_status.actuator)
        write_api.write(bucket=bucket, org=org, record=p)

async def print_gps_info(drone):
    async for gps_info in drone.telemetry.gps_info():
        print(f"GPS info: {gps_info}")

async def print_in_air(drone):
    async for in_air in drone.telemetry.in_air():
        print(f"In air: {in_air}")

async def actuators(drone):
    async for actuator_output_status in drone.telemetry.actuator_output_status():
        print(f"Actuator output: {actuator_output_status.active}")
        p = influxdb_client.Point("HUMS").field("Actuator Value", actuator_output_status.actuator)
        write_api.write(bucket=bucket, org=org, record=p)

async def mavlinkmessages(connection, drone):
    async for _ in drone.telemetry.position():
        msg = connection.recv_match(type=["ESC_STATUS", "ESC_INFO", "VFR_HUD"], blocking=False)
        
        msg_type = msg.get_type()

        if msg_type == "ESC_INFO":
            #is any esc failing:
            esc_fail0 = msg.failure_flags[0]
            #print(esc_fail0)
            p = influxdb_client.Point("HUMS").field("esc_fail0", esc_fail0)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_fail1 = msg.failure_flags[1]
            #print(esc_fail1)
            p = influxdb_client.Point("HUMS").field("esc_fail1", esc_fail1)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_fail2 = msg.failure_flags[2]
            #print(esc_fail2)
            p = influxdb_client.Point("HUMS").field("esc_fail2", esc_fail2)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_fail3 = msg.failure_flags[3]
            #print(esc_fail3)
            p = influxdb_client.Point("HUMS").field("esc_fail3", esc_fail3)
            write_api.write(bucket=bucket, org=org, record=p)

            #how many times did thay fail:
            esc0_fails = msg.error_count[0]
            #print(esc_fail0)
            p = influxdb_client.Point("HUMS").field("esc0_fails", esc0_fails)
            write_api.write(bucket=bucket, org=org, record=p)

            esc1_fails = msg.error_count[1]
            #print(esc_fail1)
            p = influxdb_client.Point("HUMS").field("esc1_fails", esc1_fails)
            write_api.write(bucket=bucket, org=org, record=p)

            esc2_fails = msg.error_count[2]
            #print(esc_fail2)
            p = influxdb_client.Point("HUMS").field("esc2_fails", esc2_fails)
            write_api.write(bucket=bucket, org=org, record=p)

            esc3_fails = msg.error_count[3]
            #print(esc_fail3)
            p = influxdb_client.Point("HUMS").field("esc3_fails", esc3_fails)
            write_api.write(bucket=bucket, org=org, record=p)

            #each esc temperatures:
            
            esc_temperature0 = msg.temperature[0]
            #print(esc_temperature0)
            p = influxdb_client.Point("HUMS").field("esc_temperature0", esc_temperature0)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_temperature1 = msg.temperature[1]
            #print(esc_temperature1)
            p = influxdb_client.Point("HUMS").field("esc_temperature1", esc_temperature1)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_temperature2 = msg.temperature[2]
            #print(esc_temperature2)
            p = influxdb_client.Point("HUMS").field("esc_temperature2", esc_temperature2)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_temperature3 = msg.temperature[3]
            #print(esc_temperature3)
            p = influxdb_client.Point("HUMS").field("esc_temperature3", esc_temperature3)
            write_api.write(bucket=bucket, org=org, record=p)

        elif msg_type == "VFR_HUD":
            groundspeed = msg.groundspeed
            #print(groundspeed)
            p = influxdb_client.Point("HUMS").field("groundspeed", groundspeed)
            write_api.write(bucket=bucket, org=org, record=p)

            heading = msg.heading
            #print(heading)
            p = influxdb_client.Point("HUMS").field("heading", heading)
            write_api.write(bucket=bucket, org=org, record=p)

        else:      
            esc_rpm0 = msg.rpm[0]
            #print(esc_rpm0)
            p = influxdb_client.Point("HUMS").field("esc_rpm0", esc_rpm0)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_rpm1 = msg.rpm[1]
            #print(esc_rpm1)
            p = influxdb_client.Point("HUMS").field("esc_rpm1", esc_rpm1)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_rpm2 = msg.rpm[2]
            #print(esc_rpm2)
            p = influxdb_client.Point("HUMS").field("esc_rpm2", esc_rpm2)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_rpm3 = msg.rpm[3]
            #print(esc_rpm3)
            p = influxdb_client.Point("HUMS").field("esc_rpm3", esc_rpm3)
            write_api.write(bucket=bucket, org=org, record=p)


            esc_voltage0 = msg.voltage[0]
            #print(esc_voltage0)
            p = influxdb_client.Point("HUMS").field("esc_voltage0", esc_voltage0)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_voltage1 = msg.voltage[1]
            #print(esc_voltage1)
            p = influxdb_client.Point("HUMS").field("esc_voltage1", esc_voltage1)
            write_api.write(bucket=bucket, org=org, record=p)


            esc_voltage2 = msg.voltage[2]
            #print(esc_voltage2)
            p = influxdb_client.Point("HUMS").field("esc_voltage2", esc_voltage2)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_voltage3 = msg.voltage[3]
            #print(esc_voltage3)
            p = influxdb_client.Point("HUMS").field("esc_voltage3", esc_voltage3)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_current0 = msg.current[0]
            #print(esc_current0)
            p = influxdb_client.Point("HUMS").field("esc_current0", esc_current0)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_current1 = msg.current[1]
            #print(esc_current1)
            p = influxdb_client.Point("HUMS").field("esc_current1", esc_current1)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_current2 = msg.current[2]
            #print(esc_current2)
            p = influxdb_client.Point("HUMS").field("esc_current2", esc_current2)
            write_api.write(bucket=bucket, org=org, record=p)

            esc_current3 = msg.current[3]
            #print(esc_current3)
            p = influxdb_client.Point("HUMS").field("esc_current3", esc_current3)
            write_api.write(bucket=bucket, org=org, record=p)


# -----------------------------------------------------------------------------------------------------------

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540") 

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

# -----------------------------------------------------------------------------------------------------------

    # Start a mavutil connection
    connection = mavutil.mavlink_connection('udpin:localhost:14550', dialect="common")

    # Wait for the connection confirmation 
    connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))


# -----------------------------------------------------------------------------------------------------------


    print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone))
    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

    mission_items = []
    mission_items.append(MissionItem(47.398039859999997,
                                     8.5455725400000002,
                                     25,
                                     10,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(47.398036222362471,
                                     8.5450146439425509,
                                     25,
                                     10,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(47.397825620791885,
                                     8.5450092830163271,
                                     25,
                                     10,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))

    mission_plan = MissionPlan(mission_items)

    await drone.mission.set_return_to_launch_after_mission(True)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Starting mission")
    
    await drone.mission.start_mission()

    # -------------------------------------------------------------------
    start_time = time.time()

    asyncio.ensure_future(flight_time(start_time, drone))     
    asyncio.ensure_future(print_battery(drone))
    asyncio.ensure_future(imu(drone))       
    asyncio.ensure_future(air_pressure(drone))    
    asyncio.ensure_future(ned_velocity(drone))
    asyncio.ensure_future(rawgps(drone))
    asyncio.ensure_future(fixedwingmetrics(drone))
    asyncio.ensure_future(mavlinkmessages(connection, drone))

    #asyncio.ensure_future(actuators(drone))
    #asyncio.ensure_future(print_gps_info(drone))
    #asyncio.ensure_future(print_in_air(drone))
    #asyncio.ensure_future(print_position(drone))

    # -------------------------------------------------------------------

    await termination_task


async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
