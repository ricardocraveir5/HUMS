#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <thread>

#include <iomanip>
#include <fstream>

#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>


using namespace mavsdk;

using std::chrono::seconds;
using std::this_thread::sleep_for;

static void send_raw_rpm(MavlinkPassthrough& mavlink_passthrough);      // the PX4 callback does not happen
static void send_efi_status(MavlinkPassthrough& mavlink_passthrough);   // the PX4 callback does not happen
static void send_esc_status(MavlinkPassthrough& mavlink_passthrough);
static void send_esc_info(MavlinkPassthrough& mavlink_passthrough);
static void send_vfr_hud(MavlinkPassthrough& mavlink_passthrough);

Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    float gimbal_pitch_deg,
    float gimbal_yaw_deg,
    Mission::MissionItem::CameraAction camera_action)
{
    Mission::MissionItem new_item{};
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.speed_m_s = speed_m_s;
    new_item.is_fly_through = is_fly_through;
    new_item.gimbal_pitch_deg = gimbal_pitch_deg;
    new_item.gimbal_yaw_deg = gimbal_yaw_deg;
    new_item.camera_action = camera_action;
    return new_item;
}

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<System> get_system(Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // Waiting for new systems to be discovered
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again to stop it searching for other autopilot
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    // Waiting 3 seconds for a heartbeat
    if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}
//------------------------------------------------------------------------------------------------------

void telemetry_callback(mavsdk::Telemetry& telemetry, std::chrono::steady_clock::time_point start_time) 
{            

    // Read the battery information
    const auto battery = telemetry.battery();

    // Read the North-East-Down Velocity
    const auto velocity_ned = telemetry.velocity_ned();
    const auto speed = sqrt(velocity_ned.north_m_s*velocity_ned.north_m_s+velocity_ned.east_m_s*velocity_ned.east_m_s+velocity_ned.down_m_s*velocity_ned.down_m_s); 
    
    const auto rawgps = telemetry.raw_gps();

    // Read imu -> acceleration, angular velocity, magnetic field, temperature and timestamp_us 
    const auto imu = telemetry.imu();
    const auto accel = sqrt(imu.acceleration_frd.forward_m_s2*imu.acceleration_frd.forward_m_s2+imu.acceleration_frd.right_m_s2*imu.acceleration_frd.right_m_s2+imu.acceleration_frd.down_m_s2*imu.acceleration_frd.down_m_s2);
    
    //Remote control status
    const auto rcstatus = telemetry.rc_status();

    // Pressure
    const auto pressure = telemetry.scaled_pressure();

    //GPS
    const auto gps = telemetry.get_gps_global_origin();

    // Read the current time
    const auto current_time = std::chrono::steady_clock::now();

    // Calculate the flight time
    const auto flight_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    std::ofstream myfile1;
    myfile1.open("flightvalues.csv", std::ios::cur);
    myfile1 << flight_time.count() << "s;" << std::fixed << std::setprecision(0) << battery.remaining_percent*100 << " %; "
            << std::fixed << std::setprecision(2) << battery.voltage_v << " V;"
            << std::fixed << std::setprecision(2) << imu.temperature_degc << " C;"
            << std::fixed << std::setprecision(2) << pressure.absolute_pressure_hpa << " hPa;"
            << std::fixed << std::setprecision(2) << velocity_ned.north_m_s 
            << ", " << std::fixed << std::setprecision(2) << velocity_ned.east_m_s
            << ", " << std::fixed << std::setprecision(2) << velocity_ned.down_m_s << " m/s;"
            << speed << " m/s;"
            << rawgps.velocity_m_s << " m/s;" 
            << std::fixed << std::setprecision(2) << imu.acceleration_frd.forward_m_s2 
            << ", " << std::fixed << std::setprecision(2) << imu.acceleration_frd.right_m_s2
            << ", " << std::fixed << std::setprecision(2) << imu.acceleration_frd.down_m_s2 << " m/s2;" 
            << accel << " m/s2;"
            << std::fixed << std::setprecision(2) << gps.second.altitude_m << " m;"    
            << std::fixed << std::setprecision(2) << gps.second.latitude_deg << " deg;"  
            << std::fixed << std::setprecision(2) << gps.second.longitude_deg << " deg\n";
    myfile1.close();

}

void send_esc_info (MavlinkPassthrough& mavlink_passthrough)
{
    mavlink_passthrough.subscribe_message_async(
        MAVLINK_MSG_ID_ESC_INFO,
        [](const mavlink_message_t& message) {

            
            mavlink_esc_info_t status;
            mavlink_msg_esc_info_decode(&message, &status);
            {

                auto escfail1 = status.failure_flags[0];
                    //std::cout << "ESC 1 failing? " << escfail1 << '\n';
                auto escfail2 = status.failure_flags[1];
                    //std::cout << "ESC 2 failing? " << escfail2 << '\n';
                auto escfail3 = status.failure_flags[2];
                    //std::cout << "ESC 3 failing? " << escfail3 << '\n';
                auto escfail4 = status.failure_flags[3];
                    //std::cout << "ESC 4 failing? " << escfail4 << '\n';

                auto esc1_fails = status.error_count[0];
                    //std::cout << "ESC 1 failed how many times? " << esc1_fails << '\n';
                auto esc2_fails = status.error_count[1];
                    //std::cout << "ESC 2 failed how many times? " << esc2_fails << '\n';
                auto esc3_fails = status.error_count[2];
                    //std::cout << "ESC 3 failed how many times? " << esc3_fails << '\n';
                auto esc4_fails = status.error_count[3];
                    //std::cout << "ESC 4 failed how many times? " << esc4_fails << '\n';

                auto esc_temperature1 = status.temperature[0];
                    //std::cout << "ESC 4 temp? " << esc_temperature1 << '\n';
                auto esc_temperature2 = status.temperature[1];
                     //std::cout << "ESC 4 temp? " << esc_temperature2 << '\n';
                auto esc_temperature3 = status.temperature[2];
                     //std::cout << "ESC 4 temp? " << esc_temperature3 << '\n';
                auto esc_temperature4 = status.temperature[3];
                     //std::cout << "ESC 4 temp? " << esc_temperature4 << '\n';

                std::ofstream myfile3;
                myfile3.open("esc_info.csv", std::ios::cur);
                myfile3 << std::fixed << std::setprecision(0) << escfail1 << " ;"
                    << std::fixed << std::setprecision(0) << escfail2 << " ;"
                    << std::fixed << std::setprecision(0) << escfail3 << " ;"
                    << std::fixed << std::setprecision(0) << escfail4 << " ;"
                    << std::fixed << std::setprecision(0) << esc1_fails << " ;"
                    << std::fixed << std::setprecision(0) << esc2_fails << " ;"
                    << std::fixed << std::setprecision(0) << esc3_fails << " ;"
                    << std::fixed << std::setprecision(0) << esc4_fails << " ;"
                    << std::fixed << std::setprecision(0) << esc_temperature1 << " C;"
                    << std::fixed << std::setprecision(0) << esc_temperature2 << " C;"
                    << std::fixed << std::setprecision(0) << esc_temperature3 << " C;"
                    << std::fixed << std::setprecision(0) << esc_temperature4 << " C\n";
                myfile3.close();
            }
        });
}


void send_esc_status (MavlinkPassthrough& mavlink_passthrough)
{
    mavlink_passthrough.subscribe_message_async(
        MAVLINK_MSG_ID_ESC_STATUS,
        [](const mavlink_message_t& message) {
            
            mavlink_esc_status_t status;
            mavlink_msg_esc_status_decode(&message, &status);
            {
                auto esc_rpm1 = status.rpm[0];
                    //std::cout << "ESC 1 rpms ?" << esc_rpm1 << '\n';
                auto esc_rpm2 = status.rpm[1];
                    //std::cout << "ESC 2 rpms ?" << esc_rpm2 << '\n';
                auto esc_rpm3 = status.rpm[2];
                    //std::cout << "ESC 3 rpms ?" << esc_rpm3 << '\n';
                auto esc_rpm4 = status.rpm[3];
                    //std::cout << "ESC 4 rpms ?" << esc_rpm4 << '\n';

                auto esc_voltage1 = status.voltage[0];
                    //std::cout << "ESC 1 voltage ?" << esc_voltage1 << '\n';
                auto esc_voltage2 = status.voltage[1];
                    //std::cout << "ESC 2 voltage ?" << esc_voltage2 << '\n';
                auto esc_voltage3 = status.voltage[2];
                    //std::cout << "ESC 3 voltage ?" << esc_voltage3 << '\n';
                auto esc_voltage4 = status.voltage[3];
                    //std::cout << "ESC 4 voltage ?" << esc_voltage4 << '\n';

                auto esc_current1 = status.current[0];
                     //std::cout << "ESC 1 current ?" << esc_current1 << '\n';
                auto esc_current2 = status.current[1];
                     //std::cout << "ESC 2 current ?" << esc_current2 << '\n';
                auto esc_current3 = status.current[2];
                     //std::cout << "ESC 3 current ?" << esc_current3 << '\n';
                auto esc_current4 = status.current[3];
                     //std::cout << "ESC 4 current ?" << esc_current4 << '\n';

                std::ofstream myfile2;
                myfile2.open("esc_status.csv", std::ios::cur);
                myfile2 << std::fixed << std::setprecision(0) << esc_rpm1 << " rpms;"
                    << std::fixed << std::setprecision(0) << esc_rpm2 << " rpms;"
                    << std::fixed << std::setprecision(0) << esc_rpm3 << " rpms;"
                    << std::fixed << std::setprecision(0) << esc_rpm4 << " rpms;"
                    << std::fixed << std::setprecision(2) << esc_voltage1 << " V;"
                    << std::fixed << std::setprecision(2) << esc_voltage2 << " V;"
                    << std::fixed << std::setprecision(2) << esc_voltage3 << " V;"
                    << std::fixed << std::setprecision(2) << esc_voltage4 << " V;"
                    << std::fixed << std::setprecision(2) << esc_current1 << " A;"
                    << std::fixed << std::setprecision(2) << esc_current2 << " A;"
                    << std::fixed << std::setprecision(2) << esc_current3 << " A;"
                    << std::fixed << std::setprecision(2) << esc_current4 << " A\n";
                myfile2.close();
            }
        });
}

void send_vfr_hud (MavlinkPassthrough& mavlink_passthrough)
{ 
    mavlink_passthrough.subscribe_message_async(
        MAVLINK_MSG_ID_VFR_HUD,                         
        [](const mavlink_message_t& message) {

            mavlink_vfr_hud_t status;
            mavlink_msg_vfr_hud_decode(&message, &status);
            {
                auto airspeed = status.airspeed;
                    //std::cout << std::setprecision(2) << "Airspeed:  " << airspeed << " m/s \n";

                auto groundspeed = status.groundspeed;
                    //std::cout << std::setprecision(2) << "Groundspeed: " << groundspeed << " m/s \n";

                auto heading = status.heading;
                    //std::cout << std::setprecision(2) << "Direction headed (0-360, 0 = north): " << heading << " deg \n";

                auto throttle = status.throttle;
                    //std::cout << std::setprecision(2) << "Throttle: " << throttle << " % \n";

                auto alt = status.alt;
                    //std::cout << std::setprecision(2) << "Altitude: " << alt << " m \n";

                auto climbrate = status.climb;
                    //std::cout << std::setprecision(2) << "Clime rate: " << climbrate << " m/s \n";

                
                std::ofstream myfile4;
                myfile4.open("vfr_hud.csv", std::ios::cur);
                myfile4 << std::fixed << std::setprecision(2) << airspeed << " m/s;"
                    << std::fixed << std::setprecision(2) << groundspeed << " m/s;"
                    << std::fixed << std::setprecision(2) << heading << " deg;"
                    << std::fixed << std::setprecision(2) << throttle << " %;"
                    << std::fixed << std::setprecision(2) << alt << " m;"
                    << std::fixed << std::setprecision(2) << climbrate << " m/s\n";
                myfile4.close();
            }
        });
}

void send_efi_status (MavlinkPassthrough& mavlink_passthrough)
{
 mavlink_passthrough.subscribe_message_async(
        MAVLINK_MSG_ID_EFI_STATUS,                         //callback does not happen
        [](const mavlink_message_t& message) {
            mavlink_efi_status_t status;
            mavlink_msg_efi_status_decode(&message, &status);
            {
                std::cout <<  status.health << '\n';
                std::cout <<  status.ecu_index << '\n';
                std::cout <<  status.rpm << '\n';
                std::cout <<  status.fuel_consumed << '\n';
                std::cout <<  status.fuel_flow << '\n';
                std::cout <<  status.engine_load << '\n';
                std::cout <<  status.throttle_position << '\n';
                std::cout <<  status.spark_dwell_time << '\n';
                std::cout <<  status.barometric_pressure << '\n';
                std::cout <<  status.intake_manifold_pressure << '\n';
                std::cout <<  status.intake_manifold_temperature << '\n';
                std::cout <<  status.cylinder_head_temperature << '\n';
                std::cout <<  status.ignition_timing << '\n';
                std::cout <<  status.injection_time << '\n';
                std::cout <<  status.exhaust_gas_temperature << '\n';
                std::cout <<  status.throttle_out << '\n';
                std::cout <<  status.pt_compensation << '\n';

            }
        });
}

void send_raw_rpm (MavlinkPassthrough& mavlink_passthrough)
{
     
 mavlink_passthrough.subscribe_message_async(
        MAVLINK_MSG_ID_RAW_RPM,                         //callback does not happen
        [](const mavlink_message_t& message) {

            mavlink_raw_rpm_t status;
            mavlink_msg_raw_rpm_decode(&message, &status);
            {
                std::cout <<  status.frequency ;
                std::cout <<  status.index ;
            }
        });
}
//------------------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk;
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success)
    {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = get_system(mavsdk);
    if (!system){
        return 1;
    }

    auto action = Action{system};
    auto mission = Mission{system};
    auto telemetry = Telemetry{system};

    // Create a MavlinkPassthrough object//////////////////////////////////////////////////////////
    auto mavlink_passthrough = MavlinkPassthrough{system};
    ///////////////////////////////////////////////////////////////////////////////////////////////

    while (!telemetry.health_all_ok())
    {
        std::cout << "Waiting for system to be ready\n";
        sleep_for(seconds(1));
    }

    std::cout << "System ready\n";
    std::cout << "Creating and uploading mission\n";

    //Start of mission upload ///////////////////////////////////////////
    
    std::vector<Mission::MissionItem> mission_items;

    mission_items.push_back(make_mission_item(
        47.398170327054473,
        8.5456490218639658,
        10.0f,
        5.0f,
        false,
        20.0f,
        60.0f,
        Mission::MissionItem::CameraAction::None));

    mission_items.push_back(make_mission_item(
        47.398241338125118,
        8.5455360114574432,
        10.0f,
        2.0f,
        true,
        0.0f,
        -60.0f,
        Mission::MissionItem::CameraAction::TakePhoto));

    mission_items.push_back(make_mission_item(
        47.398139363821485,
        8.5453846156597137,
        10.0f,
        5.0f,
        true,
        -45.0f,
        0.0f,
        Mission::MissionItem::CameraAction::StartVideo));

    mission_items.push_back(make_mission_item(
        47.398058617228855,
        8.5454618036746979,
        10.0f,
        2.0f,
        false,
        -90.0f,
        30.0f,
        Mission::MissionItem::CameraAction::StopVideo));

    mission_items.push_back(make_mission_item(
        47.398100366082858,
        8.5456969141960144,
        10.0f,
        5.0f,
        false,
        -45.0f,
        -30.0f,
        Mission::MissionItem::CameraAction::StartPhotoInterval));

    mission_items.push_back(make_mission_item(
        47.398001890458097,
        8.5455576181411743,
        10.0f,
        5.0f,
        false,
        0.0f,
        0.0f,
        Mission::MissionItem::CameraAction::StopPhotoInterval));

    
    std::cout << "Uploading mission...\n";
    Mission::MissionPlan mission_plan{};
    mission_plan.mission_items = mission_items;
    const Mission::Result upload_result = mission.upload_mission(mission_plan);

    if (upload_result != Mission::Result::Success) {
        std::cerr << "Mission upload failed: " << upload_result << ", exiting.\n";
        return 1;
    }

    //End of mission upload ////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------------------------------

    std::ofstream myfile1;
    myfile1.open("flightvalues.csv");
    myfile1 << "Flight Time; Battery %; Battery (V); Temp; Air Pressure; NED Velocity; Speed; Velocity; FRD Accel; Accel module; Altitude; Latitude; Longitude\n";
    myfile1.close();

    std::ofstream myfile2;
    myfile2.open("esc_status.csv");
    myfile2 << "esc_rpm1; esc_rpm2; esc_rpm3; esc_rpm4; esc_voltage1; esc_voltage2; esc_voltage3; esc_voltage4; esc_current1; esc_current2; esc_current3; esc_current4;\n";
    myfile2.close();

    std::ofstream myfile3;
    myfile3.open("esc_info.csv");
    myfile3 << "escfail1; escfail2; escfail3; escfail4; esc1_fails; esc2_fails; esc3_fails; esc4_fails; esc_temperature1; esc_temperature2; esc_temperature3; esc_temperature4; \n";
    myfile3.close();

    std::ofstream myfile4;
    myfile4.open("vfr_hud.csv");
    myfile4 << "Airspeed; Groundspeed; Heading; Throttle; Altitude; Climbrate\n";
    myfile4.close();

    //It saves the time the drone started flying
    const auto start_time = std::chrono::steady_clock::now(); 

    std::thread timer_thread([&](){
        while (true) {
            telemetry_callback(telemetry, start_time); 
            
            send_esc_status(mavlink_passthrough);
            send_esc_info(mavlink_passthrough);
            send_vfr_hud(mavlink_passthrough);

            send_raw_rpm(mavlink_passthrough);      //the callback does not happen
            send_efi_status(mavlink_passthrough);   //the callback does not happen
        
            // Wait for 1 second
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

//------------------------------------------------------------------------------------------------------

    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return 1;
    }
    std::cout << "Armed.\n";

    std::atomic<bool> want_to_pause{false};
    // Mission progress subscription.
    mission.subscribe_mission_progress([&want_to_pause](Mission::MissionProgress mission_progress) {
        std::cout << "Mission status update: " << mission_progress.current << " / "
                  << mission_progress.total << '\n';

        if (mission_progress.current >= 2) {
            // Flag setting
            want_to_pause = true;
        }
    });

    Mission::Result start_mission_result = mission.start_mission();
    if (start_mission_result != Mission::Result::Success) {
        std::cerr << "Starting mission failed: " << start_mission_result << '\n';
        return 1;
    }

    while (!want_to_pause) {
        sleep_for(seconds(1));
    }

    std::cout << "Pausing mission...\n";
    const Mission::Result pause_mission_result = mission.pause_mission();

    if (pause_mission_result != Mission::Result::Success) {
        std::cerr << "Failed to pause mission:" << pause_mission_result << '\n';
    }
    std::cout << "Mission paused.\n";

    // Pause for 5 seconds.
    sleep_for(seconds(5));

    // Then continue.
    Mission::Result start_mission_again_result = mission.start_mission();
    if (start_mission_again_result != Mission::Result::Success) {
        std::cerr << "Starting mission again failed: " << start_mission_again_result << '\n';
        return 1;
    }

    while (!mission.is_mission_finished().second) {
        sleep_for(seconds(1));
    }

    // Return to launch.
    std::cout << "Commanding RTL...\n";
    const Action::Result rtl_result = action.return_to_launch();
    if (rtl_result != Action::Result::Success) {
        std::cout << "Failed to command RTL: " << rtl_result << '\n';
        return 1;
    }
    std::cout << "Commanded RTL.\n";

    // Wait for armed state 
    sleep_for(seconds(2));
    
    while (telemetry.armed()) {
        sleep_for(seconds(1));
    }
    std::cout << "Disarmed, exiting.\n";
    
}
