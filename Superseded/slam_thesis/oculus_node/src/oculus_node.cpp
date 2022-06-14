/// ----------------------------------------------------------------------------
///
/// Created 2021 - Flinders University
/// Author: Dr Phil Skelton (phillip.skelton@flinders.edu.au)
/// Edited: Cindy Zhu (Thales Australia)
/// Edited: Matthew D'Souza (Thales Australia) - ROS Integration
///
/// This software, and any immediately related software written by the
/// aforementioned, are classified as Trade Secret intellectual property, and
/// should thus be treated as so by authorised users. Under no circumstances is
/// this software to be stored on Public repositories such as (but not limited
/// to): GitHub, GitLab, or SourceForge. Storage on internal secure platforms
/// is permitted. Storage on external secure platforms (as assessed by
/// industry standard IT Security auditing procedures) requires approval from
/// the creator.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
/// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
/// IN THE SOFTWARE.
///
/// ----------------------------------------------------------------------------

/**
* @file save_oculus_data.cpp
*/

// Timing functions.
#include <chrono>

// String functions.
#include <string>

// Signal handlers.
#include <csignal>

// Vector accumulation.
#include <numeric>

// Core system-level driver.
#include "blueprint_subsea_oculus_driver/OculusDriver.hpp"

// To allow interfacing with json files
#include "../include/json.hpp"

// To configure data on the fly, and save data to files
#include "../include/oculusInterface.hpp"

#include <typeinfo>

// ROS
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "oculus_node/OculusHeader.h"

// Avoid pollution with global.
namespace
{
    volatile std::sig_atomic_t gSignalStatus;
}

void signal_handler(int32_t signal);

void sendRosSonarImage(std::shared_ptr<OculusDriver::DataPacket> packet);

ros::Publisher oculus_image_pub; // Publisher of sonar image
ros::Publisher oculus_header_pub; // Publisher of image header

int main(int argc, char **argv)
{
    //   if (argc != 2)
    //   {
    //     std::cout << "Incorrect usage." << std::endl;
    //     std::cout << "Correct usage: raw_oculus_test [trace|debug|info|warning|error|fatal]" << std::endl;
    //     std::cout << "Example usage for TRACE and above: save_oculus_data trace" << std::endl;
    //     std::cout << "Example usage for WARNING and above: save_oculus_data warning" << std::endl;

    //     return 1;
    //   }

    // Set up ROS stuff
    ros::init(argc, argv, "oculus");
    ros::NodeHandle n;
    ROS_INFO("Oculus node started");

    oculus_image_pub = n.advertise<sensor_msgs::Image>("oculus/polarscan", 1000);
    oculus_header_pub = n.advertise<oculus_node::OculusHeader>("oculus/header", 1000);

    // Install a signal handler for catching Ctrl+C (SIGINT).
    std::signal(SIGINT, signal_handler);

    //   std::string oculus_logging_severity = argv[1];
    std::string oculus_logging_severity = "debug";

    OculusDriver oculus(oculus_logging_severity);

    // TODO: Quick n dodgy. Make better.
    std::string configFileName = "/home/thales/matt_thesis/catkin_ws/src/slam_thesis/oculus_node/config.json";
    oculusInterface interface(configFileName);

    oculus.beginInitialisation();

    while (oculus.isHealthy() && !oculus.isInitialised() && (gSignalStatus != SIGINT))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    oculus.setDataDepth(16); // 16 bit mode
    oculus.setGain(0.0);
    oculus.setGamma(0);
    // oculus.setFrequencyMode("low");
    oculus.setFrequencyMode("high");
    // oculus.setOculusMacAddress("20:b0:f7:04:17:d6");
    // oculus.setOculusMacAddress("20:b0:f7:05:14:ea");
    oculus.setOculusIpAddress("192.168.2.10");
    oculus.setPingRate(10);
    oculus.setRangeUpperLimit(10.0);
    // oculus.setSalinityPPT(40.0);

    interface.checkAndUpdateSonarConfig(oculus);

    // Would be nice if stuff in here was tidied up somewhere else
    while (ros::ok())
    {
        if (gSignalStatus != SIGINT)
        {
            oculus.run();

            const uint32_t kStatusUpdatePeriodMs = 1000;
            std::chrono::time_point<std::chrono::steady_clock> last_output_message_time = std::chrono::steady_clock::now();

            const std::chrono::duration<int64_t, std::ratio<1, 1000>> kLoopPeriodDuration = std::chrono::milliseconds(100);
            std::chrono::time_point<std::chrono::steady_clock> next_loop_begin_time = std::chrono::steady_clock::now();

            while (oculus.isHealthy() && (gSignalStatus != SIGINT))
            // while (oculus.isHealthy())
            {
                interface.checkAndUpdateSonarConfig(oculus);

                if (oculus.calculateMicrosBetweenTimePoints(last_output_message_time, next_loop_begin_time) >= kStatusUpdatePeriodMs)
                {
                    last_output_message_time = next_loop_begin_time;

                    if (oculus.isStandingBy())
                    {
                        std::cout << "Device currently in `standby`. Set ping_rate > 0 to enable." << std::endl;
                    }
                    else
                    {
                        std::cout << oculus.outputLatestSimplePingResult();

                        std::shared_ptr<OculusDriver::DataPacket> latest_packet = oculus.getLatestDataPacket();

                        if (latest_packet)
                        {
                            std::cout << "message_stamp: " << std::chrono::duration_cast<std::chrono::milliseconds>(latest_packet->message_stamp.time_since_epoch()).count() << std::endl;
                            std::cout << "azimuths.size(): " << latest_packet->azimuths.size() << std::endl;
                            std::cout << "unpacked_8_bit_data.size(): " << latest_packet->unpacked_8_bit_data.size() << std::endl;
                            std::cout << "unpacked_16_bit_data.size(): " << latest_packet->unpacked_16_bit_data.size() << std::endl;
                            std::cout << "unpacked_32_bit_data.size(): " << latest_packet->unpacked_32_bit_data.size() << std::endl;

                            // TODO - check if the .getGain() and the returning message have the same value when the gain dismatches
                            std::cout << "double gain:" << +oculus.getGain() << std::endl;
                            double rangeRes = latest_packet->simple_ping_result.range_resolution;
                            std::cout << "range resolution: " << rangeRes << std::endl;

                            if (latest_packet->unpacked_8_bit_data.size() > 0)
                            {
                                std::cout << "unpacked_8_bit_data -> average: "
                                          << std::accumulate(
                                                 latest_packet->unpacked_8_bit_data[0].begin(),
                                                 latest_packet->unpacked_8_bit_data[0].end(),
                                                 0.0) /
                                                 static_cast<double>(latest_packet->unpacked_8_bit_data[0].size())
                                          << std::endl;

                                // std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
                                // for (uint32_t i = 0; i < latest_packet->unpacked_8_bit_data.size(); i++) {
                                //     for (uint32_t j = 0; j < latest_packet->unpacked_8_bit_data[i].size(); j++)
                                //         std::cout << +latest_packet->unpacked_8_bit_data[i][j] << " ";
                                //     std::cout << std::endl;
                                //     std::cout << "--------------------------------------------------------------"<< std::endl;
                                // }
                                // std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
                            }
                            else if (latest_packet->unpacked_16_bit_data.size() > 0)
                            {

                                std::cout << "unpacked_16_bit_data -> average: "
                                          << std::accumulate(
                                                 latest_packet->unpacked_16_bit_data[0].begin(),
                                                 latest_packet->unpacked_16_bit_data[0].end(),
                                                 0.0) /
                                                 static_cast<double>(latest_packet->unpacked_16_bit_data[0].size())
                                          << std::endl;

                                interface.writeToBinary_16bit(latest_packet);
                                // interface.writeToCsv_16bit(latest_packet->unpacked_16_bit_data);
                                
                                sendRosSonarImage(latest_packet);

                                // std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
                                // for (uint32_t i = 0; i < latest_packet->unpacked_16_bit_data.size(); i++){
                                //   for (uint32_t j = 0; j < latest_packet->unpacked_16_bit_data[i].size(); j++)
                                //     std::cout << latest_packet->unpacked_16_bit_data[i][j] << " ";
                                //   std::cout << std::endl << "--------------------------------------------------------------"<< std::endl;
                                // }
                                // std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
                            }
                            else
                            {
                                std::cout << "unpacked_32_bit_data -> average: "
                                          << std::accumulate(
                                                 latest_packet->unpacked_32_bit_data[0].begin(),
                                                 latest_packet->unpacked_32_bit_data[0].end(),
                                                 0.0) /
                                                 static_cast<double>(latest_packet->unpacked_32_bit_data[0].size())
                                          << std::endl;

                                // std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
                                // for (uint32_t i = 0; i < latest_packet->unpacked_32_bit_data.size(); i++) {
                                //     for (uint32_t j = 0; j < latest_packet->unpacked_32_bit_data[i].size(); j++)
                                //         std::cout << latest_packet->unpacked_32_bit_data[i][j] << " ";
                                //     std::cout << std::endl << "--------------------------------------------------------------"<< std::endl;
                                // }
                                // std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
                            }
                        }
                    }
                }

                next_loop_begin_time += kLoopPeriodDuration;
                std::this_thread::sleep_until(next_loop_begin_time);
            }
        }

        ros::spinOnce();
    }

    return 0;
}

void signal_handler(int32_t signal)
{
    gSignalStatus = signal;
}

void sendRosSonarImage(std::shared_ptr<OculusDriver::DataPacket> packet)
{
    // Set scan time to current time in ROS (assume that IMU is stamped off same clock)
    sensor_msgs::Image image;
    image.header.stamp = ros::Time::now();

    // Pack the image
    uint32_t n_ranges = packet->unpacked_16_bit_data[0].size();
    uint32_t n_angles = packet->unpacked_16_bit_data.size();
    uint32_t step = n_angles * sizeof(uint16_t);
    ROS_INFO("h_px = %d | w_px = %d", n_ranges, n_angles);

    // Pack the image
    image.height = n_angles;
    image.width = n_ranges;
    image.encoding = "mono16";
    image.is_bigendian = false;                  // TODO: check that this is actually the case
    image.step = image.width * sizeof(uint16_t); // Step is no. bytes between successive rows

    // Data is a uint8 vector. Need to convent the uint16 to pairs of uint8.

    std::vector<uint16_t> data_vector{};
    for (auto &&v : packet->unpacked_16_bit_data)
    {
        data_vector.insert(data_vector.end(), v.begin(), v.end());
    }

    ROS_INFO("vector size: %d", data_vector.size() * sizeof(uint16_t));

    uint32_t n_bytes_8bit = data_vector.size() * sizeof(uint16_t);
    image.data.resize(n_bytes_8bit);
    ROS_INFO("cap: %d", image.data.capacity());
    memcpy(image.data.data(), data_vector.data(), n_bytes_8bit);

    ROS_INFO("image size: %d", image.data.size() * sizeof(uint8_t));
    

    // Pack the header
    oculus_node::OculusHeader oculus_header;
    oculus_header.header = image.header;
    oculus_header.speedOfSoundUsed = packet->simple_ping_result.speed_of_sound_used;
    oculus_header.pingId = packet->simple_ping_result.ping_id;
    oculus_header.frequency = packet->simple_ping_result.frequency;
    oculus_header.temperature = packet->simple_ping_result.temperature;
    oculus_header.pressure = packet->simple_ping_result.pressure;
    oculus_header.heading = packet->simple_ping_result.heading;
    oculus_header.pitch = packet->simple_ping_result.pitch;
    oculus_header.roll = packet->simple_ping_result.roll;
    oculus_header.pingStartTime = packet->simple_ping_result.ping_start_time;
    oculus_header.rangeCount = packet->simple_ping_result.range_count;
    oculus_header.beamCount = packet->simple_ping_result.beam_count;
    oculus_header.dataSize = packet->simple_ping_result.image_size;
    oculus_header.rangeResolution = packet->simple_ping_result.range_resolution;
    oculus_header.range = packet->simple_ping_result.simple_fire_message.range_upper_limit;
    oculus_header.gain = packet->simple_ping_result.simple_fire_message.gain_percent;
    oculus_header.freqMode = static_cast<uint8_t>(packet->simple_ping_result.simple_fire_message.frequency_mode);

    // Publish image and header
    oculus_image_pub.publish(image);
    oculus_header_pub.publish(oculus_header);
   
}
