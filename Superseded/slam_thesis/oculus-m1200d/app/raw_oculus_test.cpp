/// ----------------------------------------------------------------------------
///
/// Created 2021 - Flinders University
/// Author: Dr Phil Skelton (phillip.skelton@flinders.edu.au)
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
* @file raw_oculus_test.cpp
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


// Avoid pollution with global.
namespace
{
  volatile std::sig_atomic_t gSignalStatus;
}


void signal_handler(int32_t signal);


int main(int32_t argc, char * argv[])
{
  if (argc != 2)
  {
    std::cout << "Incorrect usage." << std::endl;
    std::cout << "Correct usage: raw_oculus_test [trace|debug|info|warning|error|fatal]" << std::endl;
    std::cout << "Example usage for TRACE and above: raw_oculus_test trace" << std::endl;
    std::cout << "Example usage for WARNING and above: raw_oculus_test warning" << std::endl;

    return 1;
  }

  // Install a signal handler for catching Ctrl+C (SIGINT).
  std::signal(SIGINT, signal_handler);

  std::string oculus_logging_severity = argv[1];

  OculusDriver oculus(oculus_logging_severity);

  oculus.beginInitialisation();

  while (oculus.isHealthy() && !oculus.isInitialised() && (gSignalStatus != SIGINT))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  oculus.setDataDepth(32);
  oculus.setGain(0.0);
  oculus.setGamma(0);
  oculus.setFrequencyMode("low");
  oculus.setOculusMacAddress("20:b0:f7:04:17:d6");
  oculus.setPingRate(10);
  oculus.setRangeUpperLimit(10.0);
  // oculus.setSalinityPPT(40.0);

  if (gSignalStatus != SIGINT)
  {
    oculus.run();

    const uint32_t kStatusUpdatePeriodMs = 1000;
    std::chrono::time_point<std::chrono::steady_clock> last_output_message_time = std::chrono::steady_clock::now();

    const std::chrono::duration<int64_t, std::ratio<1, 1000>> kLoopPeriodDuration = std::chrono::milliseconds(100);
    std::chrono::time_point<std::chrono::steady_clock> next_loop_begin_time = std::chrono::steady_clock::now();

    while (oculus.isHealthy() && (gSignalStatus != SIGINT))
    {
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

            if (latest_packet->unpacked_8_bit_data.size() > 0)
            {
              std::cout << "unpacked_8_bit_data -> average: "
                << std::accumulate(
                  latest_packet->unpacked_8_bit_data[0].begin(),
                  latest_packet->unpacked_8_bit_data[0].end(),
                  0.0) / static_cast<double>(latest_packet->unpacked_8_bit_data[0].size())
                << std::endl;
            }
            else if (latest_packet->unpacked_16_bit_data.size() > 0)
            {
              std::cout << "unpacked_16_bit_data -> average: "
                << std::accumulate(
                  latest_packet->unpacked_16_bit_data[0].begin(),
                  latest_packet->unpacked_16_bit_data[0].end(),
                  0.0) / static_cast<double>(latest_packet->unpacked_16_bit_data[0].size())
                << std::endl;
            }
            else
            {
              std::cout << "unpacked_32_bit_data -> average: "
                << std::accumulate(
                  latest_packet->unpacked_32_bit_data[0].begin(),
                  latest_packet->unpacked_32_bit_data[0].end(),
                  0.0) / static_cast<double>(latest_packet->unpacked_32_bit_data[0].size())
                << std::endl;
            }
          }
        }
      }

      next_loop_begin_time += kLoopPeriodDuration;
      std::this_thread::sleep_until(next_loop_begin_time);
    }
  }

  return 0;
}


void signal_handler(int32_t signal)
{
  gSignalStatus = signal;
}
