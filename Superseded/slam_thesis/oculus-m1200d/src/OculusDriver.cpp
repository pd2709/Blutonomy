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
* @file OculusDriver.cpp
*/


#include "blueprint_subsea_oculus_driver/OculusDriver.hpp"

/*! \mainpage Blueprint Subsea Oculus Driver
 *
 * \section author_sec Author
 *
 * Dr Phil Skelton, Flinders University (phillip.skelton@flinders.edu.au).
 *
 *
 * \section install_sec Installation
 *
 * Good luck...
 *
 * This project is designed to be system-wide installed as a static library (.a on Linux) that is discoverable by CMake.
 *
 * Typical CMake workflow is: cd `project_dir` && mkdir build && cd build && cmake .. && sudo make install.
 *
 *
 * \section problems_sec FAQ
 *
 * Q1) Does the Oculus have a low power sleep mode?
 *
 * A1) Yes, it boots into a low power standby mode. However, it will not re-enter this mode once it has been activated.
 * Upon boot, it will draw ~380 mA @ 24 VDC. Once operational, it will draw ~660 mA @ 24 VDC, up to the maximum draw as
 * specified in the datasheet.
 *
 *
 * Q2) Is this production-ready code?
 *
 * A2) Haha, hahahahaha, hah... No. It has quirks. It has bugs. It was never intended to be used outside of
 * Flinders University. The primary author is a Mechanical Engineer.
 *
 *
 * Q3) Why has <b>some_feature</b> been implemented in <i>some_convoluted_way</i>?
 *
 * A3) See A2.
 *
 *
 * Q4) Does this support Windows?
 *
 * A4) Natively? No. Can it? Yes. Will I? No. Can you? Yes.
 *
 *
 * Q5) Does this work on ARM?
 *
 * A5) Yes, at least on the armv71 that the Odroid XU4 identifies as. There may be two compilation 'info' outputs that
 * can safely be ignored (they are not 'warnings', so will not trigger -Werror). These do not appear on X86 systems.
 *
 *
 * Q6) Why are your variable names so long?
 *
 * A6) Because I want to know exactly what they are for without having to interpret or refer to something else.
 *
 *
 * Q7) Those compiler options... what the hell?
 *
 * A7) Don't be a lazy programmer.
 *
 *
 * Q8) Why don't you just use the Oculus.h file provided by Blueprint Subsea that has all of the message definitions?
 *
 * A8) Because that file is a mess. It is also an external definitions file, and I do not like my program being beholden
 * to random changes that may sneak in by that. One example is the definition of the MessageId enumerator. It was a
 * uint8_t in SDK version 1.8.x, but a uint16_t in 1.9.x; that is very frustrating to deal with when you are trying to
 * calculate exact message lengths.
 */

// Because we are on C++14, we need to declare the 'static constexpr' constants from the header here also.
// This is to ensure that the compiler knows to initialise these before they are actually used.
// Still better in so many ways than using #define.
constexpr double OculusDriver::kBearingResolutionDegrees;
constexpr double OculusDriver::kDegreesToRadians;
constexpr double OculusDriver::kDeviceCalculatesSpeedOfSound;
constexpr std::pair<OculusDriver::FrequencyMode, OculusDriver::FrequencyMode> OculusDriver::kFrequencyModeLimits;
constexpr std::pair<double, double> OculusDriver::kGainLimits;
constexpr char OculusDriver::kIpToUseMacAddressMatching[];
constexpr uint8_t OculusDriver::kMaximumNetworkSpeed;
constexpr uint16_t OculusDriver::kOculusDataPort;
constexpr uint16_t OculusDriver::kOculusId;
constexpr uint16_t OculusDriver::kOculusStatusPort;
constexpr std::pair<OculusDriver::PingRateHz, OculusDriver::PingRateHz> OculusDriver::kPingRateHzLimits;
constexpr std::pair<double, double> OculusDriver::kRangeLimitsAtHighFrequency;
constexpr std::pair<double, double> OculusDriver::kRangeLimitsAtLowFrequency;
constexpr double OculusDriver::kSalinityOfSaltWaterPPT;
constexpr std::pair<double, double> OculusDriver::kArbitrarySpeedOfSoundLimits;
constexpr float OculusDriver::kSocketTimeoutSecs;


// ---------------------------------------------------------------------------------------------------------------------
// Constructors.
// ---------------------------------------------------------------------------------------------------------------------

/*!
 * Takes a std::string representation for logging severity in descending order of inclusion (trace includes all,
 * debug includes all but trace, etc.).
 *
 * @param[in] logging_severity A std::string representation in set [trace, debug, info, warning, error, fatal].
 */
OculusDriver::OculusDriver(std::string logging_severity)
{
  if (logging_severity == "trace")
  {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::trace);
  }
  else if (logging_severity == "debug")
  {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::debug);
  }
  else if (logging_severity == "info")
  {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::info);
  }
  else if (logging_severity == "warning")
  {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::warning);
  }
  else if (logging_severity == "error")
  {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::error);
  }
  else
  {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::fatal);
  }

  BOOST_LOG_TRIVIAL(debug) << "OculusDriver() - I have been instantiated.";

  this->ping_rate_flag_ = this->PingRateFlag::kStandby;
  this->oculus_ip_address_ = "0.0.0.0";
  this->speed_of_sound_ = this->kDeviceCalculatesSpeedOfSound;
  this->salinity_ = this->kSalinityOfSaltWaterPPT;
}


// ---------------------------------------------------------------------------------------------------------------------
// Destructors.
// ---------------------------------------------------------------------------------------------------------------------

/*!
 * Primary destructor.
 *
 * This _should_ put the Oculus into a low-power mode (currently not working). It handles the thread destruction and the
 * socket destruction.
 */
OculusDriver::~OculusDriver()
{
  // TODO(Phil Skelton): Check with Blueprint as to why the Oculus does not go into a low power mode.
  this->setPingRate(0);
  this->reduce_power_ = true;
  OculusDriver::SimpleFireMessageVersion2 simple_fire_message_version_2 = this->generateSimpleFireMessageVersion2();
  this->parameters_have_changed_ = false;
  this->sendSimpleFireMessageVersion2(simple_fire_message_version_2);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  this->propagateShutdown();

  if (this->data_socket_retriever_thread_.joinable())
  {
    this->data_socket_retriever_thread_.join();
  }

  if (this->message_finder_and_unpacker_thread_.joinable())
  {
    this->message_finder_and_unpacker_thread_.join();
  }

  if (this->simple_fire_message_sender_thread_.joinable())
  {
    this->simple_fire_message_sender_thread_.join();
  }

  if (this->status_socket_retriever_thread_.joinable())
  {
    this->status_socket_retriever_thread_.join();
  }

  close(this->data_socket_);
  close(this->status_socket_);
}


// ---------------------------------------------------------------------------------------------------------------------
// Public Functions.
// ---------------------------------------------------------------------------------------------------------------------

/*!
 * Getter function for retrieving the DataDepth parameter.
 *
 * This is only what the driver thinks the parameter is set as.
 * Never use this for validation of parameter sets.
 * Instead, always read the metadata in the header of the data packet returned by the sensor.
 *
 * @return uint8_t Parameter value.
 */
uint8_t OculusDriver::getDataDepth()
{
  return this->data_depth_;
}


/*!
 * Getter function for retrieving the Gain parameter.
 *
 * This is only what the driver thinks the parameter is set as.
 * Never use this for validation of parameter sets.
 * Instead, always read the metadata in the header of the data packet returned by the sensor.
 *
 * @return <b>double</b> Parameter value.
 */
double OculusDriver::getGain()
{
  return this->gain_percent_;
}


/*!
 * Getter function for retrieving the Gamma parameter.
 *
 * This is only what the driver thinks the parameter is set as.
 * Never use this for validation of parameter sets.
 * Instead, always read the metadata in the header of the data packet returned by the sensor.
 *
 * @return Parameter value.
 */
uint8_t OculusDriver::getGamma()
{
  return this->gamma_;
}


/*!
 * Getter function for retrieving the FrequencyMode parameter as an enumeration value.
 *
 * This is only what the driver thinks the parameter is set as.
 * Never use this for validation of parameter sets.
 * Instead, always read the metadata in the header of the data packet returned by the sensor.
 *
 * @return Parameter value as enumeration.
 */
uint8_t OculusDriver::getFrequencyModeFlag()
{
  return static_cast<uint8_t>(this->frequency_mode_);
}


/*!
 * Getter function for retrieving the FrequencyMode parameter as a string representation.
 *
 * This is only what the driver thinks the parameter is set as.
 * Never use this for validation of parameter sets.
 * Instead, always read the metadata in the header of the data packet returned by the sensor.
 *
 * @return Parameter value as string representation.
 */
std::string OculusDriver::getFrequencyModeString()
{
  return this->frequency_mode_string_;
}


/*!
 * Getter function for retrieving the IP Address of the Oculus that is connected.
 *
 * This is what the device reports on the UDP StatusMessage port.
 *
 * @return Parameter value as IPv4 string.
 */
std::string OculusDriver::getOculusIpAddress()
{
  return this->oculus_ip_address_;
}


/*!
 * Getter function for retrieving the MAC Address of the Oculus.
 *
 * This is either: (preferred) the MAC address that was set by the user for discovering the Oculus;
 * or (non-preferred) the MAC address that was received from an IP-discovered Oculus.
 *
 * @return Parameter value.
 */
std::string OculusDriver::getOculusMacAddress()
{
  return this->oculus_mac_address_;
}


/*!
 * Getter function for retrieving the Ping Rate enumeration flag as an unsigned 8-bit number.
 *
 * @return Parameter value.
 */
uint8_t OculusDriver::getPingRateFlag()
{
  return static_cast<uint8_t>(this->ping_rate_flag_);
}


/*!
 * Getter function for retrieving the Ping Rate enumeration flag as a frequency (Hz) unsigned 8-bit number.
 *
 * @return Parameter value.
 */
uint8_t OculusDriver::getPingRateHz()
{
  return static_cast<uint8_t>(this->ping_rate_hz_);
}


/*!
 * Getter function for retrieving the Upper Range Limit value in metres.
 *
 * @return Parameter value.
 */
double OculusDriver::getRangeUpperLimit()
{
  return this->range_upper_limit_;
}


/*!
 * Function to seed the asynchronous initialisation procedure.
 */
void OculusDriver::beginInitialisation()
{
  this->initialisation_future_ = std::async(std::launch::async, &OculusDriver::asyncInitialise, this);
}


/*!
 * Getter function for retrieving the latest Data Packet.
 *
 * @return std::shared_ptr to the latest Data Packet.
 */
std::shared_ptr<OculusDriver::DataPacket> OculusDriver::getLatestDataPacket()
{
  if (!this->data_packet_queue_.empty())
  {
    return this->data_packet_queue_.back();
  }
  else
  {
    return nullptr;
  }
}


/*!
 * Getter function for retrieving the health flag of this driver.
 *
 * @return bool Whether the driver is happy or not.
 */
bool OculusDriver::isHealthy()
{
  return this->happy_.load();
}


/*!
 * Getter function for retrieving whether the driver is initialised or not.
 *
 * @return bool Whether the driver is initialised or not.
 */
bool OculusDriver::isInitialised()
{
  if (!this->initialisation_future_.valid())
  {
    this->propagateShutdown();
    return false;
  }
  else
  {
    std::future_status status = this->initialisation_future_.wait_for(std::chrono::milliseconds(100));

    if ((status == std::future_status::ready) || (status == std::future_status::deferred))
    {
      return this->initialisation_future_.get();
    }
    else
    {
      // This is equivalent to `else if (status == std::future_status::timeout)`.
      return false;
    }
  }
}


/*!
 * Getter function for retrieving whether the Oculus is currently in stand-by mode.
 *
 * @return bool Whether the Oculus is in stand-by mode.
 */
bool OculusDriver::isStandingBy()
{
  return (this->ping_rate_flag_ == this->PingRateFlag::kStandby);
}


/*!
 * Convenience function for setting baseline 'default' parameters.
 */
void OculusDriver::loadDefaultParameters()
{
  this->setDataDepth(8);
  this->setGain(0.0);
  this->setGamma(0xFF);
  this->setFrequencyMode("low");
  this->setPingRate(10);
  this->setRangeUpperLimit(5.0);
  this->setSalinityPPT(this->kSalinityOfSaltWaterPPT);
  this->setSpeedOfSound(this->kDeviceCalculatesSpeedOfSound);
}


/*!
 * Convenience function for parsing the header of a SimplePingResult to a nicely formatted std::string.
 *
 * This will return a std::string containing either: If no SimplePingResult exists in the queue, 4 lines;
 * if at least 1 SimplePingResult exists, it will dump the header of the latest and will return 53 lines.
 *
 * New lines are "\n" except for the very last entry which is a std::endl. This should ensure a contiguous output.
 *
 * @return Output string.
 */
std::string OculusDriver::outputLatestSimplePingResult()
{
  std::stringstream _stream;

  const uint32_t kFirstFieldLength = 35;

  // We use "\n" everywhere except the last line because std::endl flushes buffers.
  // Due to the length of this dump, we only want to flush on the last line.
  // Chances are the OS kernel buffer will be exceeded partway through the dump and will flush anyway.
  _stream << "--------------------------------------------------------------------------------" << "\n";
  _stream << std::left << "blueprint_subsea_oculus_driver - Formatted SimplePingResult output." << "\n";
  _stream << "\n";

  if (!this->data_packet_queue_.empty())
  {
    std::shared_ptr<OculusDriver::DataPacket> latest_packet = this->data_packet_queue_.back();

    _stream << "  SimplePingResult:" << "\n";
    _stream << "    SimpleFireMessageVersion2:" << "\n";
    _stream << "      OculusMessageHeader:" << "\n";
    _stream << std::setw(kFirstFieldLength) << "        oculus_id: "
      << std::hex << latest_packet->simple_ping_result.simple_fire_message.oculus_header.oculus_id << "\n";

    _stream << std::setw(kFirstFieldLength) << "        source_device_id: "
      << std::hex << latest_packet->simple_ping_result.simple_fire_message.oculus_header.source_device_id << "\n";

    _stream << std::setw(kFirstFieldLength) << "        destination_device_id: "
      << std::hex << latest_packet->simple_ping_result.simple_fire_message.oculus_header.destination_device_id << "\n";

    _stream << std::setw(kFirstFieldLength) << "        message_id: "
      << std::hex << latest_packet->simple_ping_result.simple_fire_message.oculus_header.message_id << "\n";

    _stream << std::setw(kFirstFieldLength) << "        message_version: "
      << std::dec << latest_packet->simple_ping_result.simple_fire_message.oculus_header.message_version << "\n";

    _stream << std::setw(kFirstFieldLength) << "        payload_size (B): "
      << std::dec << latest_packet->simple_ping_result.simple_fire_message.oculus_header.payload_size << "\n";

    _stream << std::setw(kFirstFieldLength) << "        unused: "
      << std::hex << latest_packet->simple_ping_result.simple_fire_message.oculus_header.unused << "\n";

    _stream << std::setw(kFirstFieldLength) << "      frequency_mode: "
      << std::hex << +static_cast<uint8_t>(latest_packet->simple_ping_result.simple_fire_message.frequency_mode) << "\n";

    _stream << std::setw(kFirstFieldLength) << "      ping_rate: "
      << std::hex << +static_cast<uint8_t>(latest_packet->simple_ping_result.simple_fire_message.ping_rate) << "\n";

    _stream << std::setw(kFirstFieldLength) << "      network_speed: "
      << +latest_packet->simple_ping_result.simple_fire_message.network_speed << "\n";

    _stream << std::setw(kFirstFieldLength) << "      gamma: "
      << +latest_packet->simple_ping_result.simple_fire_message.gamma << "\n";

    auto flags_bits = std::bitset<8>{
      static_cast<uint8_t>(latest_packet->simple_ping_result.simple_fire_message.flags)
    };
    _stream << std::setw(kFirstFieldLength) << "      flags: "
      << "0x" << std::hex << std::setfill('0')
      << std::setw(2 * sizeof(latest_packet->simple_ping_result.simple_fire_message.flags))
      << +static_cast<uint8_t>(latest_packet->simple_ping_result.simple_fire_message.flags)
      << std::setfill(' ') << "\n";

    _stream << std::setw(kFirstFieldLength) << "        use_metres: "
      << std::boolalpha << flags_bits[0] << "\n";

    _stream << std::setw(kFirstFieldLength) << "        use_16_bit_data_mode: "
      << std::boolalpha << flags_bits[1] << "\n";

    _stream << std::setw(kFirstFieldLength) << "        show_gain: "
      << std::boolalpha << flags_bits[2] << "\n";

    _stream << std::setw(kFirstFieldLength) << "        simple_return_message: "
      << std::boolalpha << flags_bits[3] << "\n";

    _stream << std::setw(kFirstFieldLength) << "        use_gain_assist: "
      << std::boolalpha << flags_bits[4] << "\n";

    _stream << std::setw(kFirstFieldLength) << "        reduce_power: "
      << std::boolalpha << flags_bits[5] << "\n";

    _stream << std::setw(kFirstFieldLength) << "      range_upper_limit (m): "
      << latest_packet->simple_ping_result.simple_fire_message.range_upper_limit << "\n";

    _stream << std::setw(kFirstFieldLength) << "      gain_percent: "
      << latest_packet->simple_ping_result.simple_fire_message.gain_percent << "\n";

    _stream << std::setw(kFirstFieldLength) << "      speed_of_sound (m/s): "
      << latest_packet->simple_ping_result.simple_fire_message.speed_of_sound << "\n";

    _stream << std::setw(kFirstFieldLength) << "      salinity (PPT): "
      << latest_packet->simple_ping_result.simple_fire_message.salinity << "\n";

    _stream << std::setw(kFirstFieldLength) << "      extra_option_flags: "
      << latest_packet->simple_ping_result.simple_fire_message.extra_option_flags << "\n";

    auto extra_option_flags_bits = std::bitset<32>{
      static_cast<uint32_t>(latest_packet->simple_ping_result.simple_fire_message.extra_option_flags)
    };
    _stream << std::setw(kFirstFieldLength) << "        use_32_bit_data_mode: "
      << std::boolalpha << extra_option_flags_bits[9] << "\n";

    _stream << std::setw(kFirstFieldLength) << "      reserved: "
      << latest_packet->simple_ping_result.simple_fire_message.reserved << "\n";

    _stream << std::setw(kFirstFieldLength) << "    ping_id: "
      << std::dec << latest_packet->simple_ping_result.ping_id << "\n";

    _stream << std::setw(kFirstFieldLength) << "    status: "
      << "0x" << std::hex << std::setfill('0')
      << std::setw(2 * sizeof(latest_packet->simple_ping_result.status))
      << latest_packet->simple_ping_result.status
      << std::setfill(' ') << "\n";

    _stream << std::setw(kFirstFieldLength) << "    frequency (Hz): "
      << std::dec << std::fixed << std::setprecision(6) << latest_packet->simple_ping_result.frequency << "\n";

    _stream << std::setw(kFirstFieldLength) << "    temperature (deg. C): "
      << std::dec << std::fixed << std::setprecision(3) << latest_packet->simple_ping_result.temperature << "\n";

    _stream << std::setw(kFirstFieldLength) << "    pressure (rel. Pa?): "
      << std::dec << std::fixed << std::setprecision(6) << latest_packet->simple_ping_result.pressure << "\n";

    _stream << std::setw(kFirstFieldLength) << "    heading (deg.): "
      << std::dec << std::fixed << std::setprecision(3) << latest_packet->simple_ping_result.heading << "\n";

    _stream << std::setw(kFirstFieldLength) << "    pitch (deg.): "
      << std::dec << std::fixed << std::setprecision(3) << latest_packet->simple_ping_result.pitch << "\n";

    _stream << std::setw(kFirstFieldLength) << "    roll (deg.): "
      << std::dec << std::fixed << std::setprecision(6) << latest_packet->simple_ping_result.roll << "\n";

    _stream << std::setw(kFirstFieldLength) << "    speed_of_sound_used (m/s): "
      << latest_packet->simple_ping_result.speed_of_sound_used << "\n";

    _stream << std::setw(kFirstFieldLength) << "    ping_start_time (s): "
      << std::dec << std::fixed << std::setprecision(6) << latest_packet->simple_ping_result.ping_start_time << "\n";

    uint8_t data_bit_count =
      (latest_packet->simple_ping_result.data_size == OculusDriver::DataSize::kData8Bit) ?
      8 :
      ((latest_packet->simple_ping_result.data_size == OculusDriver::DataSize::kData16Bit) ?
        16 :
        32);
    _stream << std::setw(kFirstFieldLength) << "    data_size (bits): "
      << std::dec << +data_bit_count << "\n";

    _stream << std::setw(kFirstFieldLength) << "    range_resolution (m): "
      << std::fixed << std::dec << latest_packet->simple_ping_result.range_resolution << "\n";

    _stream << std::setw(kFirstFieldLength) << "    range_count: "
      << std::dec << latest_packet->simple_ping_result.range_count << "\n";

    _stream << std::setw(kFirstFieldLength) << "    beam_count: "
      << std::dec << latest_packet->simple_ping_result.beam_count << "\n";

    _stream << std::setw(kFirstFieldLength) << "    spare_0: "
      << std::hex << latest_packet->simple_ping_result.spare_0 << "\n";

    _stream << std::setw(kFirstFieldLength) << "    spare_1: "
      << std::hex << latest_packet->simple_ping_result.spare_1 << "\n";

    _stream << std::setw(kFirstFieldLength) << "    spare_2: "
      << std::hex << latest_packet->simple_ping_result.spare_2 << "\n";

    _stream << std::setw(kFirstFieldLength) << "    spare_3: "
      << std::hex << latest_packet->simple_ping_result.spare_3 << "\n";

    _stream << std::setw(kFirstFieldLength) << "    image_start_offset (B): "
      << std::dec << latest_packet->simple_ping_result.image_start_offset << "\n";

    _stream << std::setw(kFirstFieldLength) << "    image_size (B): "
      << std::dec << latest_packet->simple_ping_result.image_size << "\n";

    _stream << std::setw(kFirstFieldLength) << "    message_size (B): "
      << std::dec << latest_packet->simple_ping_result.message_size << "\n";
  }
  else
  {
    _stream << "WARNING: No SimplePingResult objects found..." << "\n";
  }

  _stream << "--------------------------------------------------------------------------------" << std::endl;

  return _stream.str();
}


/*!
 * Convenience function for starting the data capture process.
 */
void OculusDriver::run()
{
  this->keep_firing_assholes_.store(true);
}


/*!
 * Convenience function for stopping the data capture process.
 */
void OculusDriver::stop()
{
  this->keep_firing_assholes_.store(false);
}


/*!
 * Convenience function for initiating the shutdown procedure.
 */
void OculusDriver::shutdown()
{
  this->propagateShutdown();
}


// ---------------------------------------------------------------------------------------------------------------------
// Private Functions.
// ---------------------------------------------------------------------------------------------------------------------

/*!
 * Asynchronous initialisation function.
 *
 * @returns bool Success (true) or Failure (false) of initialisation.
 */
bool OculusDriver::asyncInitialise()
{
  bool parameters_present_and_valid = this->validateParameters();

  if (!parameters_present_and_valid)
  {
    BOOST_LOG_TRIVIAL(fatal) << "asyncInitialise() - Error detected during parsing of parameters.";

    return false;
  }
  else
  {
    this->happy_.store(true);

    this->status_socket_ = this->createAndBindUdpSocket(this->kOculusStatusPort);
    this->status_socket_retriever_thread_ = std::thread(&OculusDriver::statusSocketRetrieverThreadFunction, this);

    BOOST_LOG_TRIVIAL(info) << "Waiting for an Oculus to be identified by StatusMessage...";

    while ((this->happy_.load() == true) && (this->oculus_identified_.load() == false))
    {
      // Don't worry that this is inaccurate; it's only a dumb waiter until the Oculus is identified.
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (this->oculus_identified_.load() == true)
    {
      this->data_socket_ = this->createAndConnectTcpSocket();

      bool sockets_successfully_connected = (
        (this->data_socket_ > 0)
        && (this->status_socket_ > 0)
        && (this->data_socket_ != this->status_socket_));

      if (!sockets_successfully_connected)
      {
        BOOST_LOG_TRIVIAL(fatal) << "asyncInitialise() - Sockets were unable to be successfully created.";

        return false;
      }
      else
      {
        // Threads start in general order of operations.
        // Only real dependency is that data_socket_retriever_thread_ starts before simple_fire_message_sender_thread_.
        // Otherwise, we may fill the TCP buffer and crash.
        this->data_socket_retriever_thread_ = std::thread(
          &OculusDriver::dataSocketRetrieverThreadFunction,
          this);

        this->simple_fire_message_sender_thread_ = std::thread(
          &OculusDriver::simpleFireMessageSenderThreadFunction,
          this);

        this->message_finder_and_unpacker_thread_ = std::thread(
          &OculusDriver::messageFinderAndUnpackerThreadFunction,
          this);

        return true;
      }
    }
    else
    {
      // No Oculus was found, which means the program is unhappy.
      return false;
    }
  }
}


/*!
 * Convenience function for calculating the time delta in microseconds between two steady_clock time_point values.
 *
 * @returns uint32_t Difference between two steady_clock time points.
 */
uint32_t OculusDriver::calculateMicrosBetweenTimePoints(
  const std::chrono::time_point<std::chrono::steady_clock> start_point,
  const std::chrono::time_point<std::chrono::steady_clock> end_point)
{
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(end_point - start_point).count());
}


/*!
 * Convenience function for calculating the time delta in microseconds between now and a steady_clock time_point value.
 *
 * @returns uint32_t Difference between now and a steady_clock time point.
 */
uint32_t OculusDriver::calculateMicrosSinceTimePoint(const std::chrono::time_point<std::chrono::steady_clock> start_point)
{
  return this->calculateMicrosBetweenTimePoints(start_point, std::chrono::steady_clock::now());
}


/*!
 * Convenience function for calculating the time delta between now and a steady_clock time_point value.
 *
 * @returns uint32_t Difference between now and a steady_clock time point.
 */
int32_t OculusDriver::createAndBindUdpSocket(uint16_t target_port)
{
  BOOST_LOG_TRIVIAL(debug)
    << "createAndBindUdpSocket() - "
    << "Attempting UDP socket on port ["
    << target_port
    << "].";

  int32_t new_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if (new_socket >= 0)
  {
    this->status_sockaddr_in_.sin_family = AF_INET;

    // Convert the uint16_t port value from host byte order (probably LSB) to network byte order (probably MSB).
#ifdef __x86_64__
    this->status_sockaddr_in_.sin_port = htons(target_port);
#elif __arm__
    // htons() is throwing a compilation error on ARM of:
    //   error: conversion to 'unsigned int' from 'int' may change the sign of the result [-Werror=sign-conversion]
    // Not in the mood to debug this given it is uint16_t in and uint16_t out, so brute force an endianness change.
    this->status_sockaddr_in_.sin_port = static_cast<uint16_t>(
      (static_cast<uint16_t>(target_port >> 8) & 0x00FF) |
      (static_cast<uint16_t>(target_port << 8) & 0xFF00)
    );
#endif

    this->status_sockaddr_in_.sin_addr.s_addr = htonl(INADDR_ANY);

    int32_t bind_successful = bind(
      new_socket,
      reinterpret_cast<struct sockaddr *>(&this->status_sockaddr_in_),
      sizeof(this->status_sockaddr_in_));

    if (bind_successful == 0)
    {
      BOOST_LOG_TRIVIAL(debug)
        << "createAndBindUdpSocket() - Successfully bound UDP socket port ["
        << target_port
        << "].";

      struct timeval timeout = {};
      timeout.tv_sec = 2;
      timeout.tv_usec = 0;

      int32_t socket_options_successful = setsockopt(
        new_socket,
        SOL_SOCKET,
        SO_RCVTIMEO,
        reinterpret_cast<char *>(&timeout),
        sizeof(timeout));

      int32_t reuse = 1;

      // Some systems need to set both the SO_REUSEADDR and SO_REUSEPORT flags.
      // This ensures that other processes (e.g., oculus_finder.py) can access the socket, if they so choose.
      socket_options_successful += setsockopt(
        new_socket,
        SOL_SOCKET,
        SO_REUSEADDR,
        reinterpret_cast<const char *>(&reuse),
        sizeof(reuse));

      socket_options_successful += setsockopt(
        new_socket,
        SOL_SOCKET,
        SO_REUSEPORT,
        reinterpret_cast<const char *>(&reuse),
        sizeof(reuse));

      if (socket_options_successful == 0)
      {
        BOOST_LOG_TRIVIAL(debug) << "createAndBindUdpSocket() - Successfully set socket options on UDP socket.";

        return new_socket;
      }
      else
      {
        BOOST_LOG_TRIVIAL(fatal) << "createAndBindUdpSocket() - Failed to set socket timeout.";

        return false;
      }
    }
    else
    {
      BOOST_LOG_TRIVIAL(fatal)
        << "createAndBindUdpSocket() - "
        << "Failed to connect socket to port ["
        << target_port
        << "], error code [";

      return false;
    }
  }
  else
  {
    BOOST_LOG_TRIVIAL(fatal) << "createAndBindUdpSocket() - Failed to create empty socket." << std::endl;

    return false;
  }
}


int32_t OculusDriver::createAndConnectTcpSocket()
{
  BOOST_LOG_TRIVIAL(debug)
    << "createAndConnectTcpSocket() - Attempting TCP socket to ["
    << this->oculus_ip_address_
    << ", "
    << this->kOculusDataPort
    << "].";

  int32_t new_socket = socket(AF_INET, SOCK_STREAM, 0);

  if (new_socket >= 0)
  {
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;

    // Convert the uint16_t port value from host byte order (probably LSB) to network byte order (probably MSB).
#ifdef __x86_64__
    server_address.sin_port = htons(this->kOculusDataPort);
#elif __arm__
    // htons() is throwing a compilation error on ARM of:
    //   error: conversion to 'unsigned int' from 'int' may change the sign of the result [-Werror=sign-conversion]
    // Not in the mood to debug this given it is uint16_t in and uint16_t out, so brute force an endianness change.
    // server_address.sin_port = htons(this->kOculusDataPort);
    server_address.sin_port = static_cast<uint16_t>(
      (static_cast<uint16_t>(this->kOculusDataPort >> 8) & 0x00FF) |
      (static_cast<uint16_t>(this->kOculusDataPort << 8) & 0xFF00)
    );
#endif

    int32_t ip_conversion_status = inet_pton(AF_INET, this->oculus_ip_address_.c_str(), &server_address.sin_addr);

    if (ip_conversion_status <= 0)
    {
      BOOST_LOG_TRIVIAL(error) << "createAndConnectTcpSocket() - Failed to convert IP address.";

      return false;
    }

    int32_t connection_successful = connect(
      new_socket,
      reinterpret_cast<struct sockaddr *>(&server_address),
      sizeof(server_address));

    if (connection_successful == 0)
    {
      BOOST_LOG_TRIVIAL(debug)
        << "createAndConnectTcpSocket() - Successfully connected TCP socket to ["
        << this->oculus_ip_address_
        << ", "
        << this->kOculusDataPort
        << "].";

      return new_socket;
    }
    else
    {
      BOOST_LOG_TRIVIAL(error)
        << "createAndConnectTcpSocket() - "
        << "Failed to connect socket to ["
        << this->oculus_ip_address_
        << ", "
        << this->kOculusDataPort
        << "], error code [";

      return false;
    }
  }
  else
  {
    BOOST_LOG_TRIVIAL(error) << "createAndConnectTcpSocket() - Failed to create empty socket.";

    return false;
  }
}


double OculusDriver::deviceBearingToAzimuth(uint16_t bearing)
{
  // Bearings are a weird format from the device in that they use <65535 to indicate negative angle:
  //   64500, 65000, 65500, 500, 1000, 1500
  // By subtraction of 65535 if bearing>threshold (65535/2), this translates to:
  //   -1035, -535, -35, 500, 1000, 1500
  // Which, at a resolution of 0.01, yields degree values:
  //   -10.35°, -5.35°, -0.35°, 5.0°, 10.0°, 15.0°
  // That can then be converted to radians:
  //   -0.18064, -0.09338, -0.00611, 0.087266, 0.174533, 0.261799
  // Of course, angles should actually progress counter-clockwise, so negate those:
  //   0.18064, 0.09338, 0.00611, -0.087266, -0.174533, -0.261799

  // Does not matter if this uses a `floor` or `ceil` to round because the data is so far away from the mid-point.
  // This is called understanding your application.
  const static uint16_t kThreshold = std::numeric_limits<uint16_t>::max() / 2;
  int32_t promoted = static_cast<int32_t>(bearing);
  int32_t bidirectional_bearing = promoted > kThreshold ? promoted - std::numeric_limits<uint16_t>::max() : promoted;
  double bearing_as_degrees = bidirectional_bearing * this->kBearingResolutionDegrees;
  double azimuth = -1.0 * bearing_as_degrees * this->kDegreesToRadians;

  return azimuth;
}


/*!
 * Floating point equality check.
 *
 * This multiplies the doubles by a promotion factor, in this case 1e9, casts both to int64_t, then checks equality.
 *
 * @param[in] first The first number.
 * @param[in] second The second number.
 * @return Logical equality between `first` and `second`.
 */
bool OculusDriver::floatsAreEqual(double first, double second)
{
  const static uint32_t kPromotionFactor = 1e9;
  return (static_cast<int64_t>(first * kPromotionFactor) == static_cast<int64_t>(second * kPromotionFactor));
}


/*!
 * Convenience function for floatsAreEqual(double first, double second).
 *
 * This accepts `first` a double, then `second` a float.
 *
 * @param[in] first The first number as a double.
 * @param[in] second The second number as a float.
 * @return Logical equality between `first` and `second`.
 */
bool OculusDriver::floatsAreEqual(double first, float second)
{
  return this->floatsAreEqual(first, static_cast<double>(second));
}


/*!
 * Convenience function for floatsAreEqual(double first, double second).
 *
 * This accepts `first` a float, then `second` a double.
 *
 * @param[in] first The first number as a float.
 * @param[in] second The second number as a double.
 * @return Logical equality between `first` and `second`.
 */
bool OculusDriver::floatsAreEqual(float first, double second)
{
  return this->floatsAreEqual(static_cast<double>(first), second);
}


/*!
 * Convenience function for floatsAreEqual(double first, double second).
 *
 * This accepts `first` a float, then `second` a float.
 *
 * @param[in] first The first number as a float.
 * @param[in] second The second number as a float.
 * @return Logical equality between `first` and `second`.
 */
bool OculusDriver::floatsAreEqual(float first, float second)
{
  return this->floatsAreEqual(static_cast<double>(first), static_cast<double>(second));
}


/*!
 * Convenience function for converting a FrequencyMode flag to a user-interpretable string.
 *
 * @param[in] frequency_mode_flag The enumerated flag to be converted.
 * @return std::string representation of the frequency mode.
 */
std::string OculusDriver::frequencyModeFlagToString(OculusDriver::FrequencyMode frequency_mode_flag)
{
  return (frequency_mode_flag == this->FrequencyMode::kLow) ? "low" : "high";
}


OculusDriver::SimpleFireMessage OculusDriver::generateSimpleFireMessage()
{
  OculusDriver::SimpleFireMessage message = {};

  message.oculus_header.oculus_id = this->oculus_id_;
  message.oculus_header.source_device_id = 0;
  message.oculus_header.destination_device_id = 0;
  message.oculus_header.message_id = this->MessageId::kSimpleFire;
  message.oculus_header.message_version = this->MessageVersion::kOriginal;

  message.frequency_mode = this->frequency_mode_;
  message.ping_rate = this->ping_rate_flag_;
  message.network_speed = this->kMaximumNetworkSpeed;
  message.gamma = this->gamma_;

  message.flags |= this->use_metres_ ? this->OptionFlag::kUseMetres : this->OptionFlag::kNoChange;
  message.flags |= this->use_16_bit_data_mode_ ? this->OptionFlag::kUse16BitData : this->OptionFlag::kNoChange;
  message.flags |= this->show_gain_ ? this->OptionFlag::kReportGain : this->OptionFlag::kNoChange;
  message.flags |= this->simple_return_message_ ? this->OptionFlag::kSimpleReturn : this->OptionFlag::kNoChange;
  message.flags |= this->use_gain_assist_ ? this->OptionFlag::kUseGainAssist : this->OptionFlag::kNoChange;
  message.flags |= this->reduce_power_ ? this->OptionFlag::kReducePower : this->OptionFlag::kNoChange;

  message.range_upper_limit = this->range_upper_limit_;
  message.gain_percent = this->gain_percent_;
  message.salinity = this->kSalinityOfSaltWaterPPT;

  if (this->floatsAreEqual(this->speed_of_sound_, this->kDeviceCalculatesSpeedOfSound))
  {
    message.speed_of_sound = this->kDeviceCalculatesSpeedOfSound;
  }
  else
  {
    message.speed_of_sound = this->speed_of_sound_;
  }

  BOOST_LOG_TRIVIAL(debug)
    << "generateSimpleFireMessage() - "
    << "A parameter has been changed, so the simple fire message has been generated again.";

  return message;
}


OculusDriver::SimpleFireMessageVersion2 OculusDriver::generateSimpleFireMessageVersion2()
{
  OculusDriver::SimpleFireMessageVersion2 message = {};

  message.oculus_header.oculus_id = this->oculus_id_;
  message.oculus_header.source_device_id = 0;
  message.oculus_header.destination_device_id = 0;
  message.oculus_header.message_id = this->MessageId::kSimpleFire;
  message.oculus_header.message_version = this->MessageVersion::kVersion2;

  message.frequency_mode = this->frequency_mode_;
  message.ping_rate = this->ping_rate_flag_;
  message.network_speed = this->kMaximumNetworkSpeed;
  message.gamma = this->gamma_;

  message.flags |= this->use_metres_ ? this->OptionFlag::kUseMetres : this->OptionFlag::kNoChange;
  message.flags |= this->use_16_bit_data_mode_ ? this->OptionFlag::kUse16BitData : this->OptionFlag::kNoChange;
  message.flags |= this->show_gain_ ? this->OptionFlag::kReportGain : this->OptionFlag::kNoChange;
  message.flags |= this->simple_return_message_ ? this->OptionFlag::kSimpleReturn : this->OptionFlag::kNoChange;
  message.flags |= this->use_gain_assist_ ? this->OptionFlag::kUseGainAssist : this->OptionFlag::kNoChange;
  message.flags |= this->reduce_power_ ? this->OptionFlag::kReducePower : this->OptionFlag::kNoChange;

  message.range_upper_limit = this->range_upper_limit_;
  message.gain_percent = this->gain_percent_;
  message.salinity = this->salinity_;
  message.speed_of_sound = this->speed_of_sound_;

  message.extra_option_flags =
    this->use_32_bit_data_mode_
    ? this->ExtraOptionFlag::kResearchDataMode
    : this->ExtraOptionFlag::kNoChange;

  BOOST_LOG_TRIVIAL(debug)
    << "generateSimpleFireMessageVersion2() - "
    << "A parameter has been changed, so the simple fire message has been generated again.";

  return message;
}


void OculusDriver::guardedUpdateToParametersChangedFlag()
{
  std::lock_guard<std::mutex> local_lock(this->parameter_change_mutex_);
  this->parameters_have_changed_ = true;
}


uint8_t OculusDriver::pingRateFlagToHz(OculusDriver::PingRateFlag ping_rate_flag)
{
  if (ping_rate_flag == this->PingRateFlag::kLowest)
  {
    return static_cast<uint8_t>(this->PingRateHz::kLowest);
  }
  else if (ping_rate_flag == this->PingRateFlag::kLow)
  {
    return static_cast<uint8_t>(this->PingRateHz::kLow);
  }
  else if (ping_rate_flag == this->PingRateFlag::kNormal)
  {
    return static_cast<uint8_t>(this->PingRateHz::kNormal);
  }
  else if (ping_rate_flag == this->PingRateFlag::kHigh)
  {
    return static_cast<uint8_t>(this->PingRateHz::kHigh);
  }
  else if (ping_rate_flag == this->PingRateFlag::kHighest)
  {
    return static_cast<uint8_t>(this->PingRateHz::kHighest);
  }
  else
  {
    return static_cast<uint8_t>(this->PingRateHz::kStandby);
  }
}


void OculusDriver::propagateShutdown()
{
  BOOST_LOG_TRIVIAL(debug) << "propagateShutdown() - Something has requested a shutdown.";

  this->scanning_enabled_.store(false);
  this->keep_firing_assholes_.store(false);
  this->happy_.store(false);

  // Various checks are done in the predicates of the condition_variables, so notify them all to wake up and check.
  this->scanning_enabled_condition_variable_.notify_all();
  this->raw_data_queue_condition_variable_.notify_all();
  this->data_packet_queue_condition_variable_.notify_all();
}


bool OculusDriver::sendSimpleFireMessage(OculusDriver::SimpleFireMessage& message)
{
  int64_t send_status = send(this->data_socket_, reinterpret_cast<void *>(&message), sizeof(message), 0);

  return send_status > 0;
}


bool OculusDriver::sendSimpleFireMessageVersion2(OculusDriver::SimpleFireMessageVersion2& message)
{
  int64_t send_status = send(this->data_socket_, reinterpret_cast<void *>(&message), sizeof(message), 0);

  return send_status > 0;
}


bool OculusDriver::socketHasDataIncoming(int32_t socket_file_descriptor, float timeout_duration)
{
  const int32_t kSecsToMicros = 1e6;
  float timeout_seconds = 0;
  float timeout_microseconds = modf(timeout_duration, &timeout_seconds) * kSecsToMicros;

  struct timeval time_value = {
    static_cast<int32_t>(timeout_seconds),
    static_cast<int32_t>(timeout_microseconds)
  };

  fd_set file_descriptor_set;
  FD_ZERO(&file_descriptor_set);
  FD_SET(socket_file_descriptor, &file_descriptor_set);

  int32_t status = select(socket_file_descriptor + 1, &file_descriptor_set, 0, 0, &time_value);

  return (status > 0);
}


bool OculusDriver::validateParameters()
{
  return true;
}


// ---------------------------------------------------------------------------------------------------------------------
// Threaded Functions
// ---------------------------------------------------------------------------------------------------------------------

void OculusDriver::statusSocketRetrieverThreadFunction()
{
  BOOST_LOG_TRIVIAL(trace) << "statusSocketRetrieverThreadFunction() - Hello, I am alive.";

  const uint32_t kReceiveDataBufferSize = 512;
  const uint8_t kInitialisationValue = 0;
  std::vector<uint8_t> raw_data_receive_buffer(kReceiveDataBufferSize, kInitialisationValue);

  OculusDriver::OculusStatusMessage status_message = {};

  // The Oculus outputs a StatusMessage at 1 Hz.
  // Run a bit faster than that in case we have multiple units to query.
  const std::chrono::duration<int64_t, std::ratio<1, 1000>> kLoopPeriodMs = std::chrono::milliseconds(100);
  std::chrono::time_point<std::chrono::steady_clock> next_loop_begin_time = std::chrono::steady_clock::now();

  while (this->happy_.load() == true)
  {
    socklen_t size_of_sockaddr_in_ = sizeof(this->status_sockaddr_in_);

    int64_t number_of_bytes_received = recvfrom(
      this->status_socket_,
      raw_data_receive_buffer.data(),
      raw_data_receive_buffer.size(),
      0,
      reinterpret_cast<struct sockaddr *>(&this->status_sockaddr_in_),
      &size_of_sockaddr_in_);

    if (number_of_bytes_received == static_cast<int32_t>(sizeof(OculusDriver::OculusStatusMessage)))
    {
      std::memcpy(&status_message, raw_data_receive_buffer.data(), sizeof(OculusDriver::OculusStatusMessage));

      if (this->oculus_identified_.load() == true)
      {
        // TODO(Phil Skelton): Fetch device health stuff.
      }
      else
      {
        // Identifier given by manufacturer for their Oculus products.
        if (status_message.oculus_header.oculus_id == this->kOculusId)
        {
          this->oculus_id_ = status_message.oculus_header.oculus_id;
          this->source_device_id_ = status_message.oculus_header.source_device_id;

          // Whether we are accepting by MAC or IP, we need the IP eventually for connecting the TCP socket.
          struct in_addr ipaddr = {};
          ipaddr.s_addr = status_message.ip_address;
          char * temp_buffer = new char[20];
          std::string device_ip = std::string(
            inet_ntop(
              AF_INET,
              reinterpret_cast<void *>(&ipaddr),
              temp_buffer,
              INET_ADDRSTRLEN));
          delete[] temp_buffer;

          // The user has entered a magical IP address that signals they want us to search by MAC address instead of IP.
          if (this->oculus_ip_address_.compare(this->kIpToUseMacAddressMatching) == 0)
          {
            std::vector<uint8_t> device_mac_bytes
            {
              status_message.mac_address_0,
              status_message.mac_address_1,
              status_message.mac_address_2,
              status_message.mac_address_3,
              status_message.mac_address_4,
              status_message.mac_address_5
            };

            // As discussed in https://stackoverflow.com/a/4738943, ether_ntoa seems to miss the leading zeros.
            // Source for ether_ntoa: https://code.woboq.org/userspace/glibc/inet/ether_ntoa_r.c.html.
            // Therefore, trivially write our own, I guess; needs to be neither fast nor pretty.
            std::ostringstream hex_stream;

            for (uint32_t index = 0; index < device_mac_bytes.size(); ++index)
            {
              hex_stream << std::hex << std::internal << std::setfill('0') << std::setw(2) << +device_mac_bytes[index];

              if (index < (device_mac_bytes.size() - 1))
              {
                hex_stream << ":";
              }
            }

            std::string device_mac = hex_stream.str();

            if (device_mac.compare(this->oculus_mac_address_) == 0)
            {
              BOOST_LOG_TRIVIAL(info) << "Accepted Oculus by user-specified MAC address.";

              this->oculus_ip_address_ = device_ip;

              BOOST_LOG_TRIVIAL(info)
                << "Resulting IP address is ["
                << this->oculus_ip_address_
                << "].";

              this->oculus_identified_.store(true);
            }
            else
            {
              // MAC addresses do not match; pass through to rate_keeper.
              BOOST_LOG_TRIVIAL(debug) << "statusSocketRetrieverThreadFunction() - "
                << "MAC address ["
                << device_mac
                << "] does not match what the user requested ["
                << this->oculus_mac_address_
                << "].";
            }
          }
          else
          {
            if (device_ip.compare(this->oculus_ip_address_) == 0)
            {
              BOOST_LOG_TRIVIAL(info) << "Accepted Oculus by user-specified IP address.";

              this->oculus_identified_.store(true);
            }
            else
            {
              BOOST_LOG_TRIVIAL(debug) << "statusSocketRetrieverThreadFunction() - "
                << "IP address ["
                << device_ip
                << "] does not match what the user requested ["
                << this->oculus_ip_address_
                << "].";
            }
          }
        }
        else
        {
          // Managed to find something that is NOT an Oculus product; pass through to rate_keeper.
        }
      }
    }
    else if (number_of_bytes_received == -1)
    {
      BOOST_LOG_TRIVIAL(warning) << "statusSocketRetrieverThreadFunction() - "
        << "Failed to receive data from socket.";
        // TODO(Phil Skelton): Add break for this.
    }
    else if (number_of_bytes_received == 0)
    {
      BOOST_LOG_TRIVIAL(fatal) << "statusSocketRetrieverThreadFunction() - "
        << "The socket server has disconnected.";

      this->propagateShutdown();
    }
    else
    {
      BOOST_LOG_TRIVIAL(warning) << "statusSocketRetrieverThreadFunction() - "
        << "Insufficient bytes for StatusMessage.";
    }

    next_loop_begin_time += kLoopPeriodMs;
    std::this_thread::sleep_until(next_loop_begin_time);
  }

  BOOST_LOG_TRIVIAL(trace) << "statusSocketRetrieverThreadFunction() - Goodbye, I am dead.";
}


void OculusDriver::dataSocketRetrieverThreadFunction()
{
  BOOST_LOG_TRIVIAL(trace) << "dataSocketRetrieverThreadFunction() - Hello, I am alive.";

  const uint32_t kReceiveDataBufferSize = 1000000;
  const uint8_t kInitialisationValue = 0;
  std::vector<uint8_t> raw_data_receive_buffer(kReceiveDataBufferSize, kInitialisationValue);

  // The maximum operating frequency of the Oculus M1200D is 40 Hz, so we want something faster than that.
  // TODO(Phil Skelton): Low power systems may require this to operate at a higher frequency. The way to determine if
  //                     this is the case is to monitor the queue size during operation. If you see that the queue is
  //                     not being emptied by the messageFinderAndUnpacker thread before the next message arrives, then
  //                     this may indicate that the socket reading is not happening fast enough.
  const std::chrono::duration<int64_t, std::ratio<1, 1000>> kLoopPeriodMs = std::chrono::milliseconds(10);
  std::chrono::time_point<std::chrono::steady_clock> next_loop_begin_time = std::chrono::steady_clock::now();

  while (this->happy_.load() == true)
  {
    if (this->keep_firing_assholes_.load() == true)
    {
      if (this->socketHasDataIncoming(this->data_socket_, this->kSocketTimeoutSecs))
      {
        auto number_of_bytes_received = recv(
          this->data_socket_,
          raw_data_receive_buffer.data(),
          raw_data_receive_buffer.size(),
          0);

        // Regardless of whether scanning is enabled or not, we need to make sure we empty the TCP buffer.
        // However, we only want to add the data to the buffer when scanning is actually enabled.
        if (number_of_bytes_received > 0)
        {
          if (this->scanning_enabled_.load() == true)
          {
            {
              std::lock_guard<std::mutex> local_lock(this->raw_data_queue_mutex_);

              std::copy(
                raw_data_receive_buffer.begin(),
                raw_data_receive_buffer.begin() + number_of_bytes_received,
                std::back_inserter(this->raw_data_queue_));
            }

            // There is no need to clear the buffer because we only ever read what we know we received from the start.

            BOOST_LOG_TRIVIAL(trace) << "dataSocketRetrieverThreadFunction() - "
              << "Received ["
              << number_of_bytes_received
              << "] Bytes; queue now has ["
              << this->raw_data_queue_.size()
              << "] Bytes.";
          }
          else
          {
            std::lock_guard<std::mutex> local_lock(this->raw_data_queue_mutex_);
            this->raw_data_queue_.clear();
            this->number_of_complete_messages_parsed_.store(0);
          }

          // Need to notify the other thread that's going to process the queue that there is now data.
          this->raw_data_queue_condition_variable_.notify_all();
        }
        else if (number_of_bytes_received == -1)
        {
          BOOST_LOG_TRIVIAL(error) << "dataSocketRetrieverThreadFunction() - "
            << "Failed to receive data from socket.";
        }
        else
        {
          BOOST_LOG_TRIVIAL(fatal) << "dataSocketRetrieverThreadFunction() - "
            << "The socket server has disconnected.";

          this->propagateShutdown();
        }
      }
      else
      {
        // Socket has no data incoming; fall through to rate_keeper.
      }
    }
    else
    {
      // We aren't currently firing; fall through to rate_keeper.
    }

    // You never do 'now() + period' because that leads to compounding delays.
    next_loop_begin_time += kLoopPeriodMs;
    std::this_thread::sleep_until(next_loop_begin_time);
  }

  BOOST_LOG_TRIVIAL(trace) << "dataSocketRetrieverThreadFunction() - Goodbye, I am dead.";
}


void OculusDriver::messageFinderAndUnpackerThreadFunction()
{
  BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - Hello, I am alive.";

  // We are looking for a fingerprint that indicates that an OculusMessageHeader is present.
  // Conveniently, the device (Oculus M1200D) tells us some necessary information on the Status port.
  // From OculusMessageHeader we can assume the first 3 fields are constant for an instantiation (validated in State 1):
  //   uint16_t oculus_id;
  //   uint16_t source_device_id;
  //   uint16_t destination_device_id; - This always seems to be 0x0000?
  // Furthermore, the fourth and fifth fields can be only one of a handful of known values (validated in State 2).
  //   uint16_t message_id;
  //   uint16_t message_version;
  // So, in total, we have 80 bits to validate and accept the header for further processing (performed in State 3).
  const std::vector<uint8_t> kTokens
  {
    static_cast<uint8_t>(this->oculus_id_ & 0x00ff),
    static_cast<uint8_t>((this->oculus_id_ >> 8) & 0x00ff),
    static_cast<uint8_t>(this->source_device_id_ & 0x00ff),
    static_cast<uint8_t>((this->source_device_id_ >> 8) & 0x00ff),
    0x00,
    0x00
  };

  const uint64_t kTokensSize = kTokens.size();

  // Some persistent variables that we require between states.
  bool possibly_found_a_header = false;
  bool found_a_valid_header = false;
  OculusDriver::OculusMessageHeader oculus_message_header = {};
  std::chrono::time_point<std::chrono::steady_clock> message_stamp = {};

  while (this->happy_.load() == true)
  {
    // This lock section will smartly sleep until the predicate is true and this thread is notified.
    {
      BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - "
        << "Sleeping on scanning_enabled_condition_variable_.";

      std::unique_lock<std::mutex> local_lock(this->scanning_enabled_mutex_);

      auto predicate = [this]()
      {
        return ((this->happy_.load() == false) || (this->scanning_enabled_.load() == true));
      };

      this->scanning_enabled_condition_variable_.wait(local_lock, predicate);

      local_lock.unlock();

      BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - "
        << "Woken by scanning_enabled_condition_variable_.";
    }

    // Need to check this after the previous loop, just in case we spuriously woke up.
    if (this->happy_.load() == false)
    {
      BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - "
        << "Driver is unhappy, breaking out...";

      break;
    }
    else if (this->scanning_enabled_.load() == false)
    {
      BOOST_LOG_TRIVIAL(debug) << "messageFinderAndUnpackerThreadFunction() - "
      << "Scanning is not enabled, continuing...";

      continue;
    }

    // This lock section will smartly sleep until notified of new data and the data queue contains sufficient data.
    {
      uint64_t size_required = 0;

      // We need different amounts of data for each state, so the predicate must change accordingly.
      if (!found_a_valid_header && !possibly_found_a_header)
      {
        size_required = kTokensSize;
      }
      else if (!found_a_valid_header && possibly_found_a_header)
      {
        size_required = sizeof(OculusDriver::OculusMessageHeader);
      }
      else if (found_a_valid_header && !possibly_found_a_header)
      {
        size_required = (sizeof(OculusDriver::OculusMessageHeader) + oculus_message_header.payload_size);
      }
      else
      {
        // This is not possible, so if something goes _very_ wrong, just break out of here.
        break;
      }

      BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - "
        << "Sleeping on raw_data_queue_condition_variable_; waiting for ["
        << size_required
        << "] Bytes.";

      std::unique_lock<std::mutex> local_lock(this->raw_data_queue_mutex_);

      auto predicate = [this, size_required]()
      {
        // The key predicate is whether the queue size is large enough.
        // However, we also need to protect against system states here.
        return (
          this->happy_.load() == false
          || this->scanning_enabled_.load() == false
          || this->raw_data_queue_.size() >= size_required);
      };

      this->raw_data_queue_condition_variable_.wait(local_lock, predicate);

      // We can release this lock now, because the rest of the code is protected by lock_guard's where required.
      // Not entirely necessary as it will unlock when it leaves scope, but for completeness, put it in.
      local_lock.unlock();

      BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - "
        << "Woken by raw_data_queue_condition_variable_.";
    }

    // Need to check this after the previous loop, just in case we spuriously woke up.
    if (this->happy_.load() == false)
    {
      break;
    }

    // If scanning has been disabled, we need to reset some things in this thread.
    if (this->scanning_enabled_.load() == false)
    {
      possibly_found_a_header = false;
      found_a_valid_header = false;
      oculus_message_header = {};
      message_stamp = {};
    }
    else if (this->scanning_enabled_.load() == true)
    {
      // ---------------------------------------------------------------------------------------------------------------
      // State 1: Attempt to find the tokens of a valid header and make it front of queue.
      // ---------------------------------------------------------------------------------------------------------------

      if (!found_a_valid_header && !possibly_found_a_header)
      {
        BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - "
          << "Entering state 1.";

        // While it is bad to hold the lock to a mutex for this long, the processing is actually quite fast, and there
        // isn't really another way to view and interrogate a snapshot of the queue without a copy.
        // When combined with the low update rate of the device, and the queueing nature of the socket, it seems
        // acceptable for this application.
        {
          std::lock_guard<std::mutex> local_lock(this->raw_data_queue_mutex_);

          auto first_token_iterator = std::find(
            this->raw_data_queue_.begin(),
            this->raw_data_queue_.end(),
            kTokens[0]);

          if (std::distance(first_token_iterator, this->raw_data_queue_.end())
            >= static_cast<int32_t>(kTokensSize))
          {
            auto next_iterator = first_token_iterator;

            for (uint32_t iteration = 1; iteration < kTokensSize; ++iteration)
            {
              next_iterator = std::next(next_iterator);
              possibly_found_a_header = (*next_iterator == kTokens[iteration]);

              if (!possibly_found_a_header)
              {
                // One of the tokens did not match, so start the process again by searching for the first token after
                // where we last found it.
                first_token_iterator = std::find(
                  std::next(first_token_iterator),
                  this->raw_data_queue_.end(),
                  kTokens[0]);

                if (std::distance(first_token_iterator, this->raw_data_queue_.end())
                  >= static_cast<int32_t>(kTokensSize))
                {
                  next_iterator = first_token_iterator;
                  iteration = 0;
                }
                else
                {
                  // Haven't been able to identify the first token; give up.
                  break;
                }
              }
              else
              {
                // We have possibly found a header; continue iterating.
              }
            }
          }
          else
          {
            // Don't have enough data to find the tokens; pass through.
          }

          if (possibly_found_a_header)
          {
            // We may have found a valid message, so timestamp this moment in case the message is accepted later.
            // The only way to get a more accurate timestamp is to stamp every packet received on the TCP socket.
            message_stamp = std::chrono::steady_clock::now();

            if (first_token_iterator == this->raw_data_queue_.begin())
            {
              BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - "
                << "Possibly found a header; it is already at the front of the queue.";
            }
            else
            {
              BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - "
                << "Possibly found a header; making it front of the queue.";

              // This is safe because we are still within the lock_guard from earlier.
              this->raw_data_queue_.erase(this->raw_data_queue_.begin(), first_token_iterator);
            }
          }
          else
          {
            BOOST_LOG_TRIVIAL(debug) << "messageFinderAndUnpackerThreadFunction() - "
              << "Could not find any headers; clearing the queue that has been searched.";

            // This is safe because we are still within the lock_guard from earlier, so the producer could not have
            // added more data to the queue.
            this->raw_data_queue_.clear();
          }
        }
      }


      // ---------------------------------------------------------------------------------------------------------------
      // State 2: Attempt to construct a full header and validate the contents.
      // ---------------------------------------------------------------------------------------------------------------

      if (!found_a_valid_header && possibly_found_a_header)
      {
        BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - "
          << "Entering state 2.";

        // Need to remember the queue_size so that later in this block we can erase appropriately.
        uint64_t queue_size = 0;

        std::vector<uint8_t> contiguous_slice;

        {
          std::lock_guard<std::mutex> local_lock(this->raw_data_queue_mutex_);
          queue_size = this->raw_data_queue_.size();

          if (queue_size >= sizeof(OculusDriver::OculusMessageHeader))
          {
            contiguous_slice = std::vector<uint8_t>
            {
              this->raw_data_queue_.begin(),
              this->raw_data_queue_.begin() + sizeof(OculusDriver::OculusMessageHeader)
            };
          }
          else
          {
            // Insufficient data to construct primitive header; pass through.
          }
        }

        if (contiguous_slice.size() == sizeof(OculusDriver::OculusMessageHeader))
        {
          bool reject_the_message = false;

          std::memcpy(
            &oculus_message_header,
            &(*contiguous_slice.begin()),
            sizeof(OculusDriver::OculusMessageHeader));

          if (oculus_message_header.message_id == this->MessageId::kSimplePingResult)
          {
            if (oculus_message_header.message_version == this->MessageVersion::kVersion2)
            {
              possibly_found_a_header = false;
              found_a_valid_header = true;

              BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - "
                << "SimplePingResultVersion2 found.";
            }
            else if (oculus_message_header.message_version == this->MessageVersion::kOriginal)
            {
              reject_the_message = true;

              BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - "
                << "SimplePingResult found; ignoring.";
            }
            else
            {
              BOOST_LOG_TRIVIAL(debug) << "messageFinderAndUnpackerThreadFunction() - "
                << "Unknown SimplePingResult message version ["
                << oculus_message_header.message_version
                << "].";

              reject_the_message = true;
            }
          }
          else
          {
            reject_the_message = true;
            possibly_found_a_header = false;

            if (oculus_message_header.message_id == this->MessageId::kUserConfig)
            {
              BOOST_LOG_TRIVIAL(debug) << "messageFinderAndUnpackerThreadFunction() - "
                << "UserConfig message; ignoring.";
            }
            else if (oculus_message_header.message_id == this->MessageId::kDummy)
            {
              BOOST_LOG_TRIVIAL(debug) << "messageFinderAndUnpackerThreadFunction() - "
                << "Dummy message; ignoring.";
            }
            else if (oculus_message_header.message_id == this->MessageId::kInitialisation)
            {
              BOOST_LOG_TRIVIAL(debug) << "messageFinderAndUnpackerThreadFunction() - "
                << "Initialisation message; ignoring.";
            }
            else
            {
              BOOST_LOG_TRIVIAL(debug) << "messageFinderAndUnpackerThreadFunction() - Oculus ID ["
                << std::hex << oculus_message_header.oculus_id
                << "].";

              BOOST_LOG_TRIVIAL(debug) << "messageFinderAndUnpackerThreadFunction() - Source Device ID ["
                << std::hex << oculus_message_header.source_device_id
                << "].";

              BOOST_LOG_TRIVIAL(debug) << "messageFinderAndUnpackerThreadFunction() - Destination Device ID ["
                << std::hex << oculus_message_header.destination_device_id
                << "].";

              BOOST_LOG_TRIVIAL(debug) << "messageFinderAndUnpackerThreadFunction() - Unknown message ID ["
                << std::hex << oculus_message_header.message_id
                << "].";
            }
          }

          // The header isn't what we want, so delete the tokens so that we don't trigger on them again.
          if (reject_the_message)
          {
            std::lock_guard<std::mutex> local_lock(this->raw_data_queue_mutex_);

            // The only way for the queue to reduce in size between now and when the queue_size was last checked
            // is if the producer has been told to stop and has cleared the queue, so we need to catch that.
            if (this->raw_data_queue_.size() >= queue_size)
            {
              this->raw_data_queue_.erase(
                this->raw_data_queue_.begin(),
                this->raw_data_queue_.begin() + static_cast<int32_t>(kTokensSize));
            }
          }
          else
          {
            // Message header has been accepted; pass through.
          }
        }
        else
        {
          // Somehow the slice contains less data than was requested; pass through.
        }
      }


      // ---------------------------------------------------------------------------------------------------------------
      // State 3: Attempt to construct a full message.
      // ---------------------------------------------------------------------------------------------------------------

      if (found_a_valid_header && !possibly_found_a_header)
      {
        BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - "
          << "Entering state 3.";

        auto required_message_size = sizeof(OculusDriver::OculusMessageHeader) + oculus_message_header.payload_size;
        std::vector<uint8_t> contiguous_slice;

        {
          std::lock_guard<std::mutex> local_lock(this->raw_data_queue_mutex_);

          // We might as well only pull data out if the entire payload is ready.
          // By definition, the following are equivalent:
          //   (sizeof(OculusDriver::OculusMessageHeader) + oculus_message_header.payload_size)
          //   == SimplePingResultVersion2.message_size
          // Therefore, there's no real benefit to first pulling the header before the entire message.
          if (this->raw_data_queue_.size() >= required_message_size)
          {
            // The std::queue is not guaranteed to be contiguous in memory, so pull the message into a std::vector,
            // which is guaranteed.
            contiguous_slice = std::vector<uint8_t>
            {
              this->raw_data_queue_.begin(),
              this->raw_data_queue_.begin() + static_cast<int32_t>(required_message_size)
            };

            this->raw_data_queue_.erase(
              this->raw_data_queue_.begin(),
              this->raw_data_queue_.begin() + static_cast<int32_t>(required_message_size)
            );
          }
          else
          {
            // Insufficient bytes to construct message; pass through.
          }
        }

        if (contiguous_slice.size() == required_message_size)
        {
          OculusDriver::SimplePingResultVersion2 simple_ping_result_version_2 = {};

          std::memcpy(
            &simple_ping_result_version_2,
            &(*contiguous_slice.begin()),
            sizeof(SimplePingResultVersion2));

          // At this point we are going to transition from the PACKED data structures that the data comes in as to some
          // NON_PACKED data structures, because that is what the system wants to work with.
          std::shared_ptr<OculusDriver::DataPacket> data_packet = std::make_shared<OculusDriver::DataPacket>();
          data_packet->message_stamp = message_stamp;
          data_packet->simple_ping_result = simple_ping_result_version_2;


          // -----------------------------------------------------------------------------------------------------------
          // Convert the device bearing outputs into usable azimuth values.
          // -----------------------------------------------------------------------------------------------------------

          data_packet->azimuths.reserve(data_packet->simple_ping_result.beam_count);

          // In this instance, `auto` helps with arm/x86 differences.
          auto header_offset = sizeof(SimplePingResultVersion2);

          for (uint32_t index = 0; index < data_packet->simple_ping_result.beam_count; ++index)
          {
            uint32_t step = 2 * index;

            uint8_t first_byte = contiguous_slice[header_offset + step];
            uint8_t second_byte = contiguous_slice[header_offset + step + 1];

            uint16_t shifted_first_byte = static_cast<uint16_t>(first_byte) & 0x00FF;
            uint16_t shifted_second_byte = static_cast<uint16_t>(second_byte << 8) & 0xFF00;

            uint16_t combined_value = shifted_first_byte | shifted_second_byte;

            double device_bearing_as_azimuth = this->deviceBearingToAzimuth(combined_value);

            data_packet->azimuths.push_back(device_bearing_as_azimuth);
          }


          // -----------------------------------------------------------------------------------------------------------
          // Unpack the raw data into the format required.
          // -----------------------------------------------------------------------------------------------------------

#ifdef __x86_64__
          auto data_offset = contiguous_slice.begin() + data_packet->simple_ping_result.image_start_offset;

          std::vector<uint8_t> raw_data = std::vector<uint8_t>(
            data_offset,
            data_offset + data_packet->simple_ping_result.image_size);
#elif __arm__
          auto data_offset = contiguous_slice.begin() +
                             static_cast<int32_t>(data_packet->simple_ping_result.image_start_offset);

          std::vector<uint8_t> raw_data = std::vector<uint8_t>(
            data_offset,
            data_offset + static_cast<int32_t>(data_packet->simple_ping_result.image_size));
#endif

          // Compilers are generally quite good at vectorising and optimising these operations.
          // If you notice that this thread is taking too long, look into this code block.
          // Some things may seem backwards, but that is how the data comes out of the sensor.
          if (data_packet->simple_ping_result.data_size == this->DataSize::kData8Bit)
          {
            for (uint32_t beam = 0; beam < data_packet->simple_ping_result.beam_count; ++beam)
            {
              std::vector<uint8_t> range_vector;
              range_vector.reserve(data_packet->simple_ping_result.range_count);
              data_packet->unpacked_8_bit_data.push_back(range_vector);
            }

            uint32_t index = 0;

            for (uint32_t range = 0; range < data_packet->simple_ping_result.range_count; ++range)
            {
              for (uint32_t beam = 0; beam < data_packet->simple_ping_result.beam_count; ++beam)
              {
                data_packet->unpacked_8_bit_data[beam].push_back(raw_data[index++]);
              }
            }
          }
          else if (data_packet->simple_ping_result.data_size == this->DataSize::kData16Bit)
          {
            for (uint32_t beam = 0; beam < data_packet->simple_ping_result.beam_count; ++beam)
            {
              std::vector<uint16_t> range_vector;
              range_vector.reserve(data_packet->simple_ping_result.range_count);
              data_packet->unpacked_16_bit_data.push_back(range_vector);
            }

            uint32_t index = 0;

            for (uint32_t range = 0; range < data_packet->simple_ping_result.range_count; ++range)
            {
              for (uint32_t beam = 0; beam < data_packet->simple_ping_result.beam_count; ++beam)
              {
                uint8_t first_byte = raw_data[index++];
                uint8_t second_byte = raw_data[index++];

                uint16_t shifted_first_byte = static_cast<uint16_t>(first_byte) & 0x00FF;
                uint16_t shifted_second_byte = static_cast<uint16_t>(second_byte << 8) & 0xFF00;

                uint16_t combined_value = shifted_first_byte | shifted_second_byte;

                data_packet->unpacked_16_bit_data[beam].push_back(combined_value);
              }
            }
          }
          else if (data_packet->simple_ping_result.data_size == this->DataSize::kData32Bit)
          {
            for (uint32_t beam = 0; beam < data_packet->simple_ping_result.beam_count; ++beam)
            {
              std::vector<uint32_t> range_vector;
              range_vector.reserve(data_packet->simple_ping_result.range_count);
              data_packet->unpacked_32_bit_data.push_back(range_vector);
            }

            uint32_t index = 0;

            for (uint32_t range = 0; range < data_packet->simple_ping_result.range_count; ++range)
            {
              for (uint32_t beam = 0; beam < data_packet->simple_ping_result.beam_count; ++beam)
              {
                uint8_t first_byte = raw_data[index++];
                uint8_t second_byte = raw_data[index++];
                uint8_t third_byte = raw_data[index++];
                uint8_t fourth_byte = raw_data[index++];

                uint32_t shifted_first_byte = static_cast<uint32_t>(first_byte) & 0x000000FF;
                uint32_t shifted_second_byte = static_cast<uint32_t>(second_byte << 8) & 0x0000FF00;
                uint32_t shifted_third_byte = static_cast<uint32_t>(third_byte << 16) & 0x00FF0000;
                uint32_t shifted_fourth_byte = static_cast<uint32_t>(fourth_byte << 24) & 0xFF000000;

                uint32_t combined_value = (
                  shifted_first_byte
                  | shifted_second_byte
                  | shifted_third_byte
                  | shifted_fourth_byte
                );

                data_packet->unpacked_32_bit_data[beam].push_back(combined_value);
              }
            }
          }
          else
          {
            // Something has gone horribly wrong if we end up here.
          }


          // -----------------------------------------------------------------------------------------------------------
          // Put it onto the queue for the publisher consumers to process.
          // -----------------------------------------------------------------------------------------------------------

          this->data_packet_queue_.push(data_packet);

          BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - "
            << "New SimplePingResult packet created; notifying processors.";

          this->data_packet_queue_condition_variable_.notify_all();
          this->number_of_complete_messages_parsed_++;

          found_a_valid_header = false;
          possibly_found_a_header = false;
        }
        else
        {
          // Insufficient bytes to construct message; pass through.
        }
      }
      else
      {
        // No valid header has been found; pass through.
      }
    }
    else
    {
      // Scanning was disabled while waiting for the lock; pass through.
    }
  }

  BOOST_LOG_TRIVIAL(trace) << "messageFinderAndUnpackerThreadFunction() - "
   << "Goodbye, I am dead.";
}


void OculusDriver::simpleFireMessageSenderThreadFunction()
{
  BOOST_LOG_TRIVIAL(trace) << "simpleFireMessageSenderThreadFunction() - "
    << "Hello, I am alive.";

  const uint8_t kSuccessiveFailureLimit = 2;
  uint8_t successive_failure_count = 0;

  OculusDriver::SimpleFireMessageVersion2 simple_fire_message_version_2 = {};

  // The Oculus needs to receive a SimpleFireMessage at something greater than 1 Hz.
  const std::chrono::duration<int64_t, std::ratio<1, 1000>> kLoopPeriodMs = std::chrono::milliseconds(500);
  std::chrono::time_point<std::chrono::steady_clock> next_loop_begin_time = std::chrono::steady_clock::now();

  while (this->happy_.load() == true)
  {
    if (this->keep_firing_assholes_.load() == true)
    {
      // We need to be able to atomically read the flag for a change in parameters and then set that flag without
      // something else overwriting it.
      // Therefore, we will lock the global mutex for parameter changes.
      // While we could use an atomic variable to guarantee no collisions, this would not catch the case where this
      // thread checks for condition and then sets the variable a moment later. It is entirely plausible that another
      // thread could access the atomic variable between the two calls in this thread, thus discarding a valid
      // parameter change request.
      {
        std::lock_guard<std::mutex> local_lock(this->parameter_change_mutex_);

        if (this->parameters_have_changed_)
        {
          // simple_fire_message = this->generateSimpleFireMessage();
          simple_fire_message_version_2 = this->generateSimpleFireMessageVersion2();
          this->parameters_have_changed_ = false;
        }
      }

      // bool success = this->sendSimpleFireMessage(simple_fire_message);
      bool success = this->sendSimpleFireMessageVersion2(simple_fire_message_version_2);

      if (success)
      {
        BOOST_LOG_TRIVIAL(trace) << "simpleFireMessageSenderThreadFunction() - "
          << "Successfully sent SimpleFireMessage.";

        successive_failure_count = 0;
      }
      else
      {
        ++successive_failure_count;

        BOOST_LOG_TRIVIAL(error) << "simpleFireMessageSenderThreadFunction() - "
          << "Failed to send a SimpleFireMessage, count ["
          << +successive_failure_count
          << "].";

        if (successive_failure_count >= kSuccessiveFailureLimit)
        {
          BOOST_LOG_TRIVIAL(fatal) << "simpleFireMessageSenderThreadFunction() - "
            << "Successive failure limit reached; terminating program.";

          this->propagateShutdown();
        }
      }
    }
    else
    {
      // Falls through to the primary rate keeper below.
    }

    next_loop_begin_time += kLoopPeriodMs;
    std::this_thread::sleep_until(next_loop_begin_time);
  }

  BOOST_LOG_TRIVIAL(trace) << "simpleFireMessageSenderThreadFunction() - Goodbye, I am dead.";
}


// ---------------------------------------------------------------------------------------------------------------------
// Setter Functions.
// ---------------------------------------------------------------------------------------------------------------------

/*!
 * Setter function for the FrequencyMode parameter.
 *
 * This accepts a std::string representation of "low" or "high" and abstracts away the underlying enumerations.
 *
 * @param[in] requested_value The requested value.
 */
void OculusDriver::setFrequencyMode(std::string requested_value)
{
  BOOST_LOG_TRIVIAL(debug) << "setFrequencyMode() - "
    << "Frequency mode requested ["
    << requested_value
    << "].";

  this->frequency_mode_string_ = requested_value;

  uint8_t parsed_frequency_mode = (
    (requested_value == "low")
    ? static_cast<uint8_t>(this->FrequencyMode::kLow)
    : static_cast<uint8_t>(this->FrequencyMode::kHigh)
  );

  OculusDriver::FrequencyMode flagged_frequency_mode = static_cast<OculusDriver::FrequencyMode>(parsed_frequency_mode);

  if ((flagged_frequency_mode == this->kFrequencyModeLimits.first)
    || (flagged_frequency_mode == this->kFrequencyModeLimits.second))
  {
    this->frequency_mode_ = flagged_frequency_mode;
    this->guardedUpdateToParametersChangedFlag();

    BOOST_LOG_TRIVIAL(info)
      << "Frequency mode set to ["
      << requested_value
      << "].";
  }
  else
  {
    BOOST_LOG_TRIVIAL(fatal) << "setFrequencyMode() - "
      << "Requested mode ["
      << requested_value
      << "] is outside of the range ["
      << this->kFrequencyModeLimits.first
      << ", "
      << this->kFrequencyModeLimits.second
      << "].";

    this->propagateShutdown();
  }
}


/*!
 * Setter function for the Gain parameter.
 *
 * This accepts a requested gain value and performs bound checking.
 *
 * @param[in] requested_value The requested value.
 */
void OculusDriver::setGain(double requested_value)
{
  BOOST_LOG_TRIVIAL(debug) << "setGain() - "
    << "Gain requested ["
    << requested_value
    << "] %.";

  const uint32_t kPromotionFactor = 1e6;
  uint32_t requested_promoted = static_cast<uint32_t>(requested_value * kPromotionFactor);
  uint32_t lower_limit_promoted = static_cast<uint32_t>(this->kGainLimits.first * kPromotionFactor);
  uint32_t upper_limit_promoted = static_cast<uint32_t>(this->kGainLimits.second * kPromotionFactor);

  if ((requested_promoted >= lower_limit_promoted) && (requested_promoted <= upper_limit_promoted))
  {
    this->gain_percent_ = requested_value;
    this->guardedUpdateToParametersChangedFlag();

    BOOST_LOG_TRIVIAL(info) << "Gain set to ["
      << this->gain_percent_
      << "].";
  }
  else
  {
    BOOST_LOG_TRIVIAL(fatal) << "setGain() - "
      << "Requested gain ["
      << requested_value
      << "] is outside of the range ["
      << this->kGainLimits.first
      << ", "
      << this->kGainLimits.second
      << "].";

    this->propagateShutdown();
  }
}


/*!
 * Setter function for the Gamma parameter.
 *
 * This accepts a uint8_t representation of gamma.
 * From the SDK:
 *   127 (0x7F) results in a gamma of 0.5.
 *   255 (0xFF) results in a gamma of 1.0.
 * It would be assumed that it is a linear relationship (e.g. 191 (0xBF) would be 0.75).
 * This has not been tested and should generally be set to 255 (0xFF).
 *
 * @param[in] requested_value The requested value.
 */
void OculusDriver::setGamma(uint8_t requested_value)
{
  BOOST_LOG_TRIVIAL(debug) << "setGamma() - "
    << "Gamma requested ["
    << requested_value
    << "] %.";

  this->gamma_ = requested_value;
  this->guardedUpdateToParametersChangedFlag();

  BOOST_LOG_TRIVIAL(info)
    << "Gamma set to ["
    << +this->gamma_
    << "].";
}


/*!
 * Setter function for the finding device by IP address.
 *
 * This accepts an IPv4 compliant std::string representation in the form of "###.###.###.###".
 *
 * @param[in] requested_value The requested value.
 */
void OculusDriver::setOculusIpAddress(std::string requested_value)
{
  BOOST_LOG_TRIVIAL(debug) << "setOculusIpAddress - "
    << "IP address requested ["
    << requested_value
    << "].";

  // Interrogation as to the validity of the IP address is handled by the socket creation function.
  this->oculus_ip_address_ = requested_value;

  BOOST_LOG_TRIVIAL(info)
    << "IP address for finding Oculus set to ["
    << this->oculus_ip_address_
    << "].";
}


void OculusDriver::setOculusMacAddress(std::string requested_value)
{
  BOOST_LOG_TRIVIAL(debug) << "setOculusMacAddress() - "
    << "MAC address requested ["
    << requested_value
    << "].";

  // Interrogation as to the validity of the MAC address is handled by the socket creation function.
  this->oculus_mac_address_ = requested_value;

  BOOST_LOG_TRIVIAL(info) << "MAC address for finding Oculus set to ["
    << requested_value
    << "].";
}


void OculusDriver::setPingRate(uint8_t requested_value)
{
  BOOST_LOG_TRIVIAL(debug) << "setPingRate() - "
    << "Ping rate requested ["
    << +requested_value
    << "] Hz.";

  // Internally we use strongly-typed enumerators, but users want an understandable parameter (e.g. frequency in Hz),
  // not a random enumerator value.
  // This makes more work for us in implementation, but makes usability so much better.
  OculusDriver::PingRateHz ping_rate_hz = static_cast<OculusDriver::PingRateHz>(requested_value);

  // Dummy initialisation to avoid [-Werror=maybe-uninitialized] from else{} statement.
  OculusDriver::PingRateFlag ping_rate_flag = this->PingRateFlag::kStandby;

  if (ping_rate_hz == this->PingRateHz::kStandby)
  {
    ping_rate_flag = this->PingRateFlag::kStandby;
  }
  else if (ping_rate_hz == this->PingRateHz::kLowest)
  {
    ping_rate_flag = this->PingRateFlag::kLowest;
  }
  else if (ping_rate_hz == this->PingRateHz::kLow)
  {
    ping_rate_flag = this->PingRateFlag::kLow;
  }
  else if (ping_rate_hz == this->PingRateHz::kNormal)
  {
    ping_rate_flag = this->PingRateFlag::kNormal;
  }
  else if (ping_rate_hz == this->PingRateHz::kHigh)
  {
    ping_rate_flag = this->PingRateFlag::kHigh;
  }
  else if (ping_rate_hz == this->PingRateHz::kHighest)
  {
    ping_rate_flag = this->PingRateFlag::kHighest;
  }
  else
  {
    BOOST_LOG_TRIVIAL(fatal) << "setPingRate() - "
      << "Requested ping rate ["
      << requested_value
      << "] is outside of the range ["
      << +static_cast<uint8_t>(this->kPingRateHzLimits.first)
      << ", "
      << +static_cast<uint8_t>(this->kPingRateHzLimits.second)
      << "].";

    this->propagateShutdown();
  }

  this->ping_rate_flag_ = ping_rate_flag;
  this->ping_rate_hz_ = ping_rate_hz;

  if (this->ping_rate_flag_ == this->PingRateFlag::kStandby)
  {
    this->scanning_enabled_.store(false);
  }
  else
  {
    this->scanning_enabled_.store(true);
  }

  this->scanning_enabled_condition_variable_.notify_all();
  this->guardedUpdateToParametersChangedFlag();

  BOOST_LOG_TRIVIAL(info)
    << "Ping rate set to ["
    << +static_cast<uint8_t>(this->ping_rate_hz_)
    << "] Hz.";
}


void OculusDriver::setRangeUpperLimit(double requested_value)
{
  BOOST_LOG_TRIVIAL(debug) << "setRangeUpperLimit() - "
    << "Range upper limit requested ["
    << requested_value
    << "] metres.";

  bool valid_range_for_current_frequency = false;

  if (this->frequency_mode_ == this->FrequencyMode::kLow)
  {
    // If you ever want impossible bugs to find, do float == float comparisons.
    // This is why we run the -Wfloat-equal compilation flag.
    // One way of dealing with it is to promote the float to an integer at a given level of accuracy.
    const uint32_t kPromotionFactor = 1e6;
    uint32_t requested_promoted = static_cast<uint32_t>(requested_value * kPromotionFactor);
    uint32_t lower_limit_promoted = static_cast<uint32_t>(this->kRangeLimitsAtLowFrequency.first * kPromotionFactor);
    uint32_t upper_limit_promoted = static_cast<uint32_t>(this->kRangeLimitsAtLowFrequency.second * kPromotionFactor);

    if ((requested_promoted >= lower_limit_promoted) && (requested_promoted <= upper_limit_promoted))
    {
      valid_range_for_current_frequency = true;
    }
  }
  else
  {
    // If you ever want impossible bugs to find, do float == float comparisons.
    // This is why we run the -Wfloat-equal compilation flag.
    // One way of dealing with it is to promote the float to an integer at a given level of accuracy.
    const uint32_t kPromotionFactor = 1e6;
    uint32_t requested_promoted = static_cast<uint32_t>(requested_value * kPromotionFactor);
    uint32_t lower_limit_promoted = static_cast<uint32_t>(this->kRangeLimitsAtHighFrequency.first * kPromotionFactor);
    uint32_t upper_limit_promoted = static_cast<uint32_t>(this->kRangeLimitsAtHighFrequency.second * kPromotionFactor);

    if ((requested_promoted >= lower_limit_promoted) && (requested_promoted <= upper_limit_promoted))
    {
      valid_range_for_current_frequency = true;
    }
  }

  if (valid_range_for_current_frequency)
  {
    this->range_upper_limit_ = requested_value;
    this->guardedUpdateToParametersChangedFlag();

    BOOST_LOG_TRIVIAL(info)
      << "Range upper limit set to ["
      << this->range_upper_limit_
      << "] metres.";
  }
  else
  {
    if (this->frequency_mode_ == this->FrequencyMode::kLow)
    {
      BOOST_LOG_TRIVIAL(fatal) << "setRangeUpperLimit() - "
        << "Requested range ["
        << requested_value
        << "] is outside of the range ["
        << +static_cast<uint8_t>(this->kRangeLimitsAtLowFrequency.first)
        << ", "
        << +static_cast<uint8_t>(this->kRangeLimitsAtLowFrequency.second)
        << "].";
    }
    else
    {
      BOOST_LOG_TRIVIAL(fatal) << "setRangeUpperLimit() - "
        << "Requested range ["
        << requested_value
        << "] is outside of the range ["
        << +static_cast<uint8_t>(this->kRangeLimitsAtHighFrequency.first)
        << ", "
        << +static_cast<uint8_t>(this->kRangeLimitsAtHighFrequency.second)
        << "].";
    }

    this->propagateShutdown();
  }
}


void OculusDriver::setSalinityPPT(double requested_value)
{
  BOOST_LOG_TRIVIAL(debug) << "setSalinityPPT() - "
    << "Salinity requested ["
    << +requested_value
    << "] parts/thousand.";

  if (!this->floatsAreEqual(this->speed_of_sound_, this->kDeviceCalculatesSpeedOfSound))
  {
    BOOST_LOG_TRIVIAL(fatal) << "setSalinityPPT() - "
      << "You have previously specified a speed of sound value, and that takes precedence over salinity.";

    this->propagateShutdown();
  }
  else
  {
    //TODO(Phil Skelton): Implement sanity bound checking.
    this->salinity_ = requested_value;
    this->guardedUpdateToParametersChangedFlag();

    BOOST_LOG_TRIVIAL(info)
      << "Salinity set to ["
      << +requested_value
      << "] parts/thousand.";
  }
}


void OculusDriver::setSpeedOfSound(double requested_value)
{
  BOOST_LOG_TRIVIAL(debug) << "setSpeedOfSound() - "
    << "Speed of sound requested ["
    << +requested_value
    << "] metres/second.";

  if (!this->floatsAreEqual(this->salinity_, this->kSalinityOfSaltWaterPPT))
  {
    BOOST_LOG_TRIVIAL(fatal) << "setSpeedOfSound() - "
      << "You have previously specified a salinity value, but speed of sound takes precedence over salinity.";

    this->propagateShutdown();
  }
  else
  {
    if ((requested_value >= this->kArbitrarySpeedOfSoundLimits.first) &&
        (requested_value <= this->kArbitrarySpeedOfSoundLimits.second))
    {
      this->speed_of_sound_ = requested_value;
      this->guardedUpdateToParametersChangedFlag();

      BOOST_LOG_TRIVIAL(info)
        << "Speed of sound set to ["
        << +requested_value
        << "] metres/second.";
    }
    else
    {
      BOOST_LOG_TRIVIAL(fatal) << "setSpeedOfSound() - "
        << "Requested speed of sound ["
        << requested_value
        << "] is outside of the range ["
        << this->kArbitrarySpeedOfSoundLimits.first
        << ", "
        << this->kArbitrarySpeedOfSoundLimits.second
        << "] metres/second.";

      this->propagateShutdown();
    }
  }
}


void OculusDriver::setDataDepth(uint8_t requested_value)
{
  BOOST_LOG_TRIVIAL(debug) << "setDataDepth() - "
    << "Data depth requested ["
    << +requested_value
    << "] bits.";

  switch (requested_value)
  {
    case (8):
    {
      this->data_depth_ = 8;
      this->use_16_bit_data_mode_ = false;
      this->use_32_bit_data_mode_ = false;

      break;
    }
    case (16):
    {
      this->data_depth_ = 16;
      this->use_16_bit_data_mode_ = true;
      this->use_32_bit_data_mode_ = false;

      break;
    }
    case (32):
    {
      this->data_depth_ = 32;
      this->use_16_bit_data_mode_ = false;
      this->use_32_bit_data_mode_ = true;

      break;
    }
    default:
    {
      BOOST_LOG_TRIVIAL(fatal) << "setDataDepth() - "
        << "Requested data depth ["
        << +requested_value
        << "] is not one of the accepted values [8, 16, 32].";

      this->propagateShutdown();
    }
  }

  this->guardedUpdateToParametersChangedFlag();

  BOOST_LOG_TRIVIAL(info)
    << "Data depth set to ["
    << +requested_value
    << "] bits.";
}

