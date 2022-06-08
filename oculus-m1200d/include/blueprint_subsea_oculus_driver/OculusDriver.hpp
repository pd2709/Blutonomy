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
* @file OculusDriver.hpp
*/


#ifndef BLUEPRINT_SUBSEA_OCULUS_DRIVER_OCULUS_DRIVER_HPP
#define BLUEPRINT_SUBSEA_OCULUS_DRIVER_OCULUS_DRIVER_HPP

#if !defined(__linux__)
#error "Only Linux operating systems are supported."
#endif

// Older systems need to be told to enable the definitions in the system "math.h" library.
#define _USE_MATH_DEFINES
#include <math.h>

#include <algorithm>
#include <arpa/inet.h>
#include <atomic>
#include <bitset>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <netdb.h>
#include <netinet/ether.h>
#include <queue>
#include <signal.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <type_traits>
#include <unistd.h>
#include <utility>
#include <vector>


class OculusDriver
{
 protected:

  // -------------------------------------------------------------------------------------------------------------------
  // PACKED Enumerations.
  // Most of these are pulled from the Oculus.h file in the SDK and then modified.
  // These are PACKED because they are network data and it is convenient to be able to use std::memcpy for storage.
  // However, these are detrimental to performance because hardware does not like doing packed things.
  // -------------------------------------------------------------------------------------------------------------------

  //! A strongly typed enum class representing data size flags.
  enum class __attribute__((__packed__)) DataSize : uint8_t
  {
    kData8Bit = 0x00, /*!< 0x00 */
    kData16Bit = 0x01, /*!< 0x01 */
    kData24Bit = 0x02, /*!< 0x02 */
    kData32Bit = 0x03 /*!< 0x03 */
  };

  // -------------------------------------------------------------------------------------------------------------------

  //! A strongly typed enum class representing frequency mode flags.
  enum class __attribute__((__packed__)) FrequencyMode : uint8_t
  {
    kLow = 0x01, /*!< 0x01 */
    kHigh = 0x02  /*!< 0x02 */
  };

  // -------------------------------------------------------------------------------------------------------------------

  //! A strongly typed enum class representing message version flags.
  enum class __attribute__((__packed__)) MessageVersion : uint16_t
  {
    kOriginal = 0x0000, /*!< 0x0000 */
    kVersion2 = 0x0002 /*!< 0x0002 */
  };

  // -------------------------------------------------------------------------------------------------------------------

  //! A strongly typed enum class representing message ID flags.
  /*! This changed from uint8_t to uint16_t somewhere between SDK 1.8.x and 1.9.x; that was fun to figure out. */
  enum class __attribute__((__packed__)) MessageId : uint16_t
  {
    kStatus = 0x0001, /*!< 0x0001 */
    kSimpleFire = 0x0015, /*!< 0x0015 */
    kPingResult = 0x0022, /*!< 0x0022 */
    kSimplePingResult = 0x0023, /*!< 0x0023 */
    kUserConfig = 0x0055, /*!< 0x0055 */
    kInitialisation = 0x0080, /*!< 0x0080 */
    kDummy = 0x00ff /*!< 0x00ff */
  };

  // -------------------------------------------------------------------------------------------------------------------

  //! A strongly typed enum class representing option flags.
  enum class __attribute__((__packed__)) OptionFlag : uint8_t
  {
    kNoChange = 0x00,
    kUseMetres = 0x01,
    kUse16BitData = 0x02,
    kReportGain = 0x04,
    kSimpleReturn = 0x08,
    kUseGainAssist = 0x10,
    kReducePower = 0x20,
    kEnable512Beams = 0x40
  };

  // -------------------------------------------------------------------------------------------------------------------

  //! A strongly typed enum class representing extra option flags.
  enum class __attribute__((__packed__)) ExtraOptionFlag : uint32_t
  {
    kNoChange = 0x00000000,
    kResearchDataMode = 0x00000200
  };

  // -------------------------------------------------------------------------------------------------------------------

  //! A strongly typed enum class representing ping rate flags.
  enum class __attribute__((__packed__)) PingRateFlag : uint8_t
  {
    kNormal = 0x00,
    kHigh = 0x01,
    kHighest = 0x02,
    kLow = 0x03,
    kLowest = 0x04,
    kStandby = 0x05
  };

  // -------------------------------------------------------------------------------------------------------------------

  //! A strongly typed enum class representing ping rate numerical values.
  enum class __attribute__((__packed__)) PingRateHz : uint8_t
  {
    kStandby = 0,
    kLowest = 2,
    kLow = 5,
    kNormal = 10,
    kHigh = 15,
    kHighest = 40
  };


  // -------------------------------------------------------------------------------------------------------------------

  //! A strongly typed enum class representing Oculus master status flags.
  enum class __attribute__((__packed__)) OculusMasterStatus : uint8_t
  {
    kSsblBoot = 0,
    kSsblRun = 1,
    kMainBoot = 2,
    kMainRun = 3
  };

  // -------------------------------------------------------------------------------------------------------------------

  //! A strongly typed enum class representing Oculus pause reason flags.
  enum class __attribute__((__packed__)) OculusPauseReason : uint8_t
  {
    kMagSwitch = 0x00,
    kBootFromMain = 0x01,
    kFlashError = 0x02,
    kJtagLoad = 0x03
  };

  // -------------------------------------------------------------------------------------------------------------------

  //! A strongly typed enum class representing Oculus temperature status flags.
  enum class __attribute__((__packed__)) OculusTemperatureStatus : uint8_t
  {
    kGood = 0x00,
    kOverheat = 0x01,
    kReserved = 0x02,
    kMaximumExceeded = 0x03
  };

  // -------------------------------------------------------------------------------------------------------------------

  //! A strongly typed enum class representing Oculus device type flags.
  enum class __attribute__((__packed__)) OculusDevice : uint16_t
  {
    kUndefined = 0x0000,
    kImagingSonar = 0x0001
  };

  // -------------------------------------------------------------------------------------------------------------------

  //! A strongly typed enum class representing Oculus device part number flags.
  enum class __attribute__((__packed__)) OculusPartNumber : uint16_t
  {
    kUndefined = 0,
    kM370s = 1041,
    kM370s_Artemis = 1229,
    kM370s_Deep = 1217,
    kM373s = 1209,
    kM373s_Deep = 1218,
    kM750d = 1032,
    kM750d_Fusion = 1134,
    kM750d_Artemis = 1135,
    kM1200d = 1042,
    kM1200d_Deep = 1219,
    kM1200d_Artemis = 1228,
    kN1200s = 1220,
    kN1200s_Deep = 1221,
    kEnd = 0xffff
  };


  // -------------------------------------------------------------------------------------------------------------------
  // PACKED Structures.
  // Most of these are pulled from the Oculus.h file in the SDK and then modified.
  // These are PACKED because they are network data and it is convenient to be able to use std::memcpy for storage.
  // However, these are detrimental to performance because hardware does not like doing packed things.
  // -------------------------------------------------------------------------------------------------------------------

  struct __attribute__((__packed__)) OculusMessageHeader
  {
    uint16_t oculus_id;
    uint16_t source_device_id;
    uint16_t destination_device_id;
    MessageId message_id;
    MessageVersion message_version;
    uint32_t payload_size;
    uint16_t unused;
  };

  // -------------------------------------------------------------------------------------------------------------------

  struct __attribute__((__packed__)) UserConfig
  {
    uint32_t address;
    uint32_t mask;
    uint32_t dhcp;
  };

  // -------------------------------------------------------------------------------------------------------------------

  struct __attribute__((__packed__)) ConfigMessage
  {
    OculusMessageHeader oculus_header;
    UserConfig config;
  };

  // -------------------------------------------------------------------------------------------------------------------

  struct __attribute__((__packed__)) SimpleFireMessage
  {
    OculusMessageHeader oculus_header;
    FrequencyMode frequency_mode;
    PingRateFlag ping_rate;
    uint8_t network_speed;
    uint8_t gamma;
    OptionFlag flags;
    double range_upper_limit;
    double gain_percent;
    double speed_of_sound;
    double salinity;
  };

  // -------------------------------------------------------------------------------------------------------------------

  // The SDK just appended a '2' on the end... I added 'Version'.
  struct __attribute__((__packed__)) SimpleFireMessageVersion2
  {
    OculusMessageHeader oculus_header;
    FrequencyMode frequency_mode;
    PingRateFlag ping_rate;
    uint8_t network_speed;
    uint8_t gamma;
    OptionFlag flags;
    double range_upper_limit;
    double gain_percent;
    double speed_of_sound;
    double salinity;
    ExtraOptionFlag extra_option_flags;
    uint32_t reserved[8];
  };

  // -------------------------------------------------------------------------------------------------------------------

  struct __attribute__((__packed__)) SimplePingResult
  {
    SimpleFireMessage simple_fire_message;
    uint32_t ping_id;
    uint32_t status;
    double frequency;
    double temperature;
    double pressure;
    double speed_of_sound_used;
    uint32_t ping_start_time;
    DataSize data_size;
    double range_resolution;
    uint16_t range_count;
    uint16_t beam_count;
    uint32_t image_start_offset;
    uint32_t image_size;
    uint32_t message_size;
  };

  // -------------------------------------------------------------------------------------------------------------------

  // The SDK just appended a '2' on the end... I added 'Version'.
  struct __attribute__((__packed__)) SimplePingResultVersion2
  {
    SimpleFireMessageVersion2 simple_fire_message;
    uint32_t ping_id;
    uint32_t status;
    double frequency;
    double temperature;
    double pressure;
    double heading;
    double pitch;
    double roll;
    double speed_of_sound_used;
    double ping_start_time;
    DataSize data_size;
    double range_resolution;
    uint16_t range_count;
    uint16_t beam_count;
    uint32_t spare_0;
    uint32_t spare_1;
    uint32_t spare_2;
    uint32_t spare_3;
    uint32_t image_start_offset;
    uint32_t image_size;
    uint32_t message_size;
  };

  // -------------------------------------------------------------------------------------------------------------------

  struct __attribute__((__packed__)) OculusVersionInfo
  {
    // The arm0 firmware version major (8 bits), minor (8 bits), build (16 bits).
    uint32_t firmware_version_0;

    // The arm0 firmware date.
    uint32_t firmware_date_0;

    // The arm1 firmware version major (8 bits), minor (8 bits), build (16 bits).
    uint32_t firmware_version_1;

    // The arm1 firmware date.
    uint32_t firmware_date_1;

    // The bitfile version.
    uint32_t firmware_version_2;

    // The bitfile date.
    uint32_t firmware_date_2;
  };

  // -------------------------------------------------------------------------------------------------------------------

  struct __attribute__((__packed__)) OculusStatusMessage
  {
    OculusMessageHeader oculus_header;
    uint32_t device_id;
    OculusDevice deviceType;
    OculusPartNumber part_number;
    uint32_t status;
    OculusVersionInfo versionInfo;
    uint32_t ip_address;
    uint32_t ip_mask;
    uint32_t connected_ip_address;
    uint8_t mac_address_0;
    uint8_t mac_address_1;
    uint8_t mac_address_2;
    uint8_t mac_address_3;
    uint8_t mac_address_4;
    uint8_t mac_address_5;
    double temperature_0;
    double temperature_1;
    double temperature_2;
    double temperature_3;
    double temperature_4;
    double temperature_5;
    double temperature_6;
    double temperature_7;
    double pressure;
  };

  // -------------------------------------------------------------------------------------------------------------------

  struct __attribute__((__packed__)) OculusInfo
  {
    OculusPartNumber part_number;
    bool has_low_frequency_mode;
    double maximum_low_frequency_range;
    bool has_high_frequency_mode;
    double maximum_high_frequency_range;
    char * model;
  };


  // -------------------------------------------------------------------------------------------------------------------
  // Constant expressions.
  // NOTE: This is what you use instead of #define for constants, for various reasons.
  // -------------------------------------------------------------------------------------------------------------------

  static constexpr double kBearingResolutionDegrees = 0.01;
  static constexpr double kDegreesToRadians = M_PI / 180.0;
  static constexpr double kDeviceCalculatesSpeedOfSound = 0.0;
  static constexpr std::pair<FrequencyMode, FrequencyMode> kFrequencyModeLimits = std::make_pair(FrequencyMode::kLow,
                                                                                                 FrequencyMode::kHigh);
  static constexpr std::pair<double, double> kGainLimits = std::make_pair(0.0, 100.0);
  static constexpr char kIpToUseMacAddressMatching[] = "0.0.0.0";
  static constexpr uint8_t kMaximumNetworkSpeed = 0xff;
  static constexpr uint16_t kOculusDataPort = 52100;
  static constexpr uint16_t kOculusId = 0x4f53;
  static constexpr uint16_t kOculusStatusPort = 52102;
  static constexpr std::pair<PingRateHz, PingRateHz> kPingRateHzLimits = std::make_pair(PingRateHz::kStandby,
                                                                                        PingRateHz::kHighest);
  static constexpr std::pair<double, double> kRangeLimitsAtHighFrequency = std::make_pair(0.5, 10.0);
  static constexpr std::pair<double, double> kRangeLimitsAtLowFrequency = std::make_pair(0.5, 30.0);
  static constexpr double kSalinityOfSaltWaterPPT = 35.0;
  static constexpr float kSocketTimeoutSecs = 1.0f;

  // Arbitrary sanity checking of speed of sound values taken from data based around salinity:
  // https://www.ndt.net/article/v11n06/khan/khan.htm.
  static constexpr std::pair<double, double> kArbitrarySpeedOfSoundLimits = std::make_pair(1300.0, 1800.0);

 public:

  // -------------------------------------------------------------------------------------------------------------------
  // NON-PACKED Structures.
  // -------------------------------------------------------------------------------------------------------------------

  struct DataPacket
  {
    /** @brief A monotonic time_point representing when the message data was first identified in the queue. */
    std::chrono::time_point<std::chrono::steady_clock> message_stamp;

    /** @brief The raw metadata packet (e.g., complete message header) that accompanies each scan. */
    SimplePingResultVersion2 simple_ping_result;

    /** @brief A vector of doubles representing azimuths with 0 degrees straight ahead, < 0 degrees to the left,
     * and > 0 degrees to the right, relative to the sensor orientation. */
    std::vector<double> azimuths;

    /** @brief A vector (azimuths) storing vectors of data (ranges) in 8-bit format. */
    std::vector<std::vector<uint8_t>> unpacked_8_bit_data{};

    /** @brief A vector (azimuths) storing vectors of data (ranges) in 16-bit format. */
    std::vector<std::vector<uint16_t>> unpacked_16_bit_data{};

    /** @brief A vector (azimuths) storing vectors of data (ranges) in 32-bit format. */
    std::vector<std::vector<uint32_t>> unpacked_32_bit_data{};
  };


 private:

  FrequencyMode frequency_mode_;
  std::string frequency_mode_string_;
  PingRateFlag ping_rate_flag_;
  PingRateHz ping_rate_hz_;

  struct sockaddr_in status_sockaddr_in_{};

  uint8_t network_speed_{};
  uint8_t gamma_{};
  bool use_16_bit_data_mode_{};
  bool use_32_bit_data_mode_{};
  uint8_t use_gain_assist_{};
  double range_upper_limit_{};
  double range_resolution_{};
  double gain_percent_{};
  uint8_t data_depth_{};
  uint16_t width_{};
  uint16_t height_{};
  double salinity_{};
  double speed_of_sound_{};

  uint16_t oculus_id_{};
  uint16_t source_device_id_{};
  uint16_t destination_device_id_{};

  int32_t data_socket_{};
  int32_t status_socket_{};

  bool reduce_power_{false};
  bool show_gain_{false};
  bool simple_return_message_{true};
  bool use_metres_{true};

  std::future<bool> initialisation_future_;

  std::mutex parameter_change_mutex_{};

  std::mutex publisher_queue_updater_mutex_{};
  std::condition_variable publisher_queue_updater_condition_variable_{};

  std::atomic<bool> scanning_enabled_{false};
  std::mutex scanning_enabled_mutex_{};
  std::condition_variable scanning_enabled_condition_variable_{};

  std::deque<uint8_t> raw_data_queue_{};
  std::mutex raw_data_queue_mutex_{};
  std::condition_variable raw_data_queue_condition_variable_{};

  std::queue<std::shared_ptr<OculusDriver::DataPacket>> data_packet_queue_{};
  std::mutex data_packet_queue_mutex_{};
  std::condition_variable data_packet_queue_condition_variable_{};

  std::string oculus_ip_address_{};
  std::string oculus_mac_address_{};

  std::atomic<bool> happy_{false};
  std::atomic<bool> keep_firing_assholes_{false};
  std::atomic<bool> parameters_have_changed_{false};
  std::atomic<bool> oculus_identified_{false};
  std::atomic<uint32_t> number_of_complete_messages_parsed_{0};

  bool asyncInitialise();

  bool readParameters();
  bool validateParameters();
  int32_t createAndConnectTcpSocket();
  int32_t createAndBindUdpSocket(uint16_t target_port);
  OculusDriver::SimpleFireMessage generateSimpleFireMessage();
  OculusDriver::SimpleFireMessageVersion2 generateSimpleFireMessageVersion2();
  void handleSimplePingResult();
  void handleUserConfig();
  void propagateShutdown();
  bool sendSimpleFireMessage(OculusDriver::SimpleFireMessage& message);
  bool sendSimpleFireMessageVersion2(OculusDriver::SimpleFireMessageVersion2& message);
  bool socketHasDataIncoming(int32_t socket_file_descriptor, float timeout_duration);

  bool floatsAreEqual(double first, double second);
  bool floatsAreEqual(double first, float second);
  bool floatsAreEqual(float first, double second);
  bool floatsAreEqual(float first, float second);

  void guardedUpdateToParametersChangedFlag();


  // --------------------------------------------------------------------------------
  // Threads and their functions.
  // --------------------------------------------------------------------------------

  std::thread data_socket_retriever_thread_;
  void dataSocketRetrieverThreadFunction();

  std::thread message_finder_and_unpacker_thread_;
  void messageFinderAndUnpackerThreadFunction();

  std::thread simple_fire_message_sender_thread_;
  void simpleFireMessageSenderThreadFunction();

  std::thread status_socket_retriever_thread_;
  void statusSocketRetrieverThreadFunction();

  std::thread data_packet_queue_cleaner_thread_;
  void processingPacketQueueCleanerThreadFunction();

  double deviceBearingToAzimuth(uint16_t bearing);
  std::string frequencyModeFlagToString(OculusDriver::FrequencyMode frequency_mode);
  uint8_t pingRateFlagToHz(OculusDriver::PingRateFlag ping_rate_flag);


  // -------------------------------------------------------------------------------------------------------------------
  // Templated functions.
  // -------------------------------------------------------------------------------------------------------------------

  template <typename OutputType>
  std::vector<OutputType> deserialiseRawDataVector(
    const std::vector<uint8_t>& input,
    const uint32_t offset_from_start,
    const uint64_t number_of_output_elements)
  {
    std::vector<OutputType> output(number_of_output_elements);

    std::memcpy(
      output.data(),
      &(*(input.begin() + offset_from_start)),
      number_of_output_elements * sizeof(OutputType));

    return output;
  }

  template <typename OutputType>
  std::vector<OutputType> deserialiseRawDataVector(
    const std::vector<uint8_t>& input)
  {
    std::vector<OutputType> output((input.size() * sizeof(uint8_t)) / sizeof(OutputType));

    std::memcpy(
      output.data(),
      input.data(),
      input.size());

    return output;
  }


  // -------------------------------------------------------------------------------------------------------------------
  // Enumerator operators.
  // -------------------------------------------------------------------------------------------------------------------
  // One of the downsides to using strongly-typed enumerators is that you cannot do certain (most) operations with them.
  // Small price to pay for type security.
  // Define operators as required.
  // While these _could_ be templated, you are better off doing it one-by-one in most cases.
  // -------------------------------------------------------------------------------------------------------------------

  friend std::ostream& operator << (std::ostream& output_stream, const DataSize& entry)
  {
    using T = std::underlying_type_t<DataSize>;
    output_stream << static_cast<T>(entry);
    return output_stream;
  }


  friend std::ostream& operator << (std::ostream& output_stream, const ExtraOptionFlag& entry)
  {
    using T = std::underlying_type_t<ExtraOptionFlag>;
    output_stream << static_cast<T>(entry);
    return output_stream;
  }


  friend std::ostream& operator << (std::ostream& output_stream, const FrequencyMode& entry)
  {
    using T = std::underlying_type_t<FrequencyMode>;
    output_stream << static_cast<T>(entry);
    return output_stream;
  }


  friend std::ostream& operator << (std::ostream& output_stream, const MessageId& entry)
  {
    using T = std::underlying_type_t<MessageId>;
    output_stream << static_cast<T>(entry);
    return output_stream;
  }


  friend std::ostream& operator << (std::ostream& output_stream, const MessageVersion& entry)
  {
    using T = std::underlying_type_t<MessageVersion>;
    output_stream << static_cast<T>(entry);
    return output_stream;
  }


  friend std::ostream& operator << (std::ostream& output_stream, const OptionFlag& entry)
  {
    using T = std::underlying_type_t<OptionFlag>;
    output_stream << static_cast<T>(entry);
    return output_stream;
  }


  friend std::ostream& operator << (std::ostream& output_stream, const PingRateFlag& entry)
  {
    using T = std::underlying_type_t<PingRateFlag>;
    output_stream << static_cast<T>(entry);
    return output_stream;
  }


  friend OptionFlag operator | (OptionFlag lhs, OptionFlag rhs)
  {
    using T = std::underlying_type_t<OptionFlag>;
    return static_cast<OptionFlag>(static_cast<T>(lhs) | static_cast<T>(rhs));
  }


  friend OptionFlag operator |= (OptionFlag &lhs, OptionFlag rhs)
  {
    lhs = lhs | rhs;
    return lhs;
  }


  friend ExtraOptionFlag operator | (ExtraOptionFlag lhs, ExtraOptionFlag rhs)
  {
    using T = std::underlying_type_t<ExtraOptionFlag>;
    return static_cast<ExtraOptionFlag>(static_cast<T>(lhs) | static_cast<T>(rhs));
  }


  friend ExtraOptionFlag operator |= (ExtraOptionFlag &lhs, ExtraOptionFlag rhs)
  {
    lhs = lhs | rhs;
    return lhs;
  }

 public:

  OculusDriver(std::string logging_severity);
  virtual ~OculusDriver();

  void beginInitialisation();
  std::shared_ptr<OculusDriver::DataPacket> getLatestDataPacket();
  bool isHealthy();
  bool isInitialised();
  bool isStandingBy();
  void loadDefaultParameters();
  std::string outputLatestSimplePingResult();
  void run();
  void shutdown();
  void stop();

  uint32_t calculateMicrosSinceTimePoint(const std::chrono::time_point<std::chrono::steady_clock> start_point);
  uint32_t calculateMicrosBetweenTimePoints(
    const std::chrono::time_point<std::chrono::steady_clock> start_point,
    const std::chrono::time_point<std::chrono::steady_clock> end_point);

  uint8_t getDataDepth();
  double getGain();
  uint8_t getGamma();
  uint8_t getFrequencyModeFlag();
  std::string getFrequencyModeString();
  std::string getOculusIpAddress();
  std::string getOculusMacAddress();
  uint8_t getPingRateFlag();
  uint8_t getPingRateHz();
  double getRangeUpperLimit();

  void setDataDepth(uint8_t requested_value);
  void setGain(double requested_value);
  void setGamma(uint8_t requested_value);
  void setFrameId(std::string requested_value);
  void setFrequencyMode(std::string requested_value);
  void setOculusIpAddress(std::string requested_value);
  void setOculusMacAddress(std::string requested_value);
  void setPingRate(uint8_t requested_value);
  void setRangeUpperLimit(double requested_value);
  void setSalinityPPT(double requested_value);
  void setSpeedOfSound(double requested_value);
};

#endif // End BLUEPRINT_SUBSEA_OCULUS_DRIVER_OCULUS_DRIVER_HPP guard.
