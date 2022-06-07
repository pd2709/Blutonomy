#include <iostream>
#include <fstream>
#include <cmath> // std::abs
#include <sys/stat.h> // mkdir
#include <chrono> // chrono::system_clock
#include <ctime>  // localtime
#include <sstream> // stringstream
#include <iomanip> // put_time

#include "json.hpp"
#include "blueprint_subsea_oculus_driver/OculusDriver.hpp"

using json = nlohmann::json;

// This class creates a interface for the oculus, to be used in conjunction with the driver developed by Flinders University.
// The interface has 3 main functionlities
//      1. To be able to update oculus settings through a json file
//      2. To be able to save sonar/image data and sonar settings into binary file(s)
//      3. To be able to save (only) sonar/image data to a .csv file
//
// This class has the following methods
//      oculusInterface(std::string& inputFilename)
//          Constructor
//      void checkAndUpdateSonarConfig(OculusDriver& oculus)
//          Update the oculus sonar setting according to parameters specified in the json file "config.json"
//      void populateMsgHeader(std::shared_ptr<OculusDriver::DataPacket> dataPacket)
//          Populate the message header that will be written to the start of each binary file
//      int writeToBinary_16bit(std::shared_ptr<OculusDriver::DataPacket> dataPacket )
//          Output a binary file with the header and 16 bit sonar data.
//          The file will sit under the folder named by the convention "/binData/YYYYMMDD_HHhMMmSSs/"
//      int writeToCsv_16bit(std::vector<std::vector<uint16_t>>& dataMatrix)
//          Writes the 16 bit data into a .csv file under /csvData
//
class oculusInterface
{
    private:
        std::string jsonFilename;
        std::string folderName;
        json jsonConfigFile;
        
        int csvCount = 0;
        int binaryCount = 0;
        
        // Header for each message
        struct MsgHeader
        {
            uint32_t fileVersion = 3; // When reading, need to skip 32 bits after this due to padding
            double speedOfSoundUsed;
            double frequency;
            double temperature;
            double pressure;
            double heading;
            double pitch;
            double roll;
            double pingStartTime;
            double range;
            double gain;
            double rangeResolution;
            // int msgHeaderSize; // not needed for the current binary file structure
            uint64_t msSinceEpoch;
            uint32_t pingId;
            uint32_t dataSize;
            uint16_t rangeCount;
            uint16_t beamCount;
            uint8_t pingRate;  // Does not change in the msgs, taken out for now
            uint8_t freqMode;
        };

        MsgHeader msgHeader; // create an instance of the header        

    public:
        // constructor
        oculusInterface(std::string& inputFilename)
        {
            this->jsonFilename = inputFilename;
            std::ifstream ifs(this->jsonFilename);
            this->jsonConfigFile = json::parse(ifs);

            // Create the folder for binary file
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);

            std::stringstream ss;
            ss<< std::put_time(std::localtime(&in_time_t), "%Y%m%d_%Hh%Mm%S");

            std::cout << ss.str() << std::endl;

            this->folderName = "/media/thales/T7/UTS/TestDay-2/Thursday 28-4-2022-CAS" + ss.str();

            int dirStatus = mkdir(this->folderName.c_str(), 0777);

            if (!dirStatus)
                std::cout << "Directory Created" << std::endl;
            else
                std::cout << "Unable to Create Directory" << std::endl;
        }

        // Update oculus settings according to the json file (config.json)
        // Input[1]:
        //      OculusDriver& oculus - the oculus instance
        // output[0] void
        void checkAndUpdateSonarConfig(OculusDriver& oculus)
        {
            std::ifstream ifs(this->jsonFilename);
            this->jsonConfigFile = json::parse(ifs);

            std::string frequencyMode = jsonConfigFile.value("Frequency Mode", "low");
            double range = jsonConfigFile.value("Range Upper Limit", 5.0);
            uint8_t pingRate = uint8_t (jsonConfigFile.value("Ping Rate", 10));
            double gain = jsonConfigFile.value("Gain", 0.0);
            
            // conversion for when gain dismatch happens
            // gain = (gain-40)*5/3;

            if (frequencyMode != oculus.getFrequencyModeString()) 
                oculus.setFrequencyMode(frequencyMode);
            if ( std::abs(range - oculus.getRangeUpperLimit()) > 0.1)
                oculus.setRangeUpperLimit(range);
            if ( std::abs(pingRate - oculus.getPingRateHz()) > 0.1)
                oculus.setPingRate(pingRate);
            if ( std::abs(gain - oculus.getGain()) > 0.1 ) 
                oculus.setGain(gain);

            return;
        }

        // Pululates the message header
        // Input[1]:
        //      std::shared_ptr<OculusDriver::DataPacket> dataPacket - data packet that contains header
        //                                                              and sonar data
        // output[0] void
        void populateMsgHeader(std::shared_ptr<OculusDriver::DataPacket> dataPacket)
        {
            // std::cout << "within populate Msg header: Azimuth Size: "<< dataPacket->azimuths.size() << std::endl;
            // this->msgHeader.fileVersion = 1; // Version 1 of file
            
            // variables that were in overall header - does not change from scan to scan
            this->msgHeader.speedOfSoundUsed = dataPacket->simple_ping_result.speed_of_sound_used;
            // this->msgHeader.msgHeaderSize = sizeof(msgHeader);

            // parameters that could change from scan to scan
            this->msgHeader.pingId = dataPacket->simple_ping_result.ping_id;
            this->msgHeader.frequency = dataPacket->simple_ping_result.frequency;
            this->msgHeader.temperature = dataPacket->simple_ping_result.temperature;
            this->msgHeader.pressure = dataPacket->simple_ping_result.pressure;
            this->msgHeader.heading = dataPacket->simple_ping_result.heading;
            this->msgHeader.pitch = dataPacket->simple_ping_result.pitch;
            this->msgHeader.roll = dataPacket->simple_ping_result.roll;
            this->msgHeader.pingStartTime = dataPacket->simple_ping_result.ping_start_time;
            this->msgHeader.rangeCount = dataPacket->simple_ping_result.range_count;
            this->msgHeader.beamCount = dataPacket->simple_ping_result.beam_count;
            this->msgHeader.dataSize = dataPacket->simple_ping_result.image_size;
            this->msgHeader.rangeResolution = dataPacket->simple_ping_result.range_resolution;
            this->msgHeader.pingRate = static_cast<uint8_t>(dataPacket->simple_ping_result.simple_fire_message.ping_rate);
            this->msgHeader.msSinceEpoch = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>
                                                    (std::chrono::system_clock::now().time_since_epoch()).count());
//
//                                                    std::cout << " ------------------------------- MS SINCE EPOCH " << this->msgHeader.msSinceEpoch  << std:: endl;
            // Sonar Settings
            this->msgHeader.range = dataPacket->simple_ping_result.simple_fire_message.range_upper_limit;
            this->msgHeader.gain = dataPacket->simple_ping_result.simple_fire_message.gain_percent;
            this->msgHeader.freqMode = static_cast<uint8_t>(dataPacket->simple_ping_result.simple_fire_message.frequency_mode);
            // std::cout 
            // << "speedOfSoundUsed: " << this->msgHeader.speedOfSoundUsed << "\n"
            // << "frequency: " << std::dec << this->msgHeader.frequency << "\n"
            // << "temperature: " << std::dec << this->msgHeader.temperature << "\n"
            // << "pressure: " << std::dec << this->msgHeader.pressure << "\n"
            // << "heading: " << std::dec << this->msgHeader.heading << "\n"
            // << "pitch: " << std::dec << this->msgHeader.pitch << "\n"
            // << "roll: " << std::dec << this->msgHeader.roll << "\n"
            // << "pintStartTime: " << std::dec << this->msgHeader.pintStartTime << "\n"
            // //  << "msgHeaderSize: " << this->msgHeader.msgHeaderSize << "\n"
            //  << "pingId: " << std::dec << this->msgHeader.pingId << "\n"
            //  << "dataSize: " << this->msgHeader.dataSize << "\n"
            //  << "rangeCount: " << std::dec << this->msgHeader.rangeCount << "\n"
            //  << "beamCount: " << std::dec << this->msgHeader.beamCount << std::endl;

            // std::cout << "Header populated" << std::endl;

            return;
        }

        // Writes the header and 16 bit data into a binary file
        // Input[1]:
        //      std::shared_ptr<OculusDriver::DataPacket> dataPacket - data packet that contains header
        //                                                              and sonar data
        // output[1]:
        //      int status - 0 for successfully outputed data into binary file,
        //                   1 for errored during the process
        int writeToBinary_16bit(std::shared_ptr<OculusDriver::DataPacket> dataPacket )
        {
            // populate header
            this->populateMsgHeader(dataPacket);

            // Make the 2D vector into a 1D vector
            std::vector<uint16_t> data_vector {};
            for(auto && v : dataPacket->unpacked_16_bit_data)
            {
                data_vector.insert(data_vector.end(), v.begin(), v.end());
            }

            // Filename
            std::string numString = std::to_string( this->binaryCount );
            long unsigned int noZero = 5-numString.length();
            numString.insert(0, noZero, '0');
            std::string binFilename = this->folderName + "/" + numString +".dat";

            // output to file
            std::ofstream writeToBinaryFile(binFilename, std::ios::out | std::ios::binary);
            if(!writeToBinaryFile) 
            {
                std::cout << "can't open file to be written" << std::endl;
                return 1;
            }    

            writeToBinaryFile.write(reinterpret_cast<char*>(&msgHeader), sizeof(this->msgHeader));
            writeToBinaryFile.write(reinterpret_cast<const char*>(&data_vector[0]),
                                 static_cast<long int>(data_vector.size()*sizeof(uint16_t)));

            writeToBinaryFile.close();
            if(!writeToBinaryFile.good()) 
            {
                std::cout << "errored when writing file" << std::endl;
                return 1;
            }
            
            std::cout << " ~~~~~~~~~~~~ outputed to binary file ~~~~~~~~~~~~ " << std::endl;

            this->binaryCount++;
            return 0;
        }


        // TODO - see if there are ways to save 8 bit and 32 bit data using the same function
        // Saves 16 bit sonar data to .csv file
        // Input[1]:
        //      std::vector<std::vector<uint16_t>>& dataMatrix - vector of vector of 16 bit sonar data
        // output[1]:
        //      int status - 0 for successfully outputed data into binary file,
        //                   1 for errored during the process
        int writeToCsv_16bit(std::vector<std::vector<uint16_t>>& dataMatrix)
        {
            std::string numString = std::to_string( this->csvCount );
            long unsigned int noZero = 5-numString.length();
            numString.insert(0, noZero, '0');
            std::string csvFilename = "csvData/data" + numString +".csv";

            std::ofstream csvFile(csvFilename);

            if(!csvFile) 
            {
                std::cout << "can't open file to be read" << std::endl;
                return 1;
            }

            // csvFile << "data"; 
            for (uint32_t i = 0; i < dataMatrix.size(); i++)
            {
                csvFile << dataMatrix[i][0];
                for (uint32_t j = 1; j < dataMatrix[i].size(); j++)
                    csvFile << "," << dataMatrix[i][j];
                csvFile << "\n";
            }

            csvFile.close();

            if(!csvFile.good()) 
            {
                std::cout << "errored when writing file" << std::endl;
                return 1;
            }
            
            std::cout << " ~~~~~~~~~~~~ Wrote csv data ~~~~~~~~~~~~ " << std::endl;
            this->csvCount++;
            
            return 0;
        }


};
