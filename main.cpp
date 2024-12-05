#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <ctime>
#include <cstdint>
#include <cstring>
#include <cmath> // for std::floor

struct GPSData {
    uint64_t time_usec;
    uint8_t fix_type;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    bool location_valid;  // is location data valid?
};

uint64_t time_sec;
uint64_t start_time_sec;
bool is_get_gga = false;
bool debug_mode = false;

char* port_name = (char*)"/dev/ttyUSB0";
int baudrate = B38400;
int input_baudrate = 38400;

// Validate the checksum of the NMEA data
bool verifyChecksum(const std::string& sentence) {
    size_t starPos = sentence.find('*');
    if (starPos == std::string::npos) return false;

    unsigned char checksum = 0;
    for (size_t i = 1; i < starPos; ++i) {
        checksum ^= sentence[i];
    }

    unsigned int providedChecksum;
    std::stringstream ss;
    ss << std::hex << sentence.substr(starPos + 1);
    ss >> providedChecksum;

    return checksum == providedChecksum;
}

// 緯度・経度を10進形式に変換する関数
double convertToDecimal(const std::string& value, const std::string& direction) {
    if (value.empty() || direction.empty()) {
        return 0.0;
    }

    try {
        double raw = std::stod(value);
        double degrees = std::floor(raw / 100);              // 度
        double minutes = raw - (degrees * 100);             // 分
        double decimal = degrees + (minutes / 60);          // 10進形式

        // 南緯または西経の場合は負の値にする
        if (direction == "S" || direction == "W") {
            decimal *= -1;
        }
        return decimal;
    } catch (const std::exception& e) {
        std::cerr << "変換エラー: " << e.what() << std::endl;
        return 0.0;
    }
}


// NMEAデータを解析する関数
GPSData parseNMEA(const std::string& sentence) {
    GPSData gps_data = {};
    gps_data.time_usec = std::time(nullptr) * 1e6;
    gps_data.location_valid = false;
    is_get_gga = false;

    if (sentence.rfind("$GNGGA", 0) == 0) {
        is_get_gga = true;
        // メッセージフィールドをカンマで分割
        std::istringstream stream(sentence);
        std::string token;
        std::vector<std::string> fields;
        while (std::getline(stream, token, ',')) {
            fields.push_back(token);
        }
        // 位置情報が確定したかどうか   
        if (fields.size() >= 2 && fields[6] != "0") {
            gps_data.location_valid = true;
        }
        // 必要なデータを取得
        if (fields.size() >= 6 && !fields[2].empty() && !fields[4].empty()) {
                gps_data.time_usec = std::stod(fields[1]) * 1e6; // UTC時刻
                gps_data.fix_type = fields[6] == "0" ? 0 : (fields[6] == "1" ? 2 : 3); // 位置情報の種類
                gps_data.lat = convertToDecimal(fields[2], fields[3]) * 1e7; // 緯度
                gps_data.lon = convertToDecimal(fields[4], fields[5])  * 1e7; // 経度  
                gps_data.alt = static_cast<int32_t>(std::stod(fields[9]) * 1000); // 高度
        }
    }

    return gps_data;
}

// GPSデータを表示する関数
void printGPSData(const GPSData& data) {
    time_sec = std::time(nullptr) - start_time_sec;

    // GGAメッセージが取得できたかどうか
    if(is_get_gga){
        // 位置情報が有効でない場合は表示しない
        if (!data.location_valid) {
            std::cout << "elapsed time (s): " << time_sec << "\n"
                    << "[Waiting for location data...]\n"
                    << "--------------------------\n";
            return;
        }
        
        std::cout << "elapsed time (s): " << time_sec << "\n"
                << "[Get location data]\n"
                <<"UTC Time(usec): " << data.time_usec << "\n"
                << "Fix type: " << static_cast<int>(data.fix_type) << "\n"
                << "Latitude: " << data.lat << "\n"
                << "Longitude: " << data.lon << "\n"
                << "Altitude: " << data.alt << "\n"
                << "--------------------------\n";
    }

}

// シリアルポートを設定する関数
int setupSerialPort(const char* port_name, int baudrate) {
    int fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Failed to open serial port: " << port_name << "\n";
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    // 設定変更
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

// シリアル通信のデータ受信と処理
void readSerialData(int fd) {
    char buffer[1024];
    std::string dataBuffer;

    while (true) {
        int bytes_read = read(fd, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            dataBuffer += buffer;

            size_t pos;
            while ((pos = dataBuffer.find('\n')) != std::string::npos) {
                std::string sentence = dataBuffer.substr(0, pos);
                dataBuffer.erase(0, pos + 1);

                if (sentence[0] == '$') {
                    if (verifyChecksum(sentence)) {
                        GPSData gps_data = parseNMEA(sentence);
                        printGPSData(gps_data);
                        if(debug_mode){std::cout << "Valid NMEA Sentence: " << sentence << "\n";}
                    } else {
                        // std::cout << "Invalid or incomplete sentence: " << sentence << "\n";
                    }
                } else {
                    // std::cout << "Ignored non-NMEA data: " << sentence << "\n";
                }
            }
        }
        usleep(10000); // wait for a while to get the next data
    }
}

void parse_command_line(int argc, char* argv[]) {
    const char* commandline_usage = "Usage: serial_gps_reader -d <device> -b <baudrate>";
    for (int i = 1; i < argc; i++) { 
		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				i++;
				port_name = argv[i];
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				i++;
                input_baudrate = atoi(argv[i]);
                switch (input_baudrate)
                {
                case 9600:
                    baudrate = B9600;
                    break;
                case 19200:
                    baudrate = B19200;
                    break;
                case 38400:
                    baudrate = B38400;
                    break;
                case 57600:
                    baudrate = B57600;
                    break;
                case 115200:
                    baudrate = B115200;
                    break;
                default:
                    std::cerr << "Invalid baudrate: " << input_baudrate << "\n";
                    throw EXIT_FAILURE;
                    break;
                }

				baudrate = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

        //debug mode
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
            std::cout << "Debug mode\n";
            debug_mode = true;
        }
    }
}

int main(int argc, char* argv[]) {
    parse_command_line(argc, argv);
    
    start_time_sec = std::time(nullptr);

    int fd = setupSerialPort(port_name, baudrate);
    if (fd == -1) {
        return -1;
    }

    std::cout << "Serial port opened: " << port_name << "\n";
    std::cout << "Baudrate: " << baudrate << "\n";

    // GPSデータの受信処理を開始
    std::cout << "Start reading GPS data...\n";
    std::cout << "--------------------------\n";
    readSerialData(fd);

    close(fd);
    return 0;
}
