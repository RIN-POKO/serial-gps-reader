# serial-gps-reader

This project is a lightweight C++ application designed to interface with GPS devices via serial communication. It is specifically developed and tested for the Beitian BE-180 GPS module, but it may also work with other NMEA-compatible GPS devices. The program reads, validates, and parses NMEA-formatted GPS data in real-time, providing key location information such as latitude, longitude, altitude, and fix type.


## Features

- **NMEA Sentence Parsing**: Supports `$GNGGA` sentences for extracting GPS data.
- **Real-time Data Processing**: Continuously reads and processes GPS data from a serial port.
- **Checksum Validation**: Ensures data integrity by verifying NMEA sentence checksums.
- **Coordinate Conversion**: Converts GPS coordinates from degrees-minutes to decimal format.
- **Command-line Configurations**: Customize serial device and baud rate via command-line arguments.

## Requirements

- **Operating System**: Linux-based systems (tested with `/dev/ttyUSB0` devices).
- **C++ Compiler**: Supports C++11 or later.
- **Dependencies**: Standard C++ libraries (`iostream`, `fstream`, `unistd.h`, etc.).

## How to Use

1. Compile the code:
   ```bash
   g++ -o serial_gps_reader main.cpp 
   ```

2. Run the program with the desired serial device and baud rate:
   ```bash
   ./serial_gps_reader -d /dev/ttyUSB0 -b 38400
   ```

3. View the parsed GPS data in the terminal.

## Command-Line Arguments

| Argument          | Description                      | Example               |
|--------------------|----------------------------------|-----------------------|
| `-d` or `--device`| Specify the serial device        | `-d /dev/ttyUSB0`     |
| `-b` or `--baud`  | Set the baud rate for communication | `-b 38400`            |
| `-v` or `--verbose` | Enable verbose output           | `-v`                  |

## Example Output

```plaintext
$ ./serial_gps_reader -d /dev/ttyUSB0 -b 38400
Serial port opened: /dev/ttyUSB0
Baudrate: 38400
Start reading GPS data...
--------------------------
elapsed time (s): 0
[Get location data]
UTC Time(usec): 80158000000
Fix type: 2
Latitude: 351551590
Longitude: 1369662918
Altitude: 77300
--------------------------
elapsed time (s): 1
[Get location data]
UTC Time(usec): 80159000000
Fix type: 2
Latitude: 351549301
Longitude: 1369662760
Altitude: 154800
--------------------------
```

## Future Improvements

- Support additional NMEA sentence types (`$GPRMC`, `$GPGLL`, etc.).
- Implement data logging to a file.
- Add multi-threading for improved performance.

## Contributing

Contributions are welcome! Feel free to open issues or submit pull requests.

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.
