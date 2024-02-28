#include "GPSdriver.hpp"


GPSdriver::GPSdriver() : CONFIGURATIONS("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"), gps_uart(uart0) {
    uint chosenBAUDRATE = uart_init(gps_uart, GPS_BAUDRATE);

    uart_set_translate_crlf(gps_uart, false);

    // Enable the uart functionality for the pins connected to the GPS
    gpio_set_function(GPS_TX, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX, GPIO_FUNC_UART);

    // mode rmc only
    send_with_checksum(gps_uart, CONFIGURATIONS, sizeof(char) * configuration_len);
    // update rate 5 times in 1 second
    send_with_checksum(gps_uart, "PMTK220,200", sizeof(char) * 12);

    sensor_valid = true;
}

size_t GPSdriver::uart_read_line(uart_inst_t *uart, char *buffer, const size_t max_length) {
    size_t i;
    // Receive the bytes with as much delay as possible without missing data
    buffer[0] = uart_getc(uart);
    for(i = 1;i < max_length - 1 && buffer[i - 1] != '\n';i++){
        sleep_us(GPS_DELAY);
        buffer[i] = uart_getc(uart);
    }

    // End the string with a terminating 0 and return the length
    buffer[i] = '\0';
    return i;
}

void GPSdriver::send_with_checksum(uart_inst_t *uart, const char *message, const size_t length) {
    char sum = 0;
    char checksum[3];

    // Calcute the checksum
    for(size_t i = 0;i < length && message[i] != '*';i++){
        sum ^= message[i];
    }

    // Convert the checksum to a hexadecimal string
    for(size_t i = 0;i < 2;i++){
        if(sum % 16 < 10){
            checksum[1 - i] = '0' + sum % 16;
        }else{
            checksum[1 - i] = 'A' + sum % 16 - 10;
        }
        sum >>= 4;
    }
    checksum[2] = '\0';

    // Send the message to the GPS in the expected format
    uart_putc_raw(uart, '$');
    uart_puts(uart, message);
    uart_putc(uart, '*');
    uart_puts(uart, checksum);
    uart_puts(uart, "\r\n");
}

void GPSdriver::read_data(GPSdata &container) {
    // Read a line from the GPS data
    const size_t length = uart_read_line(gps_uart, message, sizeof(message));

    std::string gpsMSG = std::string(message);
    if(length > 6 && message[0] == '$' && gpsMSG.find('*') != std::string::npos) {
        std::string checksum = gpsMSG.substr(gpsMSG.find('*') + 1, 2);
        std::string checksumChecker = gpsMSG.substr(1, gpsMSG.find('*') - 1);

        char sum = 0;
        char MYchecksum[2];

        // Calcute the checksum
        for(size_t i = 0;i < checksumChecker.length();i++){
            sum ^= checksumChecker[i];
        }

        // Convert the MYchecksum to a hexadecimal string
        for(size_t i = 0;i < 2;i++){
            if(sum % 16 < 10){
                MYchecksum[1 - i] = '0' + sum % 16;
            }else{
                MYchecksum[1 - i] = 'A' + sum % 16 - 10;
            }
            sum >>= 4;
        }

        std::string MYchecksumSTR{MYchecksum}; 

        if(MYchecksumSTR[0] == checksum[0] && MYchecksumSTR[1] == checksum[1]) {
            std::string gps_read(message);
            std::string delim = ","; // delimiter  
            size_t pos = 0;  
            std::string token1;
            int index = 0;

            while (( pos = gps_read.find (delim)) != std::string::npos) { 
                if(index > 13) index = 0;

                token1 = gps_read.substr(0, pos);

                switch(index) {
                    case 0:
                    case 4:
                    case 6:
                    case 8:
                    case 9:
                    case 10:
                    case 11:
                    case 12: {
                        break;
                    }

                    // return data by inserting the data to GPSdata
                    case 1: {
                        if(token1.length() < 6) break;
                        container.hr = std::stoi(token1.substr(0, 2)) + 2;
                        container.min = std::stoi(token1.substr(2).substr(0, 2));
                        container.sec = std::stof(token1.substr(3).substr(1));
                        break;
                    }

                    case 2: {
                        if(token1 == "A" || token1 == "V") {
                            container.fixState = (token1 == "A" ? 1 : 0);
                        } else {
                            container.fixState = -1;
                        }
                        
                        break;
                    }

                    case 3: {
                        if(token1 == "") break;
                        
                        float origNum = std::stof(token1);
                        float degWhole = (float) ((int) (origNum / 100));
                        float degDec = (origNum - (degWhole * 100)) / 60;
                        float deg = degWhole + degDec;
                        
                        container.N = deg;
                        break;
                    }

                    case 5: {
                        if(token1 == "") break;

                        float origNum = std::stof(token1);
                        float degWhole = (float) ((int) (origNum / 100));
                        float degDec = (origNum - (degWhole * 100)) / 60;
                        float deg = degWhole + degDec;

                        container.E = deg;
                        break;
                    }

                    case 7: {
                        if(container.fixState == 1) {
                            container.speed = std::stof(token1) * KnotsToMeterPerSec;
                        } else {
                            container.speed = 0;
                        }

                        break;
                    }
                }

                index++;
                gps_read.erase(0, pos + delim.length());  /* erase() function store the current positon and move to next token. */   
            }       
        }
    }
}