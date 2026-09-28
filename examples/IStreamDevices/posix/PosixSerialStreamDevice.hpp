#pragma once

#include "IStreamDevice.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <dirent.h>
#include <string.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>

class PosixSerialStreamDevice : public IStreamDevice {
private:
    int fd = -1;
    uint32_t current_baud = 1;

    void onTxComplete() override {
        if (txCompleteCallback) txCompleteCallback();
    }

    void onRxComplete(uint16_t size) override {
        // Handled via callback in the read() method
    }

    static speed_t mapBaudrate(uint32_t baud) {
        switch (baud) {
            case 9600:   return B9600;
            case 19200:  return B19200;
            case 38400:  return B38400;
            case 57600:  return B57600;
            case 115200: return B115200;
            default:     return B0;
        }
    }

public:
    PosixSerialStreamDevice() = default;

    virtual ~PosixSerialStreamDevice() {
        close_port();
    }

    bool open_port(const std::string& port_name) {
        fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) return false;

        // Return to blocking mode for the interface requirements
        fcntl(fd, F_SETFL, 0);
        return true;
    }

    void close_port() {
        if (fd >= 0) {
            close(fd);
            fd = -1;
        }
    }

    // --- IStreamDevice Implementation ---

    void baudrate(uint32_t baud) override {
        if (fd < 0) return;
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) return;

        speed_t s = mapBaudrate(baud);
        if (s == B0) return;

        cfsetospeed(&tty, s);
        cfsetispeed(&tty, s);

        // 8N1 Raw Mode
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_oflag &= ~OPOST;

        tcsetattr(fd, TCSANOW, &tty);
        current_baud = baud;
    }

    uint32_t baudrate() const override {
        return current_baud;
    }

    SerialError read(std::span<uint8_t> buffer, uint32_t timeout_ms, size_t* bytes_read_out = nullptr) override {
        if (fd < 0) return SerialError::INTERNAL_ERROR;

        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLIN;

        int ret = poll(&pfd, 1, timeout_ms);
        if (ret == 0) return SerialError::TIMEOUT;
        if (ret < 0) return SerialError::INTERNAL_ERROR;
		ssize_t total_size = 0;
    	ssize_t n = 0;
    	do {
    		auto target_buffer = buffer.subspan(total_size);
    		n = ::read(fd, target_buffer.data(), target_buffer.size());
    		total_size += n;
    		if (n < 0) return SerialError::INTERNAL_ERROR;
    	}while (n);

        if (bytes_read_out) *bytes_read_out = static_cast<size_t>(total_size);

        if (rxCompleteCallback && total_size > 0) {
            rxCompleteCallback(buffer.subspan(0, total_size));
        }

        return SerialError::SUCCESS;
    }

    SerialError write(std::span<const uint8_t> buffer, uint32_t timeout_ms, size_t* bytes_written_out = nullptr) override {
        if (fd < 0) return SerialError::INTERNAL_ERROR;

        ssize_t n = ::write(fd, buffer.data(), buffer.size());
        if (n < 0) return SerialError::INTERNAL_ERROR;

        if (bytes_written_out) *bytes_written_out = static_cast<size_t>(n);

        onTxComplete();
        return SerialError::SUCCESS;
    }

    SerialError flush() override {
        if (fd < 0) return SerialError::INTERNAL_ERROR;
        return (tcflush(fd, TCIOFLUSH) == 0) ? SerialError::SUCCESS : SerialError::INTERNAL_ERROR;
    }

    /**
     * Static helper to find ports in Cygwin environment
     */
    static std::vector<std::string> ListSerialPorts() {
        std::vector<std::string> ports;
        DIR* dir = opendir("/dev");
        if (dir == nullptr) return ports;

        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string filename = entry->d_name;
            if (filename.compare(0, 4, "ttyS") == 0) {
                ports.push_back("/dev/" + filename);
            }
        }
        closedir(dir);

        std::sort(ports.begin(), ports.end(), [](const std::string& a, const std::string& b) {
            try {
                // substr(9) to skip "/dev/ttyS"
                return std::stoi(a.substr(9)) < std::stoi(b.substr(9));
            } catch (...) {
                return a < b;
            }
        });

        return ports;
    }
};
