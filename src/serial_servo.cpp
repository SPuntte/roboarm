#include <boost/asio.hpp>

#include <cstdint>
#include <iostream>

int main()
{
    auto device = "/dev/ttyACM0";
    boost::asio::io_service io;
    boost::asio::serial_port serial(io, device);
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));

    std::cout << "Servo control over serial port (" << device << ")." << std::endl;

    while (true)
    {
        uint16_t idx = -1;
        uint16_t us = -1;
        while (idx < 0 || idx > 15)
        {
            std::cout << "Servo index (0-14, 15 to wildcard all): ";
            std::cin >> idx;
        }
        std::cout << "Delay: ";
        std::cin >> us;

        us &= 0x0FFF;
        us |= ((idx & 0xF) << 12);

        std::cout << "Sending: index = " << (us >> 12) << ", delay = " << (us & 0x0FFF) << std::endl;

        boost::asio::write(serial, boost::asio::buffer(&us, 2));
    }
    return 0;
}
