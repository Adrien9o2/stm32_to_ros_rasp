#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <list>

#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "stm32_to_ros/SerialID.hpp"

using namespace std::chrono_literals;

volatile sig_atomic_t running = 1;

static void signal_handler(int signum)
{
    (void)signum;
    running = 0;
    rclcpp::shutdown();
}

class UsbSerialListener : public rclcpp::Node
{
    public:
        UsbSerialListener()
        : Node("usb_serial_listener_node"), msg_len(0), reading(false), writing(false)
        {
        
            serial_port_.Open("/dev/ttyACM0");
            serial_port_.SetBaudRate( LibSerial::BaudRate::BAUD_115200 );
            serial_port_.FlushIOBuffers();
            publisher_ = this->create_publisher<std_msgs::msg::String>("read_msg", 10);

            timer_read = this->create_wall_timer(
                0ms, std::bind(&UsbSerialListener::reading_callback, this));
            timer_write = this->create_wall_timer(
                500ms, std::bind(&UsbSerialListener::writing_callback, this));
        }

        ~UsbSerialListener()
        {
            serial_port_.Close();
        }

        void writing_callback()
        {
            if(running)
            {
                std::cout << "entering\n";
                while(reading);
                std::cout << "begin write\n";
                writing = true;
                try
                {
                    char to_send[50] = "Hello stm";
                    uint8_t len = strlen(to_send);
                    write_data_header.push_back(SerialID::MSG_START);
                    write_data_header.push_back(SerialID::MSG_PRINT);
                    write_data_header.push_back(len);
                    write_data_payload.assign(len,0);
                    std::memcpy(write_data_payload.data(),to_send,len);
                    std::cout << "sending header\n";
                    serial_port_.Write(write_data_header);
                    write_data_header.clear();
                    serial_port_.Read(read_data,3,10);
                    if( read_data[0] == SerialID::MSG_ACK 
                        && read_data[1]== SerialID::MSG_PRINT 
                        && read_data[2] == strlen(to_send))
                   {
                         std::cout << "received_ack"<<std::endl;
                        serial_port_.Write(write_data_payload);
                        write_data_payload.clear();
                        read_data.clear();
                   } 
                   else
                  {
                    std::cout <<"bad ack"<<std::endl;
                  } 
                    


                }
                catch(const LibSerial::ReadTimeout &)
                {
                    RCLCPP_INFO(this->get_logger(),"Timeout Write");  
                    write_data_header.clear();
                    write_data_payload.clear();
                    read_data.clear();
                }
                


                writing = false;
           } 
        } 

        void reading_callback()
        {
            if(running)
            {
                while(writing);
                try 
                {
                    serial_port_.Read(read_data,3);
                    reading = true;
                    if(read_data[0] == SerialID::MSG_START)
                    {
                        switch (read_data[1])
                        {
                            case SerialID::MSG_PRINT:
                                msg_len = read_data[2];
				                write_data_header.push_back(SerialID::MSG_ACK);
                                write_data_header.push_back(SerialID::MSG_PRINT);
                                write_data_header.push_back(msg_len);
                                serial_port_.Write(write_data_header);
                                write_data_header.clear();
                                read_data.clear();
                                read_string.clear();
                                serial_port_.Read(read_string,msg_len,10);
                                RCLCPP_INFO(this->get_logger(),read_string.c_str());
                                break;
                            case SerialID::MSG_DATA_1:
                                msg_len = read_data[2];
                                write_data_header.push_back(SerialID::MSG_ACK);
                                write_data_header.push_back(SerialID::MSG_DATA_1);
                                write_data_header.push_back(msg_len);
                                serial_port_.Write(write_data_header);
                                write_data_header.clear();
                                read_data.clear();
                                serial_port_.Read(read_data,msg_len,10);
                                float readfloat;
                                std::memcpy(&readfloat, read_data.data(), sizeof(float));
                                display_string+=std::to_string(readfloat);
                                RCLCPP_INFO(this->get_logger(),display_string.c_str());
                                display_string.clear();
                                break;
                            
                            default:
                                break;
                        }
                    }

                }
                catch (const LibSerial::ReadTimeout &)
                {
                    RCLCPP_INFO(this->get_logger(),"Timeout Read");  
                    write_data_header.clear();
                    read_data.clear();
                    display_string.clear();
                    read_string.clear();     
                }
                reading = false;
            
            }
            
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_read;
        rclcpp::TimerBase::SharedPtr timer_write;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        LibSerial::SerialPort serial_port_;
        LibSerial::DataBuffer read_data;
        LibSerial::DataBuffer write_data_header;
        LibSerial::DataBuffer write_data_payload;
        std::string read_string;
        std::string display_string;
        uint8_t msg_len;
        bool reading;
        bool writing;
};




int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    ::signal(SIGINT,signal_handler);
    rclcpp::spin(std::make_shared<UsbSerialListener>());
    rclcpp::shutdown();
    return 0;
}
