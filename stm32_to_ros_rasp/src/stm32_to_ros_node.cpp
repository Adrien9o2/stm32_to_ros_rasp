#include <cstdio>
#include <chrono>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <list>
#include <thread>

#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"

#include "stm32_to_ros/SerialID.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

volatile sig_atomic_t running = 1;

static void signal_handler(int signum)
{
    (void)signum;
    running = 0;
    rclcpp::shutdown();
}

class WheelSpeedsConverter : public rclcpp::Node
{
    public:
        WheelSpeedsConverter()
        : Node("usb_serial_listener_node")
        {
            std::vector<std::string> port_list = serial_port_.GetAvailableSerialPorts();
            
            // std::string searchString = "/dev/ttyACM";
            
            // std::vector<std::string>::iterator iterator = std::find_if(port_list.begin(), port_list.end(), [searchString](const std::string& str) {
            //     return str.compare(0, searchString.length(), searchString) == 0;
            // });
            // if( iterator == port_list.end() )
            // {
            //     RCLCPP_ERROR(this->get_logger(),"No board connected !");         
            //     rclcpp::shutdown();
            //     return;
            // }
            // serial_port_string = *iterator;
            serial_port_.Open("/dev/stmotors");
            serial_port_gyro.Open("/dev/stgyro");
            
            serial_port_.SetBaudRate( LibSerial::BaudRate::BAUD_115200);
            serial_port_.FlushIOBuffers();


            serial_port_gyro.SetBaudRate( LibSerial::BaudRate::BAUD_115200);
            serial_port_gyro.FlushIOBuffers();

            saving = false;
            in_loop = false;
     
            to_send_to_serial_wheel_speeds.data.assign(4,0.0);
            to_send_to_serial_wheel_speeds_save.data.assign(4,0.0);
            received_from_serial_wheel_speeds.data.assign(4,0.0);

            write_data_payload_motor_speeds.assign(4*sizeof(float),0.0);

            publisher_wheel_speeds_rad_s = this->create_publisher<std_msgs::msg::Float32MultiArray>("odo/wheel_speeds_rad_s",10);
            publisher_gyro_rad = this->create_publisher<std_msgs::msg::Float32>("odo/gyro_rad",10);
            subscriber_wheel_speeds_rad_s = this->create_subscription<std_msgs::msg::Float32MultiArray>("cmd/wheel_speeds_rad_s",10, std::bind(&WheelSpeedsConverter::topic_callback, this, _1));
           
            timer_write = this->create_wall_timer(
                0ms, std::bind(&WheelSpeedsConverter::writing_to_serial_callback, this));
            timer_write_gyro = this ->create_wall_timer(0ms, std::bind(&WheelSpeedsConverter::gyro_callback,this));
        }


        ~WheelSpeedsConverter()
        {
            serial_port_.Close();
        }

        void gyro_callback()
        {
            try
            {
                serial_port_gyro.FlushIOBuffers();
                serial_port_gyro.ReadLine(serial_port_string_gyro);
                serial_port_gyro.FlushIOBuffers();
                to_publish_gyro_rad.data = std::stof(serial_port_string_gyro)*M_PI/180.0;
                publisher_gyro_rad->publish(to_publish_gyro_rad);            
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            
        }

        void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            if(in_loop)
            {
                saving = true;
                std::memcpy(to_send_to_serial_wheel_speeds_save.data.data(),msg.get()->data.data(),4*sizeof(float));
            }
            else
            {
                std::memcpy(to_send_to_serial_wheel_speeds.data.data(),msg.get()->data.data(),4*sizeof(float));
            }

        
        }


        void writing_to_serial_callback()
        {
           
            if(running)
            {
                in_loop = false;
                try
                {
    
                    
                    std::memcpy(
                        write_data_payload_motor_speeds.data(),
                        to_send_to_serial_wheel_speeds.data.data(),
                        4*sizeof(float)
                    );
                    std::cout << write_data_payload_motor_speeds.size() << std::endl;
                    serial_port_.Write(write_data_payload_motor_speeds);
                    serial_port_.DrainWriteBuffer();
                    serial_port_.Read(read_data,4*sizeof(float),100);
                    std::memcpy(
                        received_from_serial_wheel_speeds.data.data(),
                        read_data.data(),
                        4*sizeof(float)
                    );
                    publisher_wheel_speeds_rad_s->publish(received_from_serial_wheel_speeds);

                }
                catch(const LibSerial::ReadTimeout &)
                {
                    RCLCPP_INFO(this->get_logger(),"Timeout");  
            
                } 
                catch( const std::runtime_error & except)
                {
                    RCLCPP_INFO(this->get_logger(),except.what());
                }
                if(saving==true)
                {
                    std::memcpy(
                        to_send_to_serial_wheel_speeds.data.data(),
                        to_send_to_serial_wheel_speeds_save.data.data(),
                        4*sizeof(float)
                    );
                }
                clear_container(read_data);
            }
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_write;
        rclcpp::TimerBase::SharedPtr timer_write_gyro;

        
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_wheel_speeds_rad_s;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_gyro_rad;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_wheel_speeds_rad_s;
        
        LibSerial::SerialPort serial_port_;
        LibSerial::DataBuffer read_data;
        LibSerial::DataBuffer write_data_payload_motor_speeds;

        LibSerial::SerialPort serial_port_gyro;
        
        std_msgs::msg::Float32MultiArray to_send_to_serial_wheel_speeds_save;
        std_msgs::msg::Float32MultiArray to_send_to_serial_wheel_speeds;
        std_msgs::msg::Float32MultiArray received_from_serial_wheel_speeds;
        std_msgs::msg::Float32 to_publish_gyro_rad;
        
        std::string serial_port_string;
        std::string serial_port_string_gyro;


        bool in_loop;
        bool saving;

        template<class T>
        void clear_container(T container)
        {
            if( !container.empty())
            {
                container.clear();
            }
        }
};




int main(int argc, char ** argv)
{
    ::signal(SIGINT,signal_handler);


    rclcpp::init(argc, argv);
    std::shared_ptr<WheelSpeedsConverter> ptr = std::make_shared<WheelSpeedsConverter>();
    rclcpp::spin(ptr);
    rclcpp::shutdown();
    
    return 0;
}
