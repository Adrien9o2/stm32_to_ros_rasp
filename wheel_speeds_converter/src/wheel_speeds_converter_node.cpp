#include <cstdio>

#include <rclcpp/rclcpp.hpp>


#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32.hpp>


using std::placeholders::_1;

volatile sig_atomic_t running = 1;

using namespace std::chrono_literals;

typedef enum _id_moteurs
{
	front_left,
	front_right,
	back_left,
	back_right
}id_moteurs;


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
        : Node("wheel_speeds_converter_node"), robot_speeds_ok(false), wheel_speeds_ok(false), first_heading_set(false), first_cmd_read(false)
        {

          this->declare_parameter("L1PL2_m", 0.2);
          this->declare_parameter("wheel_diameter_m", 0.045);
          this->declare_parameter("kptheta",0.1);
          registered_L1PL2_m = this->get_parameter("L1PL2_m").as_double();
          registered_wheel_radius_m = this->get_parameter("L1PL2_m").as_double();
          kptheta = this->get_parameter("kptheta").as_double();
          registered_L1PL2_m_save = registered_L1PL2_m;
          registered_wheel_radius_m_save = registered_wheel_radius_m;
          kptheta_save = kptheta;

          param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
          
          auto cb_L1PL2 = [this](const rclcpp::Parameter & p) {
              this->registered_L1PL2_m_save = p.as_double();
          };
          auto cb_wheel_radius = [this](const rclcpp::Parameter & p) {
              this->registered_wheel_radius_m_save = p.as_double();
          };
          auto cb_kp = [this](const rclcpp::Parameter & p) {
              this->kptheta_save = p.as_double();
          };            
          
          cb_handle_ = param_subscriber_->add_parameter_callback("L1PL2_m", cb_L1PL2);
          cb_handle_ = param_subscriber_->add_parameter_callback("wheel_diameter_m", cb_wheel_radius);
          cb_handle_ = param_subscriber_->add_parameter_callback("kptheta",cb_kp);

          publisher_wheel_speeds_rad_s = this->create_publisher<std_msgs::msg::Float32MultiArray>("cmd/wheel_speeds_rad_s",10);
          publisher_robot_speeds_m_s_and_rad_s = this->create_publisher<geometry_msgs::msg::TwistStamped>("odo/robot_speed_m_s_and_rad_s",10);

          received_from_serial_wheel_speeds.data.assign(4,0.0);
          to_send_to_serial_wheel_speeds.data.assign(4,0.0);

          subscriber_wheel_speeds_rad_s = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "odo/wheel_speeds_rad_s",
            10, 
            std::bind(&WheelSpeedsConverter::topic_callback_wheel_speeds, this, _1)
          );

          subscriber_robot_speeds_m_s_and_rad_s = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "cmd/robot_speed_m_s_and_rad_s",
            10, 
            std::bind(&WheelSpeedsConverter::topic_callback_robot_speeds, this, _1)
          );

          subscriber_gyro_rad = this->create_subscription<std_msgs::msg::Float32>("/odo/gyro_rad",10,std::bind(&WheelSpeedsConverter::topic_callback_gyro,this,_1));

          loop_timer = this->create_wall_timer(
              0ms, std::bind(&WheelSpeedsConverter::loop, this)
          );
        }

        ~WheelSpeedsConverter()
        {

        }

        void loop()
        {
          const builtin_interfaces::msg::Time timestamp = get_clock()->now();
          if(wheel_speeds_ok)
          {
            compute_wheels_to_robot(
              received_from_serial_wheel_speeds.data.data() + front_left,
              received_from_serial_wheel_speeds.data.data() + front_right,
              received_from_serial_wheel_speeds.data.data() + back_left,
              received_from_serial_wheel_speeds.data.data() + back_right,
              &received_from_serial_robot_speeds.twist.linear.x,
              &received_from_serial_robot_speeds.twist.linear.y,
              &received_from_serial_robot_speeds.twist.angular.z
            );           
            received_from_serial_robot_speeds.header.set__stamp(timestamp);  
            publisher_robot_speeds_m_s_and_rad_s->publish(received_from_serial_robot_speeds);
            wheel_speeds_ok = false;
          }
          if( robot_speeds_ok)
          {
            if( first_cmd_read == false || first_heading_set==false)
            {
              compute_robot_to_wheels(
                &received_from_cmd_robot_speeds.twist.linear.x,
                &received_from_cmd_robot_speeds.twist.linear.y,
                &received_from_cmd_robot_speeds.twist.angular.z,
                to_send_to_serial_wheel_speeds.data.data() + front_left,
                to_send_to_serial_wheel_speeds.data.data() + front_right,
                to_send_to_serial_wheel_speeds.data.data() + back_left,
                to_send_to_serial_wheel_speeds.data.data() + back_right
              );
              publisher_wheel_speeds_rad_s->publish(to_send_to_serial_wheel_speeds);
              robot_speeds_ok = false;
              last_received_from_cmd_robot_speeds = received_from_cmd_robot_speeds;
              first_cmd_read = true;
            }
            else
            {
              double corrected_angular = received_from_cmd_robot_speeds.twist.angular.z + kptheta*(last_received_from_cmd_robot_speeds.twist.angular.z-vtheta);
              compute_robot_to_wheels(
                &received_from_cmd_robot_speeds.twist.linear.x,
                &received_from_cmd_robot_speeds.twist.linear.y,
                &corrected_angular,
                to_send_to_serial_wheel_speeds.data.data() + front_left,
                to_send_to_serial_wheel_speeds.data.data() + front_right,
                to_send_to_serial_wheel_speeds.data.data() + back_left,
                to_send_to_serial_wheel_speeds.data.data() + back_right
              );
              publisher_wheel_speeds_rad_s->publish(to_send_to_serial_wheel_speeds);
              robot_speeds_ok = false;
              last_received_from_cmd_robot_speeds = received_from_cmd_robot_speeds;
              first_cmd_read = true;
            }

          }
          registered_L1PL2_m = registered_L1PL2_m_save;
          registered_wheel_radius_m = registered_wheel_radius_m_save;
          kptheta = kptheta_save;

        }
        void topic_callback_gyro( const std_msgs::msg::Float32::SharedPtr msg)
        {
          if( first_heading_set == false)
          {
            last_heading_rad = msg->data;
            first_heading_set = true;
            timer_value_s = this->get_clock()->now().seconds();
            return;
          }
          double now = this->get_clock()->now().seconds();
          double delta = timer_value_s - now;
          vtheta = msg->data - last_heading_rad / delta;
          last_heading_rad = msg->data;
          timer_value_s = now;
        }

        void topic_callback_wheel_speeds(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
          std::memcpy(received_from_serial_wheel_speeds.data.data(),msg.get()->data.data(),4*sizeof(float));
          wheel_speeds_ok = true;
        }

        void topic_callback_robot_speeds(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        {
          received_from_cmd_robot_speeds.twist.linear.x = msg->twist.linear.x;
          received_from_cmd_robot_speeds.twist.linear.y = msg->twist.linear.y;
          received_from_cmd_robot_speeds.twist.angular.z = msg->twist.angular.z;
          robot_speeds_ok = true;
        }

        void compute_wheels_to_robot(const float *w1, const float *w2, const float *w3, const float *w4, 
                                                double *vx, double *vy, double *wz)
        {
            //                                                               |w1|
            // |vx|         |    1            1           1        1     |   |w2|
            // |vy| = R/4 * |    1           -1          -1        1     | x |w3|
            // |wz|         |-1/(L1+L2)  1/(L1+L2)  -1/(L1+L2)  1/(L1+L2)|   |w4|

            *vx = ((double)registered_wheel_radius_m_save/4.0) * (*w1 + *w2 + *w3 + *w4);
            *vy = -((double)registered_wheel_radius_m_save/4.0) * (*w1 - *w2 - *w3 + *w4);
            *wz = ((double)registered_wheel_radius_m_save/(4.0*(registered_L1PL2_m_save))) * (-*w1 + *w2 - *w3 + *w4);
        }

        void compute_robot_to_wheels(const double *vx, const double *vy, const double *wz, 
                                                float *w1, float *w2, float *w3, float *w4)
        {
            // |w1|         |1      1     -(L1+L2)|   |vx|
            // |w2|         |1     -1     -(L1+L2)|   |vy|
            // |w3| = 1/R * |1     -1     -(L1+L2)| x |wz|
            // |w4|         |1      1     -(L1+L2)|

            *w1 = (1.0/((double)registered_wheel_radius_m_save)) * (*vx + *vy + (registered_L1PL2_m_save)*(*wz));
            *w2 = (1.0/((double)registered_wheel_radius_m_save)) * (*vx - *vy - (registered_L1PL2_m_save)*(*wz));
            *w3 = (1.0/((double)registered_wheel_radius_m_save)) * (*vx - *vy + (registered_L1PL2_m_save)*(*wz));
            *w4 = (1.0/((double)registered_wheel_radius_m_save)) * (*vx + *vy - (registered_L1PL2_m_save)*(*wz));
        }
   

    private:
      rclcpp::TimerBase::SharedPtr loop_timer;

      rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_robot_speeds_m_s_and_rad_s;
      rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_wheel_speeds_rad_s;
      rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_wheel_speeds_rad_s;
      rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_gyro_rad;
      rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_robot_speeds_m_s_and_rad_s;
      
      std_msgs::msg::Float32MultiArray received_from_serial_wheel_speeds;
      std_msgs::msg::Float32MultiArray to_send_to_serial_wheel_speeds;
      geometry_msgs::msg::TwistStamped received_from_cmd_robot_speeds;
      geometry_msgs::msg::TwistStamped last_received_from_cmd_robot_speeds;
      geometry_msgs::msg::TwistStamped received_from_serial_robot_speeds;

      std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
      std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

      double registered_L1PL2_m;
      double registered_wheel_radius_m;
      double registered_L1PL2_m_save;
      double registered_wheel_radius_m_save;
      double kptheta;
      double kptheta_save;
      double kitheta;
      double kitheta_save;
      double kdtheta;
      double kdtheta_save;

      float last_heading_rad;
      bool first_heading_set;
      double timer_value_s;
      double vtheta;
      bool first_cmd_read;

      bool robot_speeds_ok;
      bool wheel_speeds_ok;
};

int main(int argc, char ** argv)
{
  ::signal(SIGINT,signal_handler);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelSpeedsConverter>());
  rclcpp::shutdown();
  return 0;
}
