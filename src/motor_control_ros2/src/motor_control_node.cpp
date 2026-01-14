#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <map>
#include <thread>
#include <chrono>

#include "motor_control_ros2/motor_base.hpp"
#include "motor_control_ros2/dji_motor.hpp"
#include "motor_control_ros2/damiao_motor.hpp"
#include "motor_control_ros2/unitree_motor.hpp"
#include "motor_control_ros2/can_driver.hpp"
#include "motor_control_ros2/serial_port.hpp"

#include "motor_control_ros2/msg/dji_motor_state.hpp"
#include "motor_control_ros2/msg/dji_motor_command.hpp"
#include "motor_control_ros2/msg/damiao_motor_state.hpp"
#include "motor_control_ros2/msg/damiao_motor_command.hpp"
#include "motor_control_ros2/msg/unitree_motor_state.hpp"
#include "motor_control_ros2/msg/unitree_motor_command.hpp"
#include "motor_control_ros2/msg/unitree_go8010_state.hpp"
#include "motor_control_ros2/msg/unitree_go8010_command.hpp"

namespace motor_control {

/**
 * @brief 电机控制节点
 * 
 * C++ 实时控制节点，负责：
 * - 管理多个电机
 * - 500Hz 控制循环
 * - CAN 通信
 * - 串口通信 (宇树电机)
 * - 发布电机状态
 */
class MotorControlNode : public rclcpp::Node {
public:
  MotorControlNode() : Node("motor_control_node") {
    // 声明参数
    this->declare_parameter("control_frequency", 500.0);
    this->declare_parameter("publish_frequency", 100.0);
    
    // 初始化 CAN 网络
    can_network_ = std::make_shared<CANNetwork>();
    
    // 设置 CAN 接收回调
    can_network_->setCallback(
      std::bind(&MotorControlNode::canRxCallback, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4)
    );
    
    // 初始化电机
    initializeMotors();
    initializeUnitreeMotors();
    
    // 启动 CAN 接收
    can_network_->start();
    
    // 创建发布者
    createPublishers();
    
    // 创建订阅者
    createSubscribers();
    
    // 启动控制循环
    double control_freq = this->get_parameter("control_frequency").as_double();
    control_timer_ = this->create_wall_timer(
      std::chrono::microseconds(static_cast<int>(1e6 / control_freq)),
      std::bind(&MotorControlNode::controlLoop, this)
    );
    
    // 启动发布循环
    double publish_freq = this->get_parameter("publish_frequency").as_double();
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000 / publish_freq)),
      std::bind(&MotorControlNode::publishStates, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "电机控制节点已启动");
  }
  
  ~MotorControlNode() {
    can_network_->stop();
    can_network_->closeAll();
    
    for (auto& [name, port] : serial_ports_) {
       port->close();
    }
  }

private:
  void initializeMotors() {
    // 示例：添加一个 CAN 总线
    can_network_->addBus(0, "/dev/ttyACM0", 921600);
    
    // 示例：添加一个 DJI GM6020 电机
    auto dji_motor = std::make_shared<DJIMotor>(
      "yaw_motor", MotorType::DJI_GM6020, 1, 0
    );
    motors_["yaw_motor"] = dji_motor;
    dji_motors_.push_back(dji_motor);
    
    RCLCPP_INFO(this->get_logger(), "已初始化主要电机");
  }

  // 辅助函数：获取或创建串口
  std::shared_ptr<SerialPort> getSerialPort(const std::string& port_name) {
    if (serial_ports_.find(port_name) == serial_ports_.end()) {
      auto port = std::make_shared<SerialPort>(port_name);
      if (port->open()) {
        serial_ports_[port_name] = port;
        RCLCPP_INFO(this->get_logger(), "已打开串口: %s", port_name.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", port_name.c_str());
        return nullptr;
      }
    }
    return serial_ports_[port_name];
  }

  void initializeUnitreeMotors() {
    // 示例：添加一个宇树 A1 电机 (ID 0)
    std::string port_name = "/dev/ttyUSB0";
    auto port = getSerialPort(port_name);
    
    if (port) {
      auto unitree_motor = std::make_shared<UnitreeMotor>(
        "fl_hip_motor", MotorType::UNITREE_A1, port, 0, 1, 0.0f
      );
      motors_["fl_hip_motor"] = unitree_motor;
      unitree_motors_.push_back(unitree_motor);
      RCLCPP_INFO(this->get_logger(), "已添加宇树 A1 电机: fl_hip_motor");
    }
  }
  
  void createPublishers() {
    // DJI 电机状态发布者
    dji_state_pub_ = this->create_publisher<motor_control_ros2::msg::DJIMotorState>(
      "dji_motor_states", 10
    );
    
    // 达妙电机状态发布者
    damiao_state_pub_ = this->create_publisher<motor_control_ros2::msg::DamiaoMotorState>(
      "damiao_motor_states", 10
    );

    // 宇树电机状态发布者
    unitree_state_pub_ = this->create_publisher<motor_control_ros2::msg::UnitreeMotorState>(
      "unitree_motor_states", 10
    );
    unitree_go_state_pub_ = this->create_publisher<motor_control_ros2::msg::UnitreeGO8010State>(
      "unitree_go8010_states", 10
    );
  }
  
  void createSubscribers() {
    // DJI 电机命令订阅者
    dji_cmd_sub_ = this->create_subscription<motor_control_ros2::msg::DJIMotorCommand>(
      "dji_motor_command", 10,
      std::bind(&MotorControlNode::djiCommandCallback, this, std::placeholders::_1)
    );
    
    // 达妙电机命令订阅者
    damiao_cmd_sub_ = this->create_subscription<motor_control_ros2::msg::DamiaoMotorCommand>(
      "damiao_motor_command", 10,
      std::bind(&MotorControlNode::damiaoCommandCallback, this, std::placeholders::_1)
    );
    
    // 宇树电机命令订阅者 (A1)
    unitree_cmd_sub_ = this->create_subscription<motor_control_ros2::msg::UnitreeMotorCommand>(
      "unitree_motor_command", 10,
      std::bind(&MotorControlNode::unitreeCommandCallback, this, std::placeholders::_1)
    );

    // 宇树电机命令订阅者 (GO-8010)
    unitree_go_cmd_sub_ = this->create_subscription<motor_control_ros2::msg::UnitreeGO8010Command>(
      "unitree_go8010_command", 10,
      std::bind(&MotorControlNode::unitreeGOCommandCallback, this, std::placeholders::_1)
    );
  }
  
  void canRxCallback(uint8_t bus_id, uint32_t can_id, 
                     const uint8_t* data, size_t len) {
    (void)bus_id; // 消除警告
    // 将 CAN 帧分发给对应的电机
    for (auto& [name, motor] : motors_) {
      motor->updateFeedback(can_id, data, len);
    }
  }
  
  void controlLoop() {
    // 500Hz 控制循环
    
    // 1. 读取电机状态
    readUnitreeMotors();
    
    // 2. 写入电机命令
    writeDJIMotors();
    writeDamiaoMotors();
    writeUnitreeMotors();
  }

  void readUnitreeMotors() {
    for (auto& motor : unitree_motors_) {
      motor->receiveFeedback();
    }
  }

  void writeUnitreeMotors() {
    for (auto& motor : unitree_motors_) {
      motor->sendCommand();
    }
  }
  
  void writeDJIMotors() {
    if (dji_motors_.empty()) return;
    
    // DJI 电机需要拼包发送
    std::map<uint32_t, std::vector<std::shared_ptr<DJIMotor>>> groups;
    
    for (auto& motor : dji_motors_) {
      groups[motor->getControlId()].push_back(motor);
    }
    
    for (auto& [control_id, motors] : groups) {
      uint8_t data[8] = {0};
      
      for (auto& motor : motors) {
        uint8_t motor_id = motor->getMotorId();
        uint8_t bytes[2];
        motor->getControlBytes(bytes);
        
        int offset = ((motor_id - 1) % 4) * 2;
        data[offset] = bytes[0];
        data[offset + 1] = bytes[1];
      }
      
      can_network_->send(motors[0]->getBusId(), control_id, data, 8);
    }
  }
  
  void writeDamiaoMotors() {
    for (auto& [name, motor] : motors_) {
      if (motor->getMotorType() == MotorType::DAMIAO_DM4340 ||
          motor->getMotorType() == MotorType::DAMIAO_DM4310) {
        
        auto damiao = std::dynamic_pointer_cast<DamiaoMotor>(motor);
        if (damiao) {
          uint32_t can_id;
          uint8_t data[8];
          size_t len;
          
          damiao->getControlFrame(can_id, data, len);
          if (len > 0) {
            can_network_->send(0, can_id, data, len);
          }
        }
      }
    }
  }
  
  void publishStates() {
    // 发布 DJI 电机状态
    if (!dji_motors_.empty()) {
      auto msg = motor_control_ros2::msg::DJIMotorState();
      msg.header.stamp = this->now();
      
      for (auto& motor : dji_motors_) {
        msg.joint_name = motor->getJointName();
        msg.model = (motor->getMotorType() == MotorType::DJI_GM6020) ? "GM6020" : "GM3508";
        msg.online = motor->isOnline();
        msg.angle = motor->getOutputPosition() * 180.0 / M_PI;
        msg.temperature = static_cast<uint8_t>(motor->getTemperature());
        
        dji_state_pub_->publish(msg);
      }
    }
    
    // 发布达妙电机状态
    // (逻辑与 DJI 类似，此处略，可根据 demand 添加)

    // 发布宇树电机状态
    if (!unitree_motors_.empty()) {
      for (auto& motor : unitree_motors_) {
        // A1
        if (motor->getMotorType() == MotorType::UNITREE_A1) {
          auto msg = motor_control_ros2::msg::UnitreeMotorState();
          msg.header.stamp = this->now();
          msg.joint_name = motor->getJointName();
          msg.online = motor->isOnline();
          msg.position = motor->getOutputPosition();
          msg.velocity = motor->getOutputVelocity();
          msg.torque = motor->getOutputTorque();
          msg.temperature = static_cast<int8_t>(motor->getTemperature());
          unitree_state_pub_->publish(msg);
        } 
        // GO-8010
        else if (motor->getMotorType() == MotorType::UNITREE_GO8010) {
          auto msg = motor_control_ros2::msg::UnitreeGO8010State();
          msg.header.stamp = this->now();
          msg.joint_name = motor->getJointName();
          msg.online = motor->isOnline();
          msg.position = motor->getOutputPosition();
          msg.velocity = motor->getOutputVelocity();
          msg.torque = motor->getOutputTorque();
          msg.temperature = static_cast<int8_t>(motor->getTemperature());
          unitree_go_state_pub_->publish(msg);
        }
      }
    }
  }
  
  void djiCommandCallback(const motor_control_ros2::msg::DJIMotorCommand::SharedPtr msg) {
    auto it = motors_.find(msg->joint_name);
    if (it != motors_.end()) {
      auto dji = std::dynamic_pointer_cast<DJIMotor>(it->second);
      if (dji) {
        dji->setOutput(msg->output);
      }
    }
  }
  
  void damiaoCommandCallback(const motor_control_ros2::msg::DamiaoMotorCommand::SharedPtr msg) {
    auto it = motors_.find(msg->joint_name);
    if (it != motors_.end()) {
      auto damiao = std::dynamic_pointer_cast<DamiaoMotor>(it->second);
      if (damiao) {
        damiao->setMITCommand(msg->pos_des, msg->vel_des, msg->kp, msg->kd, msg->torque_ff);
      }
    }
  }
  
  void unitreeCommandCallback(const motor_control_ros2::msg::UnitreeMotorCommand::SharedPtr msg) {
    auto it = motors_.find(msg->joint_name);
    if (it != motors_.end()) {
      auto motor = std::dynamic_pointer_cast<UnitreeMotor>(it->second);
      if (motor) {
        motor->setHybridCommand(msg->pos_des, msg->vel_des, msg->kp, msg->kd, msg->torque_ff);
        if (msg->mode == 10) motor->enable();
        else if (msg->mode == 0) motor->disable();
      }
    }
  }

  void unitreeGOCommandCallback(const motor_control_ros2::msg::UnitreeGO8010Command::SharedPtr msg) {
    auto it = motors_.find(msg->joint_name);
    if (it != motors_.end()) {
      auto motor = std::dynamic_pointer_cast<UnitreeMotor>(it->second);
      if (motor) {
        motor->setHybridCommand(msg->pos_des, msg->vel_des, msg->kp, msg->kd, msg->torque_ff);
         if (msg->mode == 10) motor->enable();
        else if (msg->mode == 0) motor->disable();
      }
    }
  }

private:
  std::shared_ptr<CANNetwork> can_network_;
  std::map<std::string, std::shared_ptr<MotorBase>> motors_;
  std::vector<std::shared_ptr<DJIMotor>> dji_motors_;
  
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  
  rclcpp::Publisher<motor_control_ros2::msg::DJIMotorState>::SharedPtr dji_state_pub_;
  rclcpp::Publisher<motor_control_ros2::msg::DamiaoMotorState>::SharedPtr damiao_state_pub_;
  rclcpp::Publisher<motor_control_ros2::msg::UnitreeMotorState>::SharedPtr unitree_state_pub_;
  rclcpp::Publisher<motor_control_ros2::msg::UnitreeGO8010State>::SharedPtr unitree_go_state_pub_;
  
  rclcpp::Subscription<motor_control_ros2::msg::DJIMotorCommand>::SharedPtr dji_cmd_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::DamiaoMotorCommand>::SharedPtr damiao_cmd_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::UnitreeMotorCommand>::SharedPtr unitree_cmd_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::UnitreeGO8010Command>::SharedPtr unitree_go_cmd_sub_;

  std::map<std::string, std::shared_ptr<SerialPort>> serial_ports_;
  std::vector<std::shared_ptr<UnitreeMotor>> unitree_motors_;
};

} // namespace motor_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motor_control::MotorControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
