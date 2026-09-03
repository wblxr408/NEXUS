#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>

static char buf[16] = {0};
static std_msgs::msg::Int16 cmd;

void print_help() {
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "========== FCU Command Help ========");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "基本控制指令：");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  a - 解锁");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  d - 锁定");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  t - 起飞");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  l - 降落");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "任务模式指令：");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  p - 巡航（路径规划）");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  r - 绕圆");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  c - 原地转圈");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  s - 停止（悬停当前位置）");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "路口点指令：");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  1 - 路口1");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  2 - 路口2");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  3 - 路口3");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  4 - 路口4");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "位置点指令：");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  5 - 原点");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "目标追踪指令：");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  q - 前向追踪");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  w - 下视追踪");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  e - 停止追踪");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "其他指令：");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  h - 显示帮助信息");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "  x - 退出程序");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "=====================================");
  RCLCPP_INFO(rclcpp::get_logger("fcu_command"), "");
}

int main(int argc, char **argv) {

  rclcpp::init(argc,argv);
  auto node = rclcpp::Node::make_shared("fcu_command");

  // Keep the ROS2 command topic identical to the ROS1 fcu_bridge interface.
  // The hardware bridge subscribes to /fcu_bridge/command.
  auto command = node->create_publisher<std_msgs::msg::Int16>("/fcu_bridge/command", 100);

  print_help();

  // ros2 launch 不会转发 stdin，直接打开 /dev/tty 读取终端输入
  int tty_fd = open("/dev/tty", O_RDONLY);
  if (tty_fd < 0) {
    RCLCPP_ERROR(node->get_logger(), "无法打开 /dev/tty，请使用 ros2 run 启动");
    rclcpp::shutdown();
    return 1;
  }

  while (rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(),"请输入指令（h查看帮助）：");
    ssize_t size = read(tty_fd, buf, sizeof(buf));
    if(size>0){
      if(size!=2){
        RCLCPP_INFO(node->get_logger(),"指令错误！请输入单个字符");
        continue;
      }
    }else{
      RCLCPP_INFO(node->get_logger(),"退出程序");
      close(tty_fd);
      rclcpp::shutdown();
      return 0;
    }
    switch(buf[0]){
      case 'a':
        RCLCPP_INFO(node->get_logger(),"[执行] 解锁");
        cmd.data=1;
        command->publish(cmd);
        break;
      case 'd':
        RCLCPP_INFO(node->get_logger(),"[执行] 锁定");
        cmd.data=2;
        command->publish(cmd);
        break;
      case 't':
        RCLCPP_INFO(node->get_logger(),"[执行] 起飞");
        cmd.data=3;
        command->publish(cmd);
        break;
      case 'l':
        RCLCPP_INFO(node->get_logger(),"[执行] 降落");
        cmd.data=4;
        command->publish(cmd);
        break;
      case 'p':
        RCLCPP_INFO(node->get_logger(),"[执行] 巡航模式");
        cmd.data=0;
        command->publish(cmd);
        break;
      case 'r':
        RCLCPP_INFO(node->get_logger(),"[执行] 绕圆模式");
        cmd.data=5;
        command->publish(cmd);
        break;
      case 'c':
        RCLCPP_INFO(node->get_logger(),"[执行] 原地转圈");
        cmd.data=12;
        command->publish(cmd);
        break;
      case 's':
        RCLCPP_INFO(node->get_logger(),"[执行] 停止（悬停当前位置）");
        cmd.data=6;
        command->publish(cmd);
        break;
      case '1':
        RCLCPP_INFO(node->get_logger(),"[执行] 路口1");
        cmd.data=8;
        command->publish(cmd);
        break;
      case '2':
        RCLCPP_INFO(node->get_logger(),"[执行] 路口2");
        cmd.data=9;
        command->publish(cmd);
        break;
      case '3':
        RCLCPP_INFO(node->get_logger(),"[执行] 路口3");
        cmd.data=10;
        command->publish(cmd);
        break;
      case '4':
        RCLCPP_INFO(node->get_logger(),"[执行] 路口4");
        cmd.data=11;
        command->publish(cmd);
        break;
      case '5':
        RCLCPP_INFO(node->get_logger(),"[执行] 原点");
        cmd.data=7;
        command->publish(cmd);
        break;
      case 'q':
        RCLCPP_INFO(node->get_logger(),"[执行] 前向追踪");
        cmd.data=1011;
        command->publish(cmd);
        break;
      case 'w':
        RCLCPP_INFO(node->get_logger(),"[执行] 下视追踪");
        cmd.data=1012;
        command->publish(cmd);
        break;
      case 'e':
        RCLCPP_INFO(node->get_logger(),"[执行] 停止追踪");
        cmd.data=1013;
        command->publish(cmd);
        break;
      case 'h':
        print_help();
        break;
      case 'x':
        RCLCPP_INFO(node->get_logger(),"退出程序");
        rclcpp::shutdown();
        return 0;
      default:
        RCLCPP_INFO(node->get_logger(),"非法指令！请输入h查看帮助");
        break;
    }
  }

  return 0;
}
