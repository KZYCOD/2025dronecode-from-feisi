#include "ros/ros.h"
#include <plugins/action/takeoff.h>
#include <plugins/action/land.h>
#include "mavros_cnt.h"
#include "mission.h"
//#include <pluginlib/class_loader.h>
#include <fstream>
#include <behaviortree_cpp/xml_parsing.h>

#include <boost/stacktrace.hpp>
#include <csignal>

#include <filesystem>

namespace fs = std::filesystem;

using namespace BT;

void signal_handler(int signum) {
    std::cerr << "Received signal " << signum << "\n";
    std::cerr << boost::stacktrace::stacktrace();
    std::exit(signum);
}

// #include <execinfo.h>
// #include <unistd.h>
// #include <cstdlib>

// void print_stacktrace() {
//     void* buffer[100];
//     int frames = backtrace(buffer, 100);
//     char** symbols = backtrace_symbols(buffer, frames);
//     if (symbols == nullptr) {
//         perror("backtrace_symbols");
//         return;
//     }
//     // 打印到 STDERR_FILENO（文件描述符，信号安全）
//     for (int i = 0; i < frames; ++i) {
//         dprintf(STDERR_FILENO, "%s\n", symbols[i]);
//     }
//     free(symbols);
// }

// // 信号处理函数中调用
// void signal_handler(int signum) {
//     print_stacktrace();
//     std::_Exit(1);
// }


int main(int argc, char **argv)
{
  std::signal(SIGSEGV, signal_handler);
  std::signal(SIGABRT, signal_handler);
  ros::init(argc, argv, "mission");

  std::vector<std::string> lib_so_files;
  const char *             cmake_prefix_path = std::getenv("CMAKE_PREFIX_PATH");
  if(cmake_prefix_path)
  {
    std::string cmake_prefix_path_str(cmake_prefix_path);
    std::cout << "current cmake path" << std::endl;
    // 解析 CMAKE_PREFIX_PATH 获取 devel 文件夹路径
    size_t pos = cmake_prefix_path_str.find("/devel");
    if(pos != std::string::npos)
    {
      std::string devel_path     = cmake_prefix_path_str.substr(0, pos + 6);
      std::string devel_lib_path = devel_path + "/lib";
      // 遍历目录
      for(const auto &entry : fs::directory_iterator(devel_lib_path))
      {
        if(entry.is_regular_file() && entry.path().extension() == ".so")
        {
          std::string filename = entry.path().filename().string();
          if(filename.substr(0, 3) != "lib")  // 检查是否以 lib 开头
          {
            lib_so_files.push_back(entry.path().string());
          }
        }
      }
      ROS_INFO("devel/lib path is: %s", devel_lib_path.c_str());
    }
    else
    {
      ROS_ERROR("Failed to find 'devel' in CMAKE_PREFIX_PATH: %s",
                cmake_prefix_path_str.c_str());
    }
  }
  else
  {
    ROS_ERROR("CMAKE_PREFIX_PATH environment variable not set!");
  }
  if(lib_so_files.size() == 0)
  {
    ROS_ERROR("haven't get *.so file in devel/lib path");
  }

  ros::NodeHandle nh;
  ros::NodeHandle nh_pri("~");

  auto mavros_obj =
    MavRosConnect::getInstance(std::make_shared<ros::NodeHandle>(nh));
  //  mavros_obj->InitMavRosConnect();

  std::string cfg_path;
  bool        is_debug   = false;
  bool        is_gen_xml = false;
  std::string xml_file   = "";
  nh_pri.param<std::string>("config_path", cfg_path,
                            "/home/zy/ws/src/mission_pkg/config/hitballon.xml");
  nh_pri.getParam("is_debug", is_debug);
  nh_pri.param<bool>("is_gen_xml", is_gen_xml, false);
  nh_pri.param<std::string>("xml_file", xml_file, "cfg.xml");
  std::cout << " cinfig path: " << cfg_path << std::endl;
  BehaviorTreeFactory factory;
  for(size_t i = 0; i < lib_so_files.size(); ++i)
  {
    factory.registerFromPlugin(lib_so_files[i]);
  }

  auto xml_mode = writeTreeNodesModelXML(factory, false);

  if(is_gen_xml)
  {
    std::fstream file(xml_file, std::ios_base::out);
    file << xml_mode;
    ROS_INFO("the xml has generated,it's in %s", xml_file.c_str());
    file.close();
    ros::shutdown();
    return 0;  //生成xml 文件后直接退出
  }
  auto tree = factory.createTreeFromFile(cfg_path);
  std::cout << "bellow is backboard paramers" << std::endl;
  tree.rootBlackboard()->debugMessage();

  StdCoutLogger logger(tree);

  tree.tickWhileRunning();
  ros::spin();

  return 0;
}
