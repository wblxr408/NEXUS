// Optional GUI integration probe: uses RViz's real frame, displays and renderer.
// Run against the synthetic fixture; this executable only subscribes to data.
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <QApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_rendering/render_window.hpp>

QJsonObject propertyTree(rviz_common::properties::Property * property)
{
  QJsonArray children;
  for (int i = 0; i < property->numChildren(); ++i) {
    children.append(propertyTree(property->childAt(i)));
  }
  return {{"name", property->getName()}, {"value", property->getValue().toString()},
    {"children", children}};
}

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "Usage: rviz_render_probe CONFIG.rviz OUTPUT_PREFIX\n";
    return 2;
  }
  const std::string config = argv[1], output = argv[2];
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
    "nexus_rviz_render_probe");
  auto frame = std::make_unique<rviz_common::VisualizationFrame>(node);
  frame->setApp(&app);
  frame->setSplashPath("");
  frame->initialize(node, QString::fromStdString(config));
  frame->resize(1200, 800);
  frame->show();
  QTimer::singleShot(4000, [&]() {
    frame->getRenderWindow()->captureScreenShot(output + ".png");
    auto * group = frame->getManager()->getRootDisplayGroup();
    std::ofstream stream(output + ".json");
    stream << QJsonDocument(propertyTree(group)).toJson().toStdString();
    std::cout << "RViz captured " << group->numDisplays() << " real displays\n";
    app.quit();
  });
  const int result = app.exec();
  frame.reset();
  node.reset();
  rclcpp::shutdown();
  return result;
}
