#include <fstream>
#include <sstream>
#include <geometry_msgs/msg/pose.hpp>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <thread>
#include <boost/asio.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include <condition_variable>
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

//initiate the udp server
using boost::asio::ip::udp;
using json = nlohmann::json;

std::atomic<bool> udp_running(true);
std::mutex grasp_point_mutex, 
           left_joint_positions_mutex, 
           right_joint_positions_mutex, 
           mount_joint_positions_mutex, 
           left_ee_pose_mutex,
           right_ee_pose_mutex,
           mount_ee_pose_mutex,
           right_fix_goal_pose_mutex,
           mount_fix_goal_pose_mutex;
std::condition_variable udp_condition_variable, 
                        left_joint_positions_condition_variable, 
                        right_joint_positions_condition_variable, 
                        mount_joint_positions_condition_variable,
                        right_fix_goal_pose_condition_variable,
                        mount_fix_goal_pose_condition_variable;
// std::array<double, 3> grasp_point = {0.0, 0.0, 0.0}; // default value
std::array<double, 6> grasp_point = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // default value
std::array<double, 3> grasp_tangent = {0.0, 1.0, 0.0}; // default value
std::array<double, 7> right_fix_goal_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // default value: x, y, z, w, rx, ry, rz
std::array<double, 7> mount_fix_goal_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // default value: x, y, z, w, rx, ry, rz
// std::array<double, 6> left_joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // default value
std::vector<double> left_joint_positions(6, 0.0);  // Default size 6
std::vector<double> right_joint_positions(7, 0.0); // Default size 7
std::vector<double> mount_joint_positions(7, 0.0); // Default size 7
// std::array<double, 6> ee_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // default value
std::vector<double> left_ee_pose(6, 0.0); // Default size 6
std::vector<double> right_ee_pose(6, 0.0); // Default size 6
std::vector<double> mount_ee_pose(6, 0.0); // Default size 6

// TODO@KejiaChen: get from robot model
std::vector<std::string> left_ur_joint_names = {"left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_elbow_joint", "left_wrist_1_joint", "left_wrist_2_joint", "left_wrist_3_joint"};
std::vector<std::string> right_franka_joint_names = {"right_panda_joint1", "right_panda_joint2", "right_panda_joint3", "right_panda_joint4", "right_panda_joint5", "right_panda_joint6", "right_panda_joint7"};
std::vector<std::string> mount_franka_joint_names = {"mount_panda_joint1", "mount_panda_joint2", "mount_panda_joint3", "mount_panda_joint4", "mount_panda_joint5", "mount_panda_joint6", "mount_panda_joint7"};

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_dual_arm_real");
namespace mtc = moveit::task_constructor;

bool move_to_goal_pose_task_success = false;

void udpReceiverRight(const std::string& host, int port) {
  try {
    boost::asio::io_context io_context;
    udp::socket socket(io_context, udp::endpoint(boost::asio::ip::make_address(host), port));
    std::array<char, 4096> recv_buf;

    while (udp_running) {
      udp::endpoint sender_endpoint;
      size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);

      // parse the received data
      std::string data(recv_buf.data(), len);
      json received_json = json::parse(data);

      // extract the grasp point
      auto x = received_json["data"]["init_grasp_position"]["x"];
      auto y = received_json["data"]["init_grasp_position"]["y"];
      auto z = received_json["data"]["init_grasp_position"]["z"];
      // auto rx = received_json["data"]["init_grasp_rotation"]["rx"];
      // auto ry = received_json["data"]["init_grasp_rotation"]["ry"];
      // auto rz = received_json["data"]["init_grasp_rotation"]["rz"];
      auto tx = received_json["data"]["init_grasp_tangent"]["x"];
      auto ty = received_json["data"]["init_grasp_tangent"]["y"];
      auto tz = received_json["data"]["init_grasp_tangent"]["z"];

      // update the grasp point
      {
        std::lock_guard<std::mutex> lock(grasp_point_mutex);
        grasp_point = {x, y, z};
        grasp_tangent = {tx, ty, tz};
        // grasp_point = {x, y, z,rx,ry,rz};
      }
      udp_condition_variable.notify_one();

      std::cout << "Received grasp point: " << x << ", " << y << ", " << z << std::endl;
      // std::cout << "Received grasp point: " << x << ", " << y << ", " << z  << ", " << rx << ", " << ry << ", " << rz << std::endl;
    }
  } catch (std::exception& e) {
    std::cerr << "UDP Receiver Error: " << e.what() << std::endl;
  }
}

void udpReceiverGoalPose(const std::string& host, int port,
                        std::array<double, 7>& goal_pose,
                        std::mutex& goal_pose_mutex,
                        std::condition_variable& goal_pose_condition_variable
                        ) 
{
  try {
    boost::asio::io_context io_context;
    udp::socket socket(io_context, udp::endpoint(boost::asio::ip::make_address(host), port));
    std::array<char, 4096> recv_buf;

    while (udp_running) {
      udp::endpoint sender_endpoint;
      size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);

      // parse the received data
      std::string data(recv_buf.data(), len);
      json received_json;

      try {
        received_json = json::parse(data);
      } catch (const json::exception& e) {
        std::cerr << "JSON Parsing Error: " << e.what() << std::endl;
        continue;  // Ignore invalid JSON and continue listening
      }

      std::string command = received_json["command"];
      std::cout << "Received command: " << command << std::endl;

      // extract the grasp point
      auto x = received_json["goal_pose"]["position"]["x"];
      auto y = received_json["goal_pose"]["position"]["y"];
      auto z = received_json["goal_pose"]["position"]["z"];
      auto rw = received_json["goal_pose"]["orientation"]["w"];
      auto rx = received_json["goal_pose"]["orientation"]["x"];
      auto ry = received_json["goal_pose"]["orientation"]["y"];
      auto rz = received_json["goal_pose"]["orientation"]["z"];

      // update the grasp point
      {
        std::lock_guard<std::mutex> lock(goal_pose_mutex);
        goal_pose = {x, y, z, rw, rx, ry, rz};
      }
      goal_pose_condition_variable.notify_one();

      std::cout << "Received goal pose: " << x << ", " << y << ", " << z << ", " << rw << ", " << rx << ", " << ry << ", " << rz << std::endl;

      // // Wait until the mount task has succeeded
      // while (!move_to_goal_pose_task_success) {
      //   std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Polling every 100ms
      // }

      // // Send acknowledgment
      // json ack_message;
      // ack_message["ack"] = true;  // Acknowledge success

      // // Send the response back to the sender
      // std::string ack_str = ack_message.dump();
      // socket.send_to(boost::asio::buffer(ack_str), sender_endpoint);

      // // Reset the flag
      // move_to_goal_pose_task_success = false;
    }
  } catch (std::exception& e) {
    std::cerr << "UDP Receiver Error: " << e.what() << std::endl;
  }
}

void udpReceiverSync(const std::string& host, int port,
                    // std::array<double, 6>& joint_positions,
                    std::vector<double>& joint_positions,
                    std::mutex& joint_positions_mutex,
                    std::condition_variable& joint_positions_condition_variable,
                    std::vector<double>& ee_pose,
                    std::mutex& ee_pose_mutex
                    )
{
  try {
    boost::asio::io_context io_context;
    udp::socket socket(io_context, udp::endpoint(boost::asio::ip::make_address(host), port));
    std::array<char, 4096> recv_buf;

    while (udp_running) {
      udp::endpoint sender_endpoint;
      size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);

      // parse the received data
      std::string data(recv_buf.data(), len);
      json received_json;

      try {
        received_json = json::parse(data);
      } catch (const json::exception& e) {
        std::cerr << "JSON Parsing Error: " << e.what() << std::endl;
        continue;  // Ignore invalid JSON and continue listening
      }

      std::string command = received_json["command"];
      std::cout << "Received command: " << command << std::endl;

      // check if q is there
      if (received_json.contains("q") && received_json["q"].is_array())
      {
        std::vector<double> current_q = received_json["q"].get<std::vector<double>>();

        {
          std::lock_guard<std::mutex> joint_lock(joint_positions_mutex);
          joint_positions = current_q;
        }

        // Log received values
        std::cout << "Received synchronization command with joint positions: ";
        for (double q_val : current_q) {
          std::cout << q_val << " ";
        }
        std::cout << std::endl;
      }else {
        std::cerr << "Received joint data is not an array" << std::endl;
      }

      // check if ee_pose is there
      if (received_json.contains("ee_pose") && received_json["ee_pose"].is_array())
      {
        std::vector<double> current_ee = received_json["ee_pose"].get<std::vector<double>>();

        {
          std::lock_guard<std::mutex> cartesian_lock(ee_pose_mutex);
          ee_pose = current_ee;
        }

        // Log received values
        std::cout << "Received synchronization command with end effector pose: ";
        for (double ee_val : current_ee) {
          std::cout << ee_val << " ";
        }
        std::cout << std::endl;
      }else{
        std::cout << "No ee pose data" << std::endl;
      }

      joint_positions_condition_variable.notify_one();
    }
  } catch (std::exception& e) {
    std::cerr << "UDP Receiver Error: " << e.what() << std::endl;
  }
}


class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  // ~MTCTaskNode();

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  bool doSyncTask(bool attach_object, 
                  std::string arm_group_name, 
                  std::string hand_group_name, 
                  std::string hand_frame,
                  std::mutex& joint_positions_mutex,
                  std::vector<double>& joint_positions,
                  std::vector<std::string>& joint_names,
                  std::condition_variable& joint_positions_condition_variable
                  ); 

  void doMountingTask();

  void setupPlanningScene();

  bool getSkipFlag();

  // Robot group names
  std::string left_arm_group_name;
  std::string left_hand_group_name;
  std::string left_hand_frame;

  std::string right_arm_group_name;
  std::string right_hand_group_name ;
  std::string right_hand_frame;

  std::string mount_group_name;
  std::string mount_hand_group_name;
  std::string mount_hand_frame;

  // void updateObjectPose(const geometry_msgs::msg::Pose& new_pose);

private:
  // Compose an MTC task from a series of stages.
  // mtc::Task createTask();
  mtc::Task createLeftArmTask();
  mtc::Task createLeftArmSyncTask();
  mtc::Task createGoalJointTask(bool attach_object, 
                              std::string arm_group_name, 
                              std::string hand_group_name, 
                              std::string hand_frame,
                              std::mutex& joint_positions_mutex,
                              std::vector<double>& joint_positions,
                              std::vector<std::string>& joint_names
                              );
  mtc::Task createGoalPoseTask(std::string arm_group_name, 
                              std::string hand_group_name, 
                              std::string hand_frame,
                              geometry_msgs::msg::PoseStamped goal_pose);
  mtc::Task createRightArmTask(tf2::Quaternion q2); 
  tf2::Matrix3x3 rightGraspOrientation(tf2::Vector3 cable_tangent, tf2::Matrix3x3 left_object_rot);
  mtc::Task task_;
  // // rclcpp::Node::SharedPtr node_;

  // std::thread udp_thread_;
  // std::atomic<bool> stop_thread_{false};  // Thread stop flag
  // geometry_msgs::msg::Pose received_pose_;
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface left_move_group_;
  
  // TF2 components
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Flag
  bool skip_handover_;

  // Helper methods for internal setup
  void initializeGroups();

  moveit_visual_tools::MoveItVisualTools visual_tools_;

  geometry_msgs::msg::PoseStamped getPoseTransform(const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame);

};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) },
    left_move_group_(node_, "left_ur_manipulator"),  // for dual arm or single arm
    visual_tools_(node_, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, left_move_group_.getRobotModel()),
    tf_buffer_(node_->get_clock()), // Initialize TF Buffer with node clock
    tf_listener_(tf_buffer_)       // Initialize TF Listener with TF Buffer
{
  // udp_thread_ = std::thread(&MTCTaskNode::udpReceive, this);  // Start the thread
  // RCLCPP_INFO(node_->get_logger(), "UDP receive thread started.");

  bool skip_handover;
  node_->get_parameter("skip_handover", skip_handover);

  RCLCPP_INFO(node_->get_logger(), "Received parameter 'skip_handover': %s", skip_handover ? "true" : "false");

  // Use the parameter value as needed
  skip_handover_ = skip_handover;

  visual_tools_.loadRemoteControl();

  initializeGroups();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::initializeGroups()
{
  // Define robot group names
  left_arm_group_name = "left_ur_manipulator";
  left_hand_group_name = "left_gripper";
  left_hand_frame = "left_robotiq_85_base_link";

  right_arm_group_name = "right_panda_arm";
  right_hand_group_name = "right_hand";
  right_hand_frame = "right_panda_hand";

  mount_group_name = "mount_panda_arm";
  mount_hand_group_name = "mount_hand";
  mount_hand_frame = "mount_panda_hand";

}

// MTCTaskNode::~MTCTaskNode()
// {
//   stop_thread_ = true;  // set the flag for stopping the thread
//   if (udp_thread_.joinable()) {
//     udp_thread_.join();  // wait until safety
//     RCLCPP_INFO(node_->get_logger(), "UDP receive thread stopped.");
//   }
// }

// void MTCTaskNode::updateObjectPose(const geometry_msgs::msg::Pose& new_pose) {
//   received_pose_ = new_pose;

//   //update CollisionObject position
//   moveit_msgs::msg::CollisionObject object2;
//   object2.id = "object2";
//   object2.header.frame_id = "world";
//   object2.primitives.resize(1);
//   object2.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
//   object2.primitives[0].dimensions = { 0.15, 0.02 };
//   object2.pose = received_pose_;

//   moveit::planning_interface::PlanningSceneInterface psi;
//   psi.applyCollisionObject(object2);
// }


void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.02, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.310;
  pose.position.y = -0.583;
  pose.position.z = 1.105;
  tf2::Quaternion q;
  q.setRPY(M_PI / 2, 0, M_PI / 2);  // Roll (X), Pitch (Y), Yaw (Z)
  pose.orientation = tf2::toMsg(q);
  // pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);


  // updateObjectPose(received_pose_);


  

  // // we try to generate an object2 which center is the same as the end of the rope
  // moveit_msgs::msg::CollisionObject object2;
  // object2.id = "object2";
  // object2.header.frame_id = "world";
  // object2.primitives.resize(1);
  // object2.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  // object2.primitives[0].dimensions = { 0.03, 0.02 };  //make it smaller

  // // read the position and orientation of the end of the rope from the pose.txt
  // std::ifstream pose_file("/path/to/pose.txt");//should be changed to the real path of the pose of the rope end  
  // if (pose_file.is_open()) {
  //   std::string line;
  //   std::getline(pose_file, line);  //read the first line
  //   std::istringstream ss(line);
  //   //asume that the pose.txt only has the position parameter of the end of the rope
  //   ss >> object2.pose.position.x >> object2.pose.position.y >> object2.pose.position.z;

  //   // default
  //   object2.pose.orientation.w = 1.0;
  //   object2.pose.orientation.x = 0.0;
  //   object2.pose.orientation.y = 0.0;
  //   object2.pose.orientation.z = 0.0;

  //   pose_file.close();
  // } else {
  //   RCLCPP_ERROR(rclcpp::get_logger("setupPlanningScene"), "Failed to open pose.txt");
  //   return;
  // }


  // psi.applyCollisionObject(object2);

  //Then all of the "object" in the Task of the right arm should be changed into "object2"
}

bool MTCTaskNode::getSkipFlag()
{
  return skip_handover_;
}

geometry_msgs::msg::PoseStamped MTCTaskNode::getPoseTransform(const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame)
{   
    std::string frame_id = pose.header.frame_id;
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(
            target_frame,   // Target frame
            frame_id,  // Source frame
            rclcpp::Time(0),  // Get the latest transform
            rclcpp::Duration::from_seconds(1.0)// Timeout
        );
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(LOGGER, "Could not transform %s frame to %s frame: %s", frame_id.c_str(), target_frame.c_str(), ex.what());
        return pose;
    }

    geometry_msgs::msg::PoseStamped pose_world;
    tf2::doTransform(pose, pose_world, transform);
    return pose_world;
}

void MTCTaskNode::doTask()
{
  /* planning and execution for the left arm to hand-over position*/
  mtc::Task left_task = createLeftArmTask();  
  try
  {
    left_task.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Left arm task initialization failed: " << e);
    return;
  }

  if (!left_task.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Left arm task planning failed");
    return;
  }

  left_task.introspection().publishSolution(*left_task.solutions().front());

  auto left_result = left_task.execute(*left_task.solutions().front());
  if (left_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Left arm task execution failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Left arm task executed successfully");

  /* synchronize the left arm to current position in the real world */
  RCLCPP_INFO(LOGGER, "Synchronizing the left arm to real-world position...");
  doSyncTask(true, left_arm_group_name, left_hand_group_name, left_hand_frame,
              left_joint_positions_mutex, 
              left_joint_positions, left_ur_joint_names,
              left_joint_positions_condition_variable
              );

  // get the orientation of the left end effector from planning scene
  geometry_msgs::msg::PoseStamped left_current_pose = left_move_group_.getCurrentPose("left_robotiq_85_base_link");
  auto left_ee_orientation = left_current_pose.pose.orientation; 
  // get rotation matrix from quaternion
  tf2::Quaternion left_ee_quat(left_ee_orientation.x, left_ee_orientation.y, left_ee_orientation.z, left_ee_orientation.w);
  tf2::Matrix3x3 left_ee_rot(left_ee_quat);
  tf2::Matrix3x3 rot_ee2object(
    0, 1, 0,
    0, 0, 1,
    1, 0, 0
  );
  tf2::Matrix3x3 left_object_rot = rot_ee2object * left_ee_rot;

  /* planning and execution for the right arm to grasp the object */
  RCLCPP_INFO(LOGGER, "Waiting for grasp point via UDP...");
  {
    std::unique_lock<std::mutex> lock(grasp_point_mutex);
    udp_condition_variable.wait(lock, []() {
      return !std::all_of(grasp_point.begin(), grasp_point.end(), [](double v) { return v == 0.0; });
    });
  }
  RCLCPP_INFO(LOGGER, "Grasp point received: [%f, %f, %f]", grasp_point[0], grasp_point[1], grasp_point[2]);
  RCLCPP_INFO(LOGGER, "Grasp tangent received: [%f, %f, %f]", grasp_tangent[0], grasp_tangent[1], grasp_tangent[2]);

  tf2::Vector3 cable_tangent(grasp_tangent[0], grasp_tangent[1], grasp_tangent[2]);

  // Option 1: Generate pose
  tf2::Matrix3x3 right_object_rot = rightGraspOrientation(cable_tangent, left_object_rot);
  tf2::Quaternion right_object_quat;
  right_object_rot.getRotation(right_object_quat);

  mtc::Task right_task = createRightArmTask(right_object_quat);  

  // Option 2: Fixed pose
  // geometry_msgs::msg::PoseStamped left_current_pose_right_frame = getPoseTransform(left_current_pose, "right_panda_link0");
  // auto left_ee_orientation_right_frame = left_current_pose_right_frame.pose.orientation;
  // tf2::Quaternion left_ee_quat_right_frame(left_ee_orientation_right_frame.x, left_ee_orientation_right_frame.y, left_ee_orientation_right_frame.z, left_ee_orientation_right_frame.w);
  // tf2::Matrix3x3 left_ee_rot_right_frame(left_ee_quat_right_frame);

  // // roation around y axis for 180 degrees
  // tf2::Quaternion right_ee_quat = left_ee_quat_right_frame;

  // // tf2::Matrix3x3 rot_object2ee = rot_ee2object.transpose();
  // // // tf2::Matrix3x3 right_ee_rot = rightGraspOrientation(cable_tangent, left_ee_rot);
  // // tf2::Matrix3x3 right_ee_rot = rot_ee2object*right_object_rot;

  // // tf2::Quaternion right_ee_quat;
  // // right_ee_rot.getRotation(right_ee_quat);

  // geometry_msgs::msg::PoseStamped grasp_pose_in_right_frame;
  // grasp_pose_in_right_frame.header.frame_id = "right_panda_link0";
  // {
  //   std::lock_guard<std::mutex> lock(grasp_point_mutex);
  //   grasp_pose_in_right_frame.pose.position.x = grasp_point[0];
  //   grasp_pose_in_right_frame.pose.position.y = grasp_point[1];
  //   grasp_pose_in_right_frame.pose.position.z = grasp_point[2]; // TODO@KejiaChen: Adjust the z value
  // }
  // grasp_pose_in_right_frame.pose.orientation.w = right_ee_quat.w();
  // grasp_pose_in_right_frame.pose.orientation.x = right_ee_quat.x();
  // grasp_pose_in_right_frame.pose.orientation.y = right_ee_quat.y();
  // grasp_pose_in_right_frame.pose.orientation.z =  right_ee_quat.z();

  // geometry_msgs::msg::PoseStamped grasp_pose_in_world_frame = getPoseTransform(grasp_pose_in_right_frame, "world");

  // mtc::Task right_task = createGoalPoseTask(right_arm_group_name, right_hand_group_name, right_hand_frame, grasp_pose_in_world_frame);
  
  try
  {
    right_task.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Right arm task initialization failed: " << e);
    return;
  }

  if (!right_task.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Right arm task planning failed");
    return;
  }

  right_task.introspection().publishSolution(*right_task.solutions().front());
  visual_tools_.prompt("[Publishing] Press 'next' to execute trajectory");

  auto right_result = right_task.execute(*right_task.solutions().front());
  if (right_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Right arm task execution failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Right arm task executed successfully");

  return;
}

bool MTCTaskNode::doSyncTask(bool attach_object, 
                            std::string arm_group_name, 
                            std::string hand_group_name, 
                            std::string hand_frame,
                            std::mutex& joint_positions_mutex,
                            std::vector<double>& joint_positions,
                            std::vector<std::string>& joint_names,
                            std::condition_variable& joint_positions_condition_variable
                            )
{
  RCLCPP_INFO(LOGGER, "Waiting for joint position via UDP...");
  {
    std::unique_lock<std::mutex> lock(joint_positions_mutex);
    joint_positions_condition_variable.wait(lock, [&joint_positions]() {
      return !std::all_of(joint_positions.begin(), joint_positions.end(), [](double v) { return v == 0.0; });
    });
  }

  RCLCPP_INFO(LOGGER, "Joint position received: [%f, %f, %f, %f, %f, %f, %f]", joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3], joint_positions[4], joint_positions[5], joint_positions[6]);

  mtc::Task sync_task = createGoalJointTask(attach_object, arm_group_name, hand_group_name, hand_frame,
                                          joint_positions_mutex, 
                                          joint_positions, joint_names
                                          );
  try
  {
    sync_task.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Arm sync task initialization failed: " << e.what());
    return false;
  }

  if (!sync_task.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Arm sync task planning failed");
    return false;
  }
  
  // Do not publish the solution for synchronization task
  // sync_task.introspection().publishSolution(*sync_task.solutions().front());
  auto sync_result = sync_task.execute(*sync_task.solutions().front());
  if (sync_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Arm sync task execution failed");
    return false;
  }
  RCLCPP_INFO(LOGGER, "Arm sync task executed successfully");

  return true;
}

void MTCTaskNode::doMountingTask()
{
  /* synchronize all arms to current position in the real world */
  // RCLCPP_INFO(LOGGER, "Synchronizing the left arm to real-world position...");
  // doSyncTask(true, left_arm_group_name, left_hand_group_name, left_hand_frame,
  //             left_joint_positions_mutex, 
  //             left_joint_positions, left_ur_joint_names,
  //             left_joint_positions_condition_variable
  //             );

  RCLCPP_INFO(LOGGER, "Synchronizing the right arm to real-world position...");
  doSyncTask(false, right_arm_group_name, right_hand_group_name, right_hand_frame,
             right_joint_positions_mutex, 
             right_joint_positions, right_franka_joint_names,
             right_joint_positions_condition_variable
             );

  RCLCPP_INFO(LOGGER, "Synchronizing the mount arm to real-world position...");
  doSyncTask(false, mount_group_name, mount_hand_group_name, mount_hand_frame,
             mount_joint_positions_mutex, 
             mount_joint_positions, mount_franka_joint_names,
             mount_joint_positions_condition_variable
             );

  /* planning and execution for right arm joint position to desired clip pose */
  RCLCPP_INFO(LOGGER, "Waiting for right arm goal pose via UDP...");
  {
    std::unique_lock<std::mutex> lock(right_fix_goal_pose_mutex);
    right_fix_goal_pose_condition_variable.wait(lock, [&]() {
      return !std::all_of(right_fix_goal_pose.begin(), right_fix_goal_pose.end(), [](double v) { return v == 0.0; });
    });
  }

  geometry_msgs::msg::PoseStamped right_pose_in_right_frame;
  right_pose_in_right_frame.header.frame_id = "right_panda_link0";
  right_pose_in_right_frame.pose.position.x = right_fix_goal_pose[0];
  right_pose_in_right_frame.pose.position.y = right_fix_goal_pose[1];
  right_pose_in_right_frame.pose.position.z = right_fix_goal_pose[2];
  right_pose_in_right_frame.pose.orientation.w = right_fix_goal_pose[3];
  right_pose_in_right_frame.pose.orientation.x = right_fix_goal_pose[4];
  right_pose_in_right_frame.pose.orientation.y = right_fix_goal_pose[5];
  right_pose_in_right_frame.pose.orientation.z = right_fix_goal_pose[6];

  geometry_msgs::msg::PoseStamped right_pose_in_world_frame = getPoseTransform(right_pose_in_right_frame, "world");

  RCLCPP_INFO(LOGGER, "Received right arm goal pose: [%f, %f, %f, %f, %f, %f, %f]", right_pose_in_world_frame.pose.position.x, right_pose_in_world_frame.pose.position.y, right_pose_in_world_frame.pose.position.z, right_pose_in_world_frame.pose.orientation.w, right_pose_in_world_frame.pose.orientation.x, right_pose_in_world_frame.pose.orientation.y, right_pose_in_world_frame.pose.orientation.z);

  auto right_task = createGoalPoseTask(right_arm_group_name, right_hand_group_name, right_hand_frame, right_pose_in_world_frame);
  // auto right_task = createGoalPoseTask(mount_group_name, mount_hand_group_name, mount_hand_frame, right_pose_in_world_frame);

  // {
  //   std::unique_lock<std::mutex> lock(right_joint_positions_mutex);
  //   right_joint_positions_condition_variable.wait(lock, []() {
  //     return !std::all_of(right_joint_positions.begin(), right_joint_positions.end(), [](double v) { return v == 0.0; });
  //   });
  // }

  // auto right_task = createGoalJointTask(false, right_arm_group_name, right_hand_group_name, right_hand_frame,
  //                                         right_joint_positions_mutex, 
  //                                         right_joint_positions, right_franka_joint_names
  //                                       );
  
  try
  {
    right_task.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Right arm move to pose task initialization failed: " << e);
    return;
  }

  if (!right_task.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Right arm move to pose task planning failed");
    return;
  }

  right_task.introspection().publishSolution(*right_task.solutions().front());

  auto right_result = right_task.execute(*right_task.solutions().front());
  if (right_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Right arm move to pose task execution failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Right arm move to pose task executed successfully");
  move_to_goal_pose_task_success = true;

  /*********** planning and execution for mount arm to desired clip pose ***********/
  RCLCPP_INFO(LOGGER, "Waiting for mount arm goal pose via UDP...");
  {
    std::unique_lock<std::mutex> lock(mount_fix_goal_pose_mutex);
    mount_fix_goal_pose_condition_variable.wait(lock, []() {
      return !std::all_of(mount_fix_goal_pose.begin(), mount_fix_goal_pose.end(), [](double v) { return v == 0.0; });
    });
  }

  geometry_msgs::msg::PoseStamped mount_pose_in_mount_frame;
  mount_pose_in_mount_frame.header.frame_id = "right_panda_link0";  // Recieved goal is in right arm base frame
  mount_pose_in_mount_frame.pose.position.x = mount_fix_goal_pose[0];
  mount_pose_in_mount_frame.pose.position.y = mount_fix_goal_pose[1];
  mount_pose_in_mount_frame.pose.position.z = mount_fix_goal_pose[2];
  mount_pose_in_mount_frame.pose.orientation.w = mount_fix_goal_pose[3];
  mount_pose_in_mount_frame.pose.orientation.x = mount_fix_goal_pose[4];
  mount_pose_in_mount_frame.pose.orientation.y = mount_fix_goal_pose[5];
  mount_pose_in_mount_frame.pose.orientation.z = mount_fix_goal_pose[6];

  geometry_msgs::msg::PoseStamped mount_pose_in_world_frame = getPoseTransform(mount_pose_in_mount_frame, "world");
  
  // auto mount_task = createGoalPoseTask(right_arm_group_name, right_hand_group_name, right_hand_frame, mount_pose_in_world_frame);
  auto mount_task = createGoalPoseTask(mount_group_name, mount_hand_group_name, mount_hand_frame, mount_pose_in_world_frame);
  try
  {
    mount_task.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Mount arm move to pose task initialization failed: " << e);
    return;
  }

  if (!mount_task.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Mount arm move to pose task planning failed");
    return;
  }

  mount_task.introspection().publishSolution(*mount_task.solutions().front());

  auto mount_result = mount_task.execute(*mount_task.solutions().front());
  if (mount_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Mount arm move to pose task execution failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Mount arm move to pose task executed successfully");
  

}

tf2::Matrix3x3 MTCTaskNode::rightGraspOrientation(tf2::Vector3 cable_tangent, tf2::Matrix3x3 left_object_rot)
{
// 'cable_tangent': tf2::Vector3, the rope tangent at the grasp
// 'left_object_rot': tf2::Matrix3x3, the old orientation
// We want new orientation s.t. x-axis = cable_tangent.

tf2::Vector3 x_new = cable_tangent.normalized(); // new X = rope tangent

// Step 2: choose an old axis to preserve as much as possible
tf2::Vector3 z_old = left_object_rot.getColumn(2); // old Z

// Step 3: project z_old so it's perpendicular to x_new
tf2::Vector3 z_new_raw = z_old - (z_old.dot(x_new))*x_new;
double length_z_new = z_new_raw.length();
if (length_z_new < 1e-9)
{
  // fallback if z_old is parallel to x_new
  tf2::Vector3 fallback(0,0,1); 
  if (fabs(fallback.dot(x_new)) > 0.99) 
    fallback = tf2::Vector3(0,1,0);
  z_new_raw = fallback - fallback.dot(x_new)*x_new;
  length_z_new = z_new_raw.length();
}
tf2::Vector3 z_new = z_new_raw / length_z_new;

// Step 4: y_new = z_new x x_new (right-handed)
tf2::Vector3 y_new = z_new.cross(x_new);
y_new.normalize();

// Step 5: Build the new orientation
tf2::Matrix3x3 left_object_new(
    x_new.x(), y_new.x(), z_new.x(),
    x_new.y(), y_new.y(), z_new.y(),
    x_new.z(), y_new.z(), z_new.z()
);

// rotation around z axis for 180 degree
// tf2::Matrix3x3 rot_z_180(
//     -1, 0, 0,
//     0, -1, 0,
//     0, 0, 1
// );
// left_object_new = rot_z_180 * left_object_new;

// (Optional) Convert to quaternion
tf2::Quaternion object_q;
left_object_new.getRotation(object_q);

return left_object_new;

}

mtc::Task MTCTaskNode::createLeftArmTask()
{
  mtc::Task task;
  task.stages()->setName("left arm task");
  task.loadRobotModel(node_);

  std::vector<double> delta = {0, 0, 0};
  std::vector<double> orients = {0, 0, 0, 1};


  task.setProperty("left_group", left_arm_group_name);
  task.setProperty("left_eef", left_hand_group_name);
  task.setProperty("left_ik_frame", left_hand_frame);


  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop


  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));
  
  //3 options of the solvers
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  //open the left hand
  auto stage_open_left_hand =
      std::make_unique<mtc::stages::MoveTo>("open left hand", interpolation_planner);
  stage_open_left_hand->setGroup(left_hand_group_name);  
  stage_open_left_hand->setGoal("gripper_open");  
  task.add(std::move(stage_open_left_hand)); 

  // //move to pick the objekt
  // auto stage_move_to_pick_left = std::make_unique<mtc::stages::Connect>(
  //     "move to pick left",
  //     mtc::stages::Connect::GroupPlannerVector{ { left_arm_group_name, sampling_planner } });
  // stage_move_to_pick_left->setTimeout(5.0);
  // stage_move_to_pick_left->properties().configureInitFrom(mtc::Stage::PARENT);
  // task.add(std::move(stage_move_to_pick_left));

  mtc::Stage* attach_object_stage_left =
      nullptr;  // Forward attach_object_stage to place pose generator

  //creates a SerialContainer which contains the stages relevant to the picking action
  {
    auto grasp_left = std::make_unique<mtc::SerialContainer>("pick object left");
    task.properties().exposeTo(grasp_left->properties(), { "left_eef", "left_group", "left_ik_frame" });
    // grasp->properties().configureInitFrom(mtc::Stage::PARENT,
    //                                       { "left_eef", "left_group", "left_ik_frame" });

    grasp_left->properties().set("eef", task.properties().get<std::string>("left_eef"));// Provide standard names for substages
    grasp_left->properties().set("group", task.properties().get<std::string>("left_group"));
    grasp_left->properties().set("ik_frame", task.properties().get<std::string>("left_ik_frame"));

  {
    //allow collision between left gripper and the object
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (left_gripper,object)");
    stage->allowCollisions("object",
                          task.getRobotModel()
                              ->getJointModelGroup(left_hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          true);
    grasp_left->insert(std::move(stage));
  }
  
  {
    //close the left hand
    auto stage = std::make_unique<mtc::stages::MoveTo>("close left hand", interpolation_planner);
    stage->setGroup(left_hand_group_name);
    stage->setGoal("gripper_close");
    grasp_left->insert(std::move(stage));
  }
  //attach the object to the hand
  {
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object left");
  stage->attachObject("object", left_hand_frame);
  attach_object_stage_left = stage.get();
    
    grasp_left->insert(std::move(stage));
  }

    task.add(std::move(grasp_left));
  }

  {
    auto stage_move_to_place_left = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { left_arm_group_name, sampling_planner },
                                                   });

    stage_move_to_place_left->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place_left));
  }
  {
    auto place_left = std::make_unique<mtc::SerialContainer>("place object left");
    task.properties().exposeTo(place_left->properties(), { "left_eef", "left_group", "left_ik_frame" });
    place_left->properties().set("eef", task.properties().get<std::string>("left_eef"));// Provide standard names for substages
    place_left->properties().set("group", task.properties().get<std::string>("left_group"));
    place_left->properties().set("ik_frame", task.properties().get<std::string>("left_ik_frame"));
  {
    // Sample place pose
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose left");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "place_pose_left");
    stage->setObject("object");
    stage->setTargetDelta(delta);
    stage->setTargetOrient(orients);

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = "world";
    target_pose_msg.pose.position.z = 1.645;
    // target_pose_msg.pose.position.x = -0.1;
    target_pose_msg.pose.position.x = 0.0;
    target_pose_msg.pose.position.y = 0.6;

    // target_pose_msg.pose.position.z = 1.104;
    // target_pose_msg.pose.position.x = 0.624;
    // target_pose_msg.pose.position.y = -0.119;
    // target_pose_msg.pose.orientation.w = 1.0;

    Eigen::Quaterniond q_place =
    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *   
    Eigen::AngleAxisd( M_PI / 2, Eigen::Vector3d::UnitY()) *  
    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());  
    target_pose_msg.pose.orientation.x = q_place.x();
    target_pose_msg.pose.orientation.y = q_place.y();
    target_pose_msg.pose.orientation.z = q_place.z();
    target_pose_msg.pose.orientation.w = q_place.w();

    stage->setPose(target_pose_msg);
    stage->setMonitoredStage(attach_object_stage_left);  // Hook into attach_object_stage
    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("place pose left IK", std::move(stage));
    wrapper->setMaxIKSolutions(2);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame("object");
    
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    place_left->insert(std::move(wrapper));
  }

  // {
  //   auto stage_rotate_ee = std::make_unique<mtc::stages::MoveRelative>("rotate EE 180 degrees", sampling_planner);
  //   stage_rotate_ee->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   stage_rotate_ee->setMinMaxDistance(3.1412, 3.1415);  // Exactly 180 degrees (π radians)
  //   stage_rotate_ee->setIKFrame(left_hand_frame);
  //   stage_rotate_ee->properties().set("marker_ns", "left_eef");

  //   // Define rotation axis (rotate around Z-axis of end effector)
  //   geometry_msgs::msg::Vector3Stamped vec;
  //   // vec.header.frame_id = "world";  // Rotate in the EE frame
  //   vec.header.frame_id = left_hand_frame;  // Rotate in the EE frame
  //   vec.vector.z = 1.0;  // Rotate around Z-axis
  //   stage_rotate_ee->setDirection(vec);
  //   // 允许旋转轨迹插值
  //   stage_rotate_ee->setProperty("interpolation", "circular");

  //   // 降低 min_fraction 以防止路径规划失败
  //   stage_rotate_ee->setProperty("min_fraction", 0.01);
  //   place_left->insert(std::move(stage_rotate_ee));
  // }
  
  // {
  //   auto stage = std::make_unique<mtc::stages::MoveTo>("open left hand", interpolation_planner);
  //   stage->setGroup(left_hand_group_name);
  //   stage->setGoal("gripper_open");
  //   place_left->insert(std::move(stage));
  // }
  {
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (left_gripper,object)");
    stage->allowCollisions("object",
                          task.getRobotModel()
                              ->getJointModelGroup(left_hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          false);
    place_left->insert(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object left");
    stage->detachObject("object", left_hand_frame);
    place_left->insert(std::move(stage));
  } 

  // //retreat
  // // {
  // //   auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
  // //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  // //   stage->setMinMaxDistance(0.1, 0.3);
  // //   stage->setIKFrame(left_hand_frame);
  // //   stage->properties().set("marker_ns", "retreat");

  // //   // Set retreat direction
  // //   geometry_msgs::msg::Vector3Stamped vec;
  // //   vec.header.frame_id = "world";
  // //   vec.vector.x = -0.5;
  // //   stage->setDirection(vec);
  // //   place->insert(std::move(stage));
  // // }
    task.add(std::move(place_left));
    RCLCPP_WARN_STREAM(LOGGER, "task created");
  }

  return task;
}

mtc::Task MTCTaskNode::createLeftArmSyncTask(){
   mtc::Task task;
  task.stages()->setName("left arm task");
  task.loadRobotModel(node_);

  std::vector<double> delta = {0, 0, 0};
  std::vector<double> orients = {0, 0, 0, 1};


  task.setProperty("left_group", left_arm_group_name);
  task.setProperty("left_eef", left_hand_group_name);
  task.setProperty("left_ik_frame", left_hand_frame);


  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop


  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));
  
  // 3 options of the solvers
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto joint_interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_interpolation_planner = std::make_shared<mtc::solvers::CartesianPath>();

  //attach the object to the hand
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object left");
    stage->attachObject("object", left_hand_frame);
    task.add(std::move(stage));
  }
  // forbid collision
  {
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (left_gripper,object)");
    stage->allowCollisions("object",
                          task.getRobotModel()
                              ->getJointModelGroup(left_hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          false);
    task.add(std::move(stage));
  }
  // move to synchronization joint position
  {
    // set Goal from joint positions
    std::map<std::string, double> joint_goal;
    {
      std::lock_guard<std::mutex> joint_lock(left_joint_positions_mutex);
      joint_goal = {{"left_shoulder_pan_joint", left_joint_positions[0]},
                    {"left_shoulder_lift_joint", left_joint_positions[1]},
                    {"left_elbow_joint", left_joint_positions[2]},
                    {"left_wrist_1_joint", left_joint_positions[3]},
                    {"left_wrist_2_joint", left_joint_positions[4]},
                    {"left_wrist_3_joint", left_joint_positions[5]}};
    }

    auto stage_move_to_joint = std::make_unique<mtc::stages::MoveTo>("move to synchronization position", joint_interpolation_planner);
    stage_move_to_joint->setGroup(left_arm_group_name);
    stage_move_to_joint->setGoal(joint_goal);
    task.add(std::move(stage_move_to_joint));
  }
  // // further move to synchronization cartesian ee pose
  // {
  //   // set Goal from ee pose
  //   geometry_msgs::msg::PoseStamped ee_goal;
  //   {
  //     std::lock_guard<std::mutex> cartesian_lock(ee_pose_mutex);
  //     ee_goal.header.frame_id = "left_base_link";
  //     ee_goal.pose.position.x = ee_pose[0];
  //     ee_goal.pose.position.y = ee_pose[1];
  //     ee_goal.pose.position.z = ee_pose[2];

  //     // Convert Euler angles (RX, RY, RZ) to quaternion
  //     tf2::Quaternion quaternion;
  //     quaternion.setRPY(ee_pose[3], ee_pose[4], ee_pose[5]);

  //     // Set the orientation of ee_goal
  //     ee_goal.pose.orientation = tf2::toMsg(quaternion);
  //   }

  //   // IK frame at TCP
  //   Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
  //   grasp_frame_transform.translation().z() = 0.197;

  //   auto stage_move_to_cartesian = std::make_unique<mtc::stages::MoveTo>("move to synchronization cartesian pose", cartesian_interpolation_planner);
  //   stage_move_to_cartesian->setGroup(left_arm_group_name);
  //   stage_move_to_cartesian->setGoal(ee_goal);
  //   stage_move_to_cartesian->setIKFrame(grasp_frame_transform, left_hand_frame);
  //   task.add(std::move(stage_move_to_cartesian));
  // }

  // detach the object
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object left");
    stage->detachObject("object", left_hand_frame);
    task.add(std::move(stage));
  } 
  
  return task;
}

mtc::Task MTCTaskNode::createGoalJointTask(bool attach_object, 
                                         std::string arm_group_name, 
                                         std::string hand_group_name, 
                                         std::string hand_frame,
                                         std::mutex& joint_positions_mutex,
                                         std::vector<double>& joint_positions,
                                         std::vector<std::string>& joint_names
                                         )
{
  mtc::Task task;
  task.stages()->setName("synchronization arm task");
  task.loadRobotModel(node_);

  std::vector<double> delta = {0, 0, 0};
  std::vector<double> orients = {0, 0, 0, 1};

  // task.setProperty("left_group", arm_group_name);
  // task.setProperty("left_eef", hand_group_name);
  // task.setProperty("left_ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop


  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));
  
  // 3 options of the solvers
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto joint_interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_interpolation_planner = std::make_shared<mtc::solvers::CartesianPath>();

  //attach the object to the hand
  if (attach_object)
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object ");
    stage->attachObject("object", hand_frame);
    task.add(std::move(stage));
  }
  // forbid collision
  {
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision between gripper and object");
    stage->allowCollisions("object",
                          task.getRobotModel()
                              ->getJointModelGroup(hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          false);
    task.add(std::move(stage));
  }
  // move to synchronization joint position
  {
    // set Goal from joint positions
    std::map<std::string, double> joint_goal;
    {
      if (joint_positions.size() != joint_names.size()) {
          std::cerr << "Error: joint_positions size (" << joint_positions.size() 
                    << ") does not match joint_names size (" << joint_names.size() << ")" << std::endl;
          return task;
      }

      std::lock_guard<std::mutex> joint_lock(joint_positions_mutex);

      for (size_t i = 0; i < joint_names.size(); i++)
      {
        joint_goal[joint_names[i]] = joint_positions[i];
      }

      std::cout << "printing joint goal" << std::endl;
      for (auto const& [key, val] : joint_goal)
      {
        std::cout << key << ": " << val << std::endl;
      }

    }

    // auto stage_move_to_joint = std::make_unique<mtc::stages::MoveTo>("move to synchronization position", joint_interpolation_planner);
    auto stage_move_to_joint = std::make_unique<mtc::stages::MoveTo>("move to synchronization position", sampling_planner);
    stage_move_to_joint->setGroup(arm_group_name);
    stage_move_to_joint->setGoal(joint_goal);
    task.add(std::move(stage_move_to_joint));
  }

   // // further move to synchronization cartesian ee pose
  // {
  //   // set Goal from ee pose
  //   geometry_msgs::msg::PoseStamped ee_goal;
  //   {
  //     std::lock_guard<std::mutex> cartesian_lock(ee_pose_mutex);
  //     ee_goal.header.frame_id = "left_base_link";
  //     ee_goal.pose.position.x = ee_pose[0];
  //     ee_goal.pose.position.y = ee_pose[1];
  //     ee_goal.pose.position.z = ee_pose[2];

  //     // Convert Euler angles (RX, RY, RZ) to quaternion
  //     tf2::Quaternion quaternion;
  //     quaternion.setRPY(ee_pose[3], ee_pose[4], ee_pose[5]);

  //     // Set the orientation of ee_goal
  //     ee_goal.pose.orientation = tf2::toMsg(quaternion);
  //   }

  //   // IK frame at TCP
  //   Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
  //   grasp_frame_transform.translation().z() = 0.197;

  //   auto stage_move_to_cartesian = std::make_unique<mtc::stages::MoveTo>("move to synchronization cartesian pose", cartesian_interpolation_planner);
  //   stage_move_to_cartesian->setGroup(left_arm_group_name);
  //   stage_move_to_cartesian->setGoal(ee_goal);
  //   stage_move_to_cartesian->setIKFrame(grasp_frame_transform, left_hand_frame);
  //   task.add(std::move(stage_move_to_cartesian));
  // }

  // detach the object
  if (attach_object)
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object from arm");
    stage->detachObject("object", hand_frame);
    task.add(std::move(stage));
  } 
  
  return task;
}

mtc::Task MTCTaskNode::createRightArmTask(tf2::Quaternion q2)
{
  mtc::Task task;
  task.stages()->setName("right arm task");
  task.loadRobotModel(node_);

  moveit_msgs::msg::CollisionObject object2;
  object2.id = "object2";
  object2.header.frame_id = "right_panda_link0";
  object2.primitives.resize(1);
  object2.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object2.primitives[0].dimensions = { 0.02, 0.02 };

  geometry_msgs::msg::Pose pose2;
  //  // read the position received from the udp
  {
    std::lock_guard<std::mutex> lock(grasp_point_mutex);
    pose2.position.x = grasp_point[0];
    pose2.position.y = grasp_point[1];
    pose2.position.z = grasp_point[2]; // -0.03
  }

  // pose2.position.x = 0.0;
  // pose2.position.y = 0.6;
  // pose2.position.z = 1.34;
  // pose2.position.x = 0.82;
  // pose2.position.y = -0.04;
  // pose2.position.z = 0.47;
  // tf2::Quaternion q2;
  // q2.setRPY(0, 0, 0);  // Roll (X), Pitch (Y), Yaw (Z)
  // q2.setRPY(grasp_point[3], grasp_point[4], grasp_point[5]);  // Roll (X), Pitch (Y), Yaw (Z)
  pose2.orientation = tf2::toMsg(q2);
  // pose.orientation.w = 1.0;
  object2.pose = pose2;

  moveit::planning_interface::PlanningSceneInterface psi2;
  psi2.applyCollisionObject(object2);

  std::vector<double> delta = {0, 0, 0};
  std::vector<double> orients = {0, 0, 0, 1};

  task.setProperty("right_group", right_arm_group_name);
  task.setProperty("right_eef", right_hand_group_name);
  task.setProperty("right_ik_frame", right_hand_frame);

  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  auto stage_open_right_hand =
      std::make_unique<mtc::stages::MoveTo>("open right hand", interpolation_planner);
  stage_open_right_hand->setGroup(right_hand_group_name); 
  stage_open_right_hand->setGoal("open");  
  task.add(std::move(stage_open_right_hand));

  auto stage_move_to_pick_right = std::make_unique<mtc::stages::Connect>(
      "move to pick right",
      mtc::stages::Connect::GroupPlannerVector{ { right_arm_group_name, sampling_planner } });
  stage_move_to_pick_right->setTimeout(5.0);
  stage_move_to_pick_right->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick_right));  

  mtc::Stage* attach_object_stage_right =
      nullptr;  // Forward attach_object_stage to place pose generator

  //creates a SerialContainer which contains the stages relevant to the picking action
  {
    auto grasp_right = std::make_unique<mtc::SerialContainer>("pick object right");
    task.properties().exposeTo(grasp_right->properties(), { "right_eef", "right_group", "right_ik_frame" });

    grasp_right->properties().set("eef", task.properties().get<std::string>("right_eef"));// Provide standard names for substages
    grasp_right->properties().set("group", task.properties().get<std::string>("right_group"));
    grasp_right->properties().set("ik_frame", task.properties().get<std::string>("right_ik_frame"));
  //create a stage to approach the object
  {
    auto stage =
        std::make_unique<mtc::stages::MoveRelative>("approach object right", cartesian_planner);
    stage->properties().set("marker_ns", "approach_object_right");
    stage->properties().set("link", right_hand_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.01, 0.015);

    // Set hand forward direction
    geometry_msgs::msg::Vector3Stamped vec_right;
    vec_right.header.frame_id = right_hand_frame;
    vec_right.vector.z = 1.0;
    stage->setDirection(vec_right);
    grasp_right->insert(std::move(stage));
  }
  {
    // Sample grasp pose
    auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose right");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_pose_right");
    stage->properties().set("max_rotation_angle", 1.5*M_PI);
    stage->properties().set("min_rotation_angle", M_PI);
    stage->setPreGraspPose("open");
    stage->setObject("object2");
    stage->setTargetDelta(delta);
    stage->setTargetOrient(orients);
    stage->setAngleDelta(M_PI / 12);
    stage->setRotationAxis(Eigen::Vector3d::UnitZ());
    stage->setMonitoredStage(current_state_ptr);  // Hook into current state
    Eigen::Isometry3d grasp_right_frame_transform;
    // Eigen::Isometry3d grasp_right_frame_transform = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    grasp_right_frame_transform.linear() = q.matrix();
    grasp_right_frame_transform.translation().z() = 0.1034;
    // grasp_right_frame_transform.translation().y() = 0.05;//Grasp one end of the object so that it is easier to hand it over to another arm

    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("grasp pose right IK", std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(grasp_right_frame_transform, right_hand_frame);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    grasp_right->insert(std::move(wrapper));
  }
  
  {
    //allow collision between right hand and the object
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (right_hand,object2)");
    stage->allowCollisions("object2",
                          task.getRobotModel()
                              ->getJointModelGroup(right_hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          true);
    grasp_right->insert(std::move(stage));
  }

  {
    //close the right hand
    auto stage = std::make_unique<mtc::stages::MoveTo>("close right hand", interpolation_planner);
    stage->setGroup(right_hand_group_name);
    stage->setGoal("close");
    grasp_right->insert(std::move(stage));
  }
  //attach the object to the hand
  {
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object2 right");
  stage->attachObject("object2", right_hand_frame);
  attach_object_stage_right = stage.get();
    
    grasp_right->insert(std::move(stage));
  }

    task.add(std::move(grasp_right));
  }
  return task;
}

mtc::Task MTCTaskNode::createGoalPoseTask(std::string arm_group_name, 
                                          std::string hand_group_name, 
                                          std::string hand_frame,
                                          geometry_msgs::msg::PoseStamped goal_pose, 
                                          )
{
  mtc::Task task;
  task.stages()->setName("move to goal pose task");
  task.loadRobotModel(node_);

  std::vector<double> delta = {0, 0, 0};
  std::vector<double> orients = {0, 0, 0, 1};

  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));  

  //creates a SerialContainer which contains the stages relevant to the picking action
  {
    auto grasp_container = std::make_unique<mtc::SerialContainer>("move to goal pose");
    // task.properties().exposeTo(grasp_container->properties(), { "right_eef", "right_group", "right_ik_frame" });

    grasp_container->properties().set("eef", arm_group_name);// Provide standard names for substages
    grasp_container->properties().set("group", hand_group_name);
    grasp_container->properties().set("ik_frame", hand_frame);
  {
  // Fixed grasp pose
    auto stage = std::make_unique<mtc::stages::FixedCartesianPoses>("fixed clipping pose");
    stage->addPose(goal_pose);
    stage->setMonitoredStage(current_state_ptr);

    // IK frame at TCP
    Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
    grasp_frame_transform.translation().z() = 0.1034;

    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("clipping pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(grasp_frame_transform, hand_frame);
    // wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->setGroup(arm_group_name);
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose", "eef"});
    grasp_container->insert(std::move(wrapper));
  }

  task.add(std::move(grasp_container));
  }

  return task;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  
  //start the UDP receiver thread
  std::thread udp_thread_right(udpReceiverRight, "192.168.1.7", 5060);
  std::thread udp_thread_left_sync(udpReceiverSync,  "192.168.1.7", 5070,
                              std::ref(left_joint_positions),
                              std::ref(left_joint_positions_mutex),
                              std::ref(left_joint_positions_condition_variable),
                              std::ref(left_ee_pose),
                              std::ref(left_ee_pose_mutex)
                              );

  std::thread udp_thread_right_sync(udpReceiverSync,  "192.168.1.7", 5080,
                              std::ref(right_joint_positions),
                              std::ref(right_joint_positions_mutex),
                              std::ref(right_joint_positions_condition_variable),
                              std::ref(right_ee_pose),
                              std::ref(right_ee_pose_mutex)
                              );
  
  std::thread udp_thread_mount_sync(udpReceiverSync,  "192.168.1.7", 5081,
                              std::ref(mount_joint_positions),
                              std::ref(mount_joint_positions_mutex),
                              std::ref(mount_joint_positions_condition_variable),
                              std::ref(mount_ee_pose),
                              std::ref(mount_ee_pose_mutex)
                              );

  std::thread udp_thread_right_goal_pose(udpReceiverGoalPose,  "192.168.1.7", 5085,
                              std::ref(right_fix_goal_pose),
                              std::ref(right_fix_goal_pose_mutex),
                              std::ref(right_fix_goal_pose_condition_variable)
                              );

  std::thread udp_thread_mount_goal_pose(udpReceiverGoalPose,  "192.168.1.7", 5086,
                              std::ref(mount_fix_goal_pose),
                              std::ref(mount_fix_goal_pose_mutex),
                              std::ref(mount_fix_goal_pose_condition_variable)
                              );                            

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();

  if (!mtc_task_node->getSkipFlag())
  {
    mtc_task_node->doTask();
  }else{
    RCLCPP_WARN_STREAM(LOGGER, "Skip the handover task");
  }

  mtc_task_node->doMountingTask();

  // // stop the UDP receiver thread
  udp_running = false;
  udp_condition_variable.notify_all();
  if (udp_thread_right.joinable()) {
    udp_thread_right.join();
  }

  left_joint_positions_condition_variable.notify_all();
  if (udp_thread_left_sync.joinable()) {
    udp_thread_left_sync.join();
  }

  right_joint_positions_condition_variable.notify_all();
  if (udp_thread_right_sync.joinable()) {
    udp_thread_right_sync.join();
  }

  mount_joint_positions_condition_variable.notify_all();
  if (udp_thread_mount_sync.joinable()) {
    udp_thread_mount_sync.join();
  }

  right_fix_goal_pose_condition_variable.notify_all();
  if (udp_thread_right_goal_pose.joinable()) {
    udp_thread_right_goal_pose.join();
  }

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}