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

#include <thread>
#include <boost/asio.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include <condition_variable>
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <moveit/move_group_interface/move_group_interface.h>

//initiate the udp server
using boost::asio::ip::udp;
using json = nlohmann::json;

std::atomic<bool> udp_running(true);
std::mutex grasp_point_mutex;
std::mutex left_joint_positions_mutex;
std::mutex left_ee_pose_mutex;
std::condition_variable udp_condition_variable;
std::condition_variable left_joint_positions_condition_variable;
// std::array<double, 3> grasp_point = {0.0, 0.0, 0.0}; // default value
std::array<double, 6> grasp_point = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // default value
std::array<double, 3> grasp_tangent = {0.0, 1.0, 0.0}; // default value
// std::array<double, 6> left_joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // default value
std::vector<double> left_joint_positions(6, 0.0);  // Default size 6
std::vector<double> left_ee_pose(6, 0.0); // Default size 6
// std::array<double, 6> ee_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // default value

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_dual_arm_real");
namespace mtc = moveit::task_constructor;

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

void udpReceiverLeft(const std::string& host, int port,
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
      json received_json = json::parse(data);

      // check if the received data is an array
      if (received_json.is_array()) {
        // extract the joint positions
        std::vector<double> current_ee(received_json.begin(), received_json.begin() + 6);
        std::vector<double> current_q(received_json.begin() + 6, received_json.end());

        // update the grasp point
        {
          std::lock_guard<std::mutex> joint_lock(joint_positions_mutex);
          joint_positions = current_q;
          // for (size_t i = 0; i < joint_positions.size() && i < current_q.size(); ++i) {
          //   joint_positions[i] = current_q[i];
          // }
          std::lock_guard<std::mutex> cartesian_lock(ee_pose_mutex);
          ee_pose = current_ee;
          // for (size_t i = 0; i < ee_pose.size() && i < current_ee.size(); ++i) {
          //   ee_pose[i] = current_ee[i];
          // }
        }
        joint_positions_condition_variable.notify_one();

        // std::cout << "Received left arm tcp position: " << current_ee << std::endl;
        // std::cout << "Received left arm joint position: " << current_q << std::endl;
      } else {
        std::cerr << "Received data is not an array" << std::endl;
      }

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

  void setupPlanningScene();

  // void udpReceive();

  // void updateObjectPose(const geometry_msgs::msg::Pose& new_pose);

private:
  // Compose an MTC task from a series of stages.
  // mtc::Task createTask();
  mtc::Task createLeftArmTask();
  mtc::Task createLeftArmSyncTask();
  mtc::Task createRightArmTask(tf2::Quaternion q2);
  tf2::Matrix3x3 rightGraspOrientation(tf2::Vector3 cable_tangent, tf2::Matrix3x3 left_object_rot);
  mtc::Task task_;
  // // rclcpp::Node::SharedPtr node_;

  // std::thread udp_thread_;
  // std::atomic<bool> stop_thread_{false};  // Thread stop flag
  // geometry_msgs::msg::Pose received_pose_;
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface left_move_group_;

};



MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) },
    left_move_group_(node_, "left_ur_manipulator")  // for dual arm or single arm
{
  // udp_thread_ = std::thread(&MTCTaskNode::udpReceive, this);  // Start the thread
  // RCLCPP_INFO(node_->get_logger(), "UDP receive thread started.");
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

// MTCTaskNode::~MTCTaskNode()
// {
//   stop_thread_ = true;  // set the flag for stopping the thread
//   if (udp_thread_.joinable()) {
//     udp_thread_.join();  // wait until safety
//     RCLCPP_INFO(node_->get_logger(), "UDP receive thread stopped.");
//   }
// }


// void MTCTaskNode::udpReceive() {
//   using boost::asio::ip::udp;

//   boost::asio::io_context io_context;
//   udp::socket socket(io_context, udp::endpoint(udp::v4(), 12345));  // 12345 is the port number
//   char recv_buffer[1024];

//   while (rclcpp::ok()) {
//     udp::endpoint sender_endpoint;
//     size_t len = socket.receive_from(boost::asio::buffer(recv_buffer), sender_endpoint);

//     // Parse the received coordinate data
//     float x, y, z;
//     std::istringstream iss(std::string(recv_buffer, len));
//     iss >> x >> y >> z;

//     geometry_msgs::msg::Pose new_pose;
//     new_pose.position.x = x;
//     new_pose.position.y = y;
//     new_pose.position.z = z;
//     new_pose.orientation.w = 1.0;  

//     updateObjectPose(new_pose);
//     RCLCPP_INFO(node_->get_logger(), "UDP receive loop exited.");
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
  // Note: this step is necessary if the left arm is moved manually to a certain position
  RCLCPP_INFO(LOGGER, "Waiting for joint positions via UDP...");
  {
    std::unique_lock<std::mutex> lock(left_joint_positions_mutex);
    left_joint_positions_condition_variable.wait(lock, []() {
      return !std::all_of(left_joint_positions.begin(), left_joint_positions.end(), [](double v) { return v == 0.0; });
    });
  }
  RCLCPP_INFO(LOGGER, "UR joint position received: [%f, %f, %f, %f, %f, %f]", left_joint_positions[0], left_joint_positions[1], left_joint_positions[2], left_joint_positions[3], left_joint_positions[4], left_joint_positions[5]);

  mtc::Task left_sync_task = createLeftArmSyncTask();
  try
  {
    left_sync_task.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Left arm sync task initialization failed: " << e.what());
    return;
  }

  if (!left_sync_task.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Left arm sync task planning failed");
    return;
  }

  // left_sync_task.introspection().publishSolution(*left_sync_task.solutions().front());

  auto left_sync_result = left_sync_task.execute(*left_sync_task.solutions().front());
  if (left_sync_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Left arm sync task execution failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Left arm sync task executed successfully");

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
  tf2::Matrix3x3 right_object_rot = rightGraspOrientation(cable_tangent, left_object_rot);
  tf2::Quaternion right_object_quat;
  right_object_rot.getRotation(right_object_quat);

  mtc::Task right_task = createRightArmTask(right_object_quat);  
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

  auto right_result = right_task.execute(*right_task.solutions().front());
  if (right_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Right arm task execution failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Right arm task executed successfully");

  return;
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

  const auto& left_arm_group_name = "left_ur_manipulator";
  const auto& left_hand_group_name = "left_gripper";
  const auto& left_hand_frame = "left_robotiq_85_base_link";
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
  // //create a stage to approach the object
  // {
  //   auto stage =
  //       std::make_unique<mtc::stages::MoveRelative>("approach object left", cartesian_planner);
  //   stage->properties().set("marker_ns", "approach_object_left");
  //   stage->properties().set("link", left_hand_frame);
  //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   stage->setMinMaxDistance(0.01, 0.015);

  //   // Set hand forward direction
  //   geometry_msgs::msg::Vector3Stamped vec;
  //   vec.header.frame_id = left_hand_frame;
  //   vec.vector.z = 1.0;
  //   stage->setDirection(vec);
  //   grasp_left->insert(std::move(stage));
  // }
  // RCLCPP_INFO_STREAM(LOGGER, "generate grasp pose");
  // {
  //   // Sample grasp pose
  //   auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose left");
  //   stage->properties().configureInitFrom(mtc::Stage::PARENT);
  //   stage->properties().set("marker_ns", "grasp_pose_left");
  //   stage->setPreGraspPose("gripper_open");
  //   stage->setTargetDelta(delta);
  //   stage->setTargetOrient(orients);
  //   stage->setObject("object");
  //   stage->setAngleDelta(M_PI / 12);
  //   stage->setMonitoredStage(current_state_ptr);  // Hook into current state
  //   Eigen::Isometry3d grasp_frame_transform;
  //   Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
  //                         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
  //                         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  //   grasp_frame_transform.linear() = q.matrix();
  //   grasp_frame_transform.translation().z() = 0.15;
  //   grasp_frame_transform.translation().y() = 0.0;//Grasp one end of the object so that it is easier to hand it over to another arm
  //   RCLCPP_INFO_STREAM(LOGGER, "generated grasp pose");

  //   // Compute IK
  //   auto wrapper =
  //       std::make_unique<mtc::stages::ComputeIK>("grasp pose left IK", std::move(stage));
  //   wrapper->setMaxIKSolutions(2);
  //   wrapper->setMinSolutionDistance(1.0);
  //   wrapper->setIKFrame(grasp_frame_transform, left_hand_frame);
  //   wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  //   wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  //   grasp_left->insert(std::move(wrapper));
  // }
  //   RCLCPP_INFO_STREAM(LOGGER, "IK computed");
  
  

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

  // {
  //   //lift the object
  //   auto stage =
  //       std::make_unique<mtc::stages::MoveRelative>("lift object left", cartesian_planner);
  //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   stage->setMinMaxDistance(0.005, 0.01);
  //   stage->setIKFrame(left_hand_frame);
  //   stage->properties().set("marker_ns", "lift_object_left");

  //   // Set upward direction
  //   geometry_msgs::msg::Vector3Stamped vec_left;
  //   vec_left.header.frame_id = "world";
  //   vec_left.vector.z = 2.8;
  //   stage->setDirection(vec_left);
  //   grasp_left->insert(std::move(stage));
  // }
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






  // {
  //   auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
  //   stage->properties().set("group", "left_ur_manipulator");
  //   // stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   stage->setGoal("home");
  //   task.add(std::move(stage));
  // }


  return task;
}

mtc::Task MTCTaskNode::createLeftArmSyncTask(){
   mtc::Task task;
  task.stages()->setName("left arm task");
  task.loadRobotModel(node_);

  const auto& left_arm_group_name = "left_ur_manipulator";
  const auto& left_hand_group_name = "left_gripper";
  const auto& left_hand_frame = "left_robotiq_85_base_link";
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
    pose2.position.z = grasp_point[2]-0.03;
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

  const auto& right_arm_group_name = "right_panda_arm";
  const auto& right_hand_group_name = "right_hand";
  const auto& right_hand_frame = "right_panda_hand";
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
    stage->setPreGraspPose("open");
    stage->setObject("object2");
    stage->setTargetDelta(delta);
    stage->setTargetOrient(orients);
    stage->setAngleDelta(M_PI / 12);
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

  // {
  //   //lift the object
  //   auto stage =
  //       std::make_unique<mtc::stages::MoveRelative>("lift object right", cartesian_planner);
  //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   stage->setMinMaxDistance(0.01, 0.02);
  //   stage->setIKFrame(right_hand_frame);
  //   stage->properties().set("marker_ns", "lift_object_right");

  //   // Set upward direction
  //   geometry_msgs::msg::Vector3Stamped vec_right;
  //   vec_right.header.frame_id = "world";
  //   vec_right.vector.z = 1.0;
  //   stage->setDirection(vec_right);
  //   grasp_right->insert(std::move(stage));
  // }

    task.add(std::move(grasp_right));
  }
  // {
  //   auto stage_move_to_place_right = std::make_unique<mtc::stages::Connect>(
  //       "move to place right",
  //       mtc::stages::Connect::GroupPlannerVector{ { right_arm_group_name, sampling_planner },
  //                                                  });
  //   stage_move_to_place_right->setTimeout(5.0);
  //   stage_move_to_place_right->properties().configureInitFrom(mtc::Stage::PARENT);
  //   task.add(std::move(stage_move_to_place_right));
  // }
  // {
  //   auto place_right = std::make_unique<mtc::SerialContainer>("place object2 right");
  //   task.properties().exposeTo(place_right->properties(), { "right_eef", "right_group", "right_ik_frame" });
  //   place_right->properties().set("eef", task.properties().get<std::string>("right_eef"));// Provide standard names for substages
  //   place_right->properties().set("group", task.properties().get<std::string>("right_group"));
  //   place_right->properties().set("ik_frame", task.properties().get<std::string>("right_ik_frame"));
  // {
  //   // Sample place pose
  //   auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose right");
  //   stage->properties().configureInitFrom(mtc::Stage::PARENT);
  //   stage->properties().set("marker_ns", "place_pose_right");
  //   stage->setObject("object2");
  //   stage->setTargetDelta(delta);
  //   stage->setTargetOrient(orients);

  //   geometry_msgs::msg::PoseStamped target_pose_msg_right;
  //   target_pose_msg_right.header.frame_id = "world";
  //   target_pose_msg_right.pose.position.z = 1.145;
  //   target_pose_msg_right.pose.position.x = 0.2;
  //   target_pose_msg_right.pose.position.y = 0.80;
  //   target_pose_msg_right.pose.orientation.w = 1.0;
  //   // tf2::Quaternion quaternion;
  //   // quaternion.setRPY(-M_PI / 2, 0, 0);
  //   // target_pose_msg_right.pose.orientation.x = quaternion.x();
  //   // target_pose_msg_right.pose.orientation.y = quaternion.y();
  //   // target_pose_msg_right.pose.orientation.z = quaternion.z();
  //   // target_pose_msg_right.pose.orientation.w = quaternion.w();
  //   stage->setPose(target_pose_msg_right);
  //   stage->setMonitoredStage(attach_object_stage_right);  // Hook into attach_object_stage
  //   // Compute IK
  //   auto wrapper =
  //       std::make_unique<mtc::stages::ComputeIK>("place pose right IK", std::move(stage));
  //   wrapper->setMaxIKSolutions(2);
  //   wrapper->setMinSolutionDistance(1.0);
  //   wrapper->setIKFrame("object2");
  //   wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  //   wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  //   place_right->insert(std::move(wrapper));
  // }
  
  // {
  //   auto stage = std::make_unique<mtc::stages::MoveTo>("open right hand", interpolation_planner);
  //   stage->setGroup(right_hand_group_name);
  //   stage->setGoal("open");
  //   place_right->insert(std::move(stage));
  // }
  // {
  //   auto stage =
  //       std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (right_gripper,object)");
  //   stage->allowCollisions("object2",
  //                         task.getRobotModel()
  //                             ->getJointModelGroup(right_hand_group_name)
  //                             ->getLinkModelNamesWithCollisionGeometry(),
  //                         false);
  //   place_right->insert(std::move(stage));
  // }
  // {
  //   auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object right");
  //   stage->detachObject("object2", right_hand_frame);
  //   place_right->insert(std::move(stage));
  // } 
  //   task.add(std::move(place_right));
  // }
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
  std::thread udp_thread_left(udpReceiverLeft,  "192.168.1.7", 5070,
                              std::ref(left_joint_positions),
                              std::ref(left_joint_positions_mutex),
                              std::ref(left_joint_positions_condition_variable),
                              std::ref(left_ee_pose),
                              std::ref(left_ee_pose_mutex)
                              );

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  // // stop the UDP receiver thread
  udp_running = false;
  udp_condition_variable.notify_all();
  if (udp_thread_right.joinable()) {
    udp_thread_right.join();
  }

  left_joint_positions_condition_variable.notify_all();
  if (udp_thread_left.joinable()) {
    udp_thread_left.join();
  }

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}