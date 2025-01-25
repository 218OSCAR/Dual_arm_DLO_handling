





















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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_dual_arm_panda_ur");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};



MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.15, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.3;
  pose.position.y = -1.1;
  pose.position.z = 1.07;
  tf2::Quaternion q;
  q.setRPY(M_PI / 2, 0, 0);  // Roll (X), Pitch (Y), Yaw (Z)
  pose.orientation = tf2::toMsg(q);
  // pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);


  moveit_msgs::msg::CollisionObject object2;
  object2.id = "object2";
  object2.header.frame_id = "world";
  object2.primitives.resize(1);
  object2.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object2.primitives[0].dimensions = { 0.15, 0.02 };

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0;
  pose2.position.y = 0;
  pose2.position.z = 1.1;
  tf2::Quaternion q2;
  q2.setRPY(0, 0, 0);  // Roll (X), Pitch (Y), Yaw (Z)
  pose.orientation = tf2::toMsg(q2);
  // pose.orientation.w = 1.0;
  object2.pose = pose2;

  moveit::planning_interface::PlanningSceneInterface psi2;
  psi2.applyCollisionObject(object2);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

  void updateObjectPose(const geometry_msgs::msg::Pose& new_pose)
  {
      moveit_msgs::msg::CollisionObject object;
      object.id = "object";  // 确保 ID 与原始定义一致
      object.header.frame_id = "world";
      object.primitives.resize(1);
      object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      object.primitives[0].dimensions = { 0.15, 0.02 };

      // 更新位姿
      object.pose = new_pose;

      // 更新到场景
      moveit::planning_interface::PlanningSceneInterface psi;
      psi.applyCollisionObject(object);

      RCLCPP_INFO(LOGGER, "Updated object pose in the planning scene");
  }


mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  // const auto& arm_group_name = "panda_arm";
  // const auto& hand_group_name = "hand";
  // const auto& hand_frame = "panda_hand";

  const auto& left_arm_group_name = "left_ur_manipulator";
  const auto& left_hand_group_name = "left_gripper";
  const auto& left_hand_frame = "left_robotiq_85_base_link";

  const auto& right_arm_group_name = "right_panda_arm";
  const auto& right_hand_group_name = "right_hand";
  const auto& right_hand_frame = "right_panda_hand";

  // Set task properties
  // task.setProperty("group", arm_group_name);
  // task.setProperty("eef", hand_group_name);
  // task.setProperty("ik_frame", hand_frame);

  task.setProperty("left_group", left_arm_group_name);
  task.setProperty("left_eef", left_hand_group_name);
  task.setProperty("left_ik_frame", left_hand_frame);

  task.setProperty("right_group", right_arm_group_name);
  task.setProperty("right_eef", right_hand_group_name);
  task.setProperty("right_ik_frame", right_hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
    // mtc::Stage* left_arm_state_ptr = nullptr;
    // mtc::Stage* right_arm_state_ptr = nullptr;
  #pragma GCC diagnostic pop

  //   // 定义左臂的状态阶段
  // auto left_arm_state = std::make_unique<mtc::stages::CurrentState>("left arm current state");
  // left_arm_state_ptr = left_arm_state.get();
  // task.add(std::move(left_arm_state));

  // // 定义右臂的状态阶段
  // auto right_arm_state = std::make_unique<mtc::stages::CurrentState>("right arm current state");
  // right_arm_state_ptr = right_arm_state.get();
  // task.add(std::move(right_arm_state));


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
  //move to pick the objekt
  auto stage_move_to_pick_left = std::make_unique<mtc::stages::Connect>(
      "move to pick left",
      mtc::stages::Connect::GroupPlannerVector{ { left_arm_group_name, sampling_planner } });
  stage_move_to_pick_left->setTimeout(5.0);
  stage_move_to_pick_left->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick_left));

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
  //create a stage to approach the object
  {
    auto stage =
        std::make_unique<mtc::stages::MoveRelative>("approach object left", cartesian_planner);
    stage->properties().set("marker_ns", "approach_object_left");
    stage->properties().set("link", left_hand_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.1, 0.15);

    // Set hand forward direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = left_hand_frame;
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    grasp_left->insert(std::move(stage));
  }
  {
    // Sample grasp pose
    auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose left");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_pose_left");
    stage->setPreGraspPose("gripper_open");
    stage->setObject("object");
    stage->setAngleDelta(M_PI / 12);
    stage->setMonitoredStage(current_state_ptr);  // Hook into current state
    Eigen::Isometry3d grasp_frame_transform;
    Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    grasp_frame_transform.linear() = q.matrix();
    grasp_frame_transform.translation().z() = 0.15;
    grasp_frame_transform.translation().y() = 0.05;//Grasp one end of the object so that it is easier to hand it over to another arm

    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("grasp pose left IK", std::move(stage));
    wrapper->setMaxIKSolutions(2);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(grasp_frame_transform, left_hand_frame);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    grasp_left->insert(std::move(wrapper));
  }
  
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

  {
    //lift the object
    auto stage =
        std::make_unique<mtc::stages::MoveRelative>("lift object left", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.1, 0.3);
    stage->setIKFrame(left_hand_frame);
    stage->properties().set("marker_ns", "lift_object_left");

    // Set upward direction
    geometry_msgs::msg::Vector3Stamped vec_left;
    vec_left.header.frame_id = "world";
    vec_left.vector.z = 2.8;
    stage->setDirection(vec_left);
    grasp_left->insert(std::move(stage));
  }
    task.add(std::move(grasp_left));
  }
  {
    auto stage_move_to_place_left = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { left_arm_group_name, sampling_planner },
                                                   });
        // mtc::stages::Connect::GroupPlannerVector{ { left_arm_group_name, sampling_planner },
        //                                           { left_hand_group_name, sampling_planner } });//wtf? useless group and cause an ERROR
    stage_move_to_place_left->setTimeout(5.0);
    // stage_move_to_place->properties().set("group", "left_ur_manipulator");
    // stage_move_to_place->properties().set("eef", "left_gripper");
    // stage_move_to_place->properties().set("ik_frame", "left_robotiq_85_base_link");
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

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = "world";
    target_pose_msg.pose.position.z = 1.5;
    target_pose_msg.pose.position.x = 0;
    target_pose_msg.pose.position.y = 0;
    target_pose_msg.pose.orientation.w = 1.0;
    // tf2::Quaternion quaternion;
    // quaternion.setRPY(-M_PI / 2, 0, 0);
    // target_pose_msg.pose.orientation.x = quaternion.x();
    // target_pose_msg.pose.orientation.y = quaternion.y();
    // target_pose_msg.pose.orientation.z = quaternion.z();
    // target_pose_msg.pose.orientation.w = quaternion.w();
    stage->setPose(target_pose_msg);
    stage->setMonitoredStage(attach_object_stage_left);  // Hook into attach_object_stage
    // updateObjectPose(target_pose_msg.pose);
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
  
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open left hand", interpolation_planner);
    stage->setGroup(left_hand_group_name);
    stage->setGoal("gripper_open");
    place_left->insert(std::move(stage));
  }
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
  }


  // geometry_msgs::msg::PoseStamped left_place_pose;
  //     left_place_pose.header.frame_id = "world";
  //     left_place_pose.pose.position.z = 1.5;
  //     left_place_pose.pose.position.x = 0;
  //     left_place_pose.pose.position.y = 0;
  //     left_place_pose.pose.orientation.w = 1.0;
  //   updateObjectPose(left_place_pose.pose);


  // {
  //   auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
  //   stage->properties().set("group", "left_ur_manipulator");
  //   // stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   stage->setGoal("home");
  //   task.add(std::move(stage));
  // }

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
    stage->setMinMaxDistance(0.1, 0.15);

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
    stage->setAngleDelta(M_PI / 12);
    stage->setMonitoredStage(current_state_ptr);  // Hook into current state
    Eigen::Isometry3d grasp_right_frame_transform;
    Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
    grasp_right_frame_transform.linear() = q.matrix();
    grasp_right_frame_transform.translation().z() = 0.1;
    // grasp_right_frame_transform.translation().x() = -0.1;
    // grasp_frame_transform.translation().y() = -0.05;//Grasp one end of the object so that it is easier to hand it over to another arm

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
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object right");
  stage->attachObject("object2", right_hand_frame);
  attach_object_stage_right = stage.get();
    
    grasp_right->insert(std::move(stage));
  }

  {
    //lift the object
    auto stage =
        std::make_unique<mtc::stages::MoveRelative>("lift object right", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.01, 0.08);
    stage->setIKFrame(right_hand_frame);
    stage->properties().set("marker_ns", "lift_object_right");

    // Set upward direction
    geometry_msgs::msg::Vector3Stamped vec_right;
    vec_right.header.frame_id = "world";
    vec_right.vector.z = 1.0;
    stage->setDirection(vec_right);
    grasp_right->insert(std::move(stage));
  }

    task.add(std::move(grasp_right));
  }
  {
    auto stage_move_to_place_right = std::make_unique<mtc::stages::Connect>(
        "move to place right",
        mtc::stages::Connect::GroupPlannerVector{ { right_arm_group_name, sampling_planner },
                                                   });
    stage_move_to_place_right->setTimeout(5.0);
    stage_move_to_place_right->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place_right));
  }
  {
    auto place_right = std::make_unique<mtc::SerialContainer>("place object right");
    task.properties().exposeTo(place_right->properties(), { "right_eef", "right_group", "right_ik_frame" });
    place_right->properties().set("eef", task.properties().get<std::string>("right_eef"));// Provide standard names for substages
    place_right->properties().set("group", task.properties().get<std::string>("right_group"));
    place_right->properties().set("ik_frame", task.properties().get<std::string>("right_ik_frame"));
  {
    // Sample place pose
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose right");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "place_pose_right");
    stage->setObject("object2");

    geometry_msgs::msg::PoseStamped target_pose_msg_right;
    target_pose_msg_right.header.frame_id = "world";
    target_pose_msg_right.pose.position.z = 1.5;
    target_pose_msg_right.pose.position.x = 0;
    target_pose_msg_right.pose.position.y = 1.0;
    target_pose_msg_right.pose.orientation.w = 1.0;
    // tf2::Quaternion quaternion;
    // quaternion.setRPY(-M_PI / 2, 0, 0);
    // target_pose_msg.pose.orientation.x = quaternion.x();
    // target_pose_msg.pose.orientation.y = quaternion.y();
    // target_pose_msg.pose.orientation.z = quaternion.z();
    // target_pose_msg.pose.orientation.w = quaternion.w();
    stage->setPose(target_pose_msg_right);
    stage->setMonitoredStage(attach_object_stage_right);  // Hook into attach_object_stage
    // updateObjectPose(target_pose_msg.pose);
    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("place pose right IK", std::move(stage));
    wrapper->setMaxIKSolutions(2);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame("object2");
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    place_right->insert(std::move(wrapper));
  }
  
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open right hand", interpolation_planner);
    stage->setGroup(right_hand_group_name);
    stage->setGoal("open");
    place_right->insert(std::move(stage));
  }
  {
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (right_gripper,object)");
    stage->allowCollisions("object2",
                          task.getRobotModel()
                              ->getJointModelGroup(right_hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          false);
    place_right->insert(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object right");
    stage->detachObject("object2", right_hand_frame);
    place_right->insert(std::move(stage));
  } 
    task.add(std::move(place_right));
  }

  // geometry_msgs::msg::PoseStamped initial_pose;
  //     initial_pose.header.frame_id = "world";
  //     initial_pose.pose.position.x = 0.3;
  //     initial_pose.pose.position.y = -1.1;
  //     initial_pose.pose.position.z = 1.07;
  //     tf2::Quaternion quaternion;
  //     quaternion.setRPY(-M_PI / 2, 0, 0);
  //     initial_pose.pose.orientation.x = quaternion.x();
  //     initial_pose.pose.orientation.y = quaternion.y();
  //     initial_pose.pose.orientation.z = quaternion.z();
  //     initial_pose.pose.orientation.w = quaternion.w();
  // updateObjectPose(initial_pose.pose);
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}