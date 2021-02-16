/*
 * ur10e_collision_free_demo.cpp
 *
 *  Created on: Nov 23, 2020
 *      Author: yik
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <ur10e_collision_free_demo/ur10e_collision_free_demo.h>

static bool plotting = false; /**< Enable plotting */

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

const double OFFSET = 0.005;

const std::string LINK_BOX_NAME = "box";
const std::string LINK_END_EFFECTOR_NAME = "iiwa_link_ee";

static const double LONGEST_VALID_SEGMENT_LENGTH = 0.005;

namespace online_planning_test
{
TestExample::TestExample(const ros::NodeHandle& nh, bool plotting, bool rviz)
                    : Example(plotting, rviz), nh_(nh)
                      {
  // total waypoints
  waypoints_robot_a_.clear();
  waypoints_robot_b_.clear();

  waypoint_pose_a_.resize(6);
  waypoint_pose_b_.resize(6);
                      }
void TestExample::make_circle_waypoints(int direction_, double radious_)
{
  double x_,y_;
  double rotation_x_, rotation_y_, rotation_z_;
  double theta_;
  double offset_;
  x_ = 0;
  y_ = 0;
  rotation_z_ = 0;
  theta_ = 30*DEGREE2RADIAN;
  offset_ = 90*DEGREE2RADIAN;

  rw::math::Transform3D<> tf_small_pulley_to_circle_points_;
  rw::math::Transform3D<> tf_big_pulley_to_circle_points_;
  rw::math::Transform3D<> tf_big_pulley_to_small_pulley_;

  tf_big_pulley_to_small_pulley_ =  Transform3D<> (Vector3D<>(-0.130, 0, -0.01), EAA<>(0,0,0).toRotation3D()); // EAA


  // output is relative to pulley's center point.
  if(direction_ > 0)
  {
    for(int num = 1; num <= 3; num ++)
    {
      x_ = -radious_*sin(theta_*num + offset_);
      y_ = radious_*cos(theta_*num + offset_);
      rotation_y_ = 20*DEGREE2RADIAN*num;
      rotation_z_ = 0*DEGREE2RADIAN*num;
      rotation_x_ = -15*DEGREE2RADIAN*num;

      tf_small_pulley_to_circle_points_ = Transform3D<> (Vector3D<>(x_, y_, 0), RPY<>(rotation_z_,rotation_y_,rotation_x_).toRotation3D()); // RPY
      tf_big_pulley_to_circle_points_ = tf_big_pulley_to_small_pulley_*tf_small_pulley_to_circle_points_;

      waypoint_pose_a_[0] = tf_big_pulley_to_circle_points_.P()[0];
      waypoint_pose_a_[1] = tf_big_pulley_to_circle_points_.P()[1];
      waypoint_pose_a_[2] = tf_big_pulley_to_circle_points_.P()[2];
      waypoint_pose_a_[3] = RPY<> (tf_big_pulley_to_circle_points_.R())[0];
      waypoint_pose_a_[4] = RPY<> (tf_big_pulley_to_circle_points_.R())[1];
      waypoint_pose_a_[5] = RPY<> (tf_big_pulley_to_circle_points_.R())[2];

      waypoints_robot_a_.push_back(waypoint_pose_a_);
    }

  }
  else
  {

    for(int num = 1; num <= 3; num ++)
    {
      x_ = radious_*sin(theta_*num + offset_);
      y_ = radious_*cos(theta_*num + offset_);
      rotation_y_ = -20*DEGREE2RADIAN*num;
      rotation_z_ = 0*DEGREE2RADIAN*num;
      rotation_x_ = -15*DEGREE2RADIAN*num;

      tf_big_pulley_to_circle_points_ = Transform3D<> (Vector3D<>(x_, y_, -0.01), RPY<>(rotation_z_,rotation_y_,rotation_x_).toRotation3D()); // RPY


      waypoint_pose_b_[0] = tf_big_pulley_to_circle_points_.P()[0];
      waypoint_pose_b_[1] = tf_big_pulley_to_circle_points_.P()[1];
      waypoint_pose_b_[2] = tf_big_pulley_to_circle_points_.P()[2];
      waypoint_pose_b_[3] = RPY<> (tf_big_pulley_to_circle_points_.R())[0];
      waypoint_pose_b_[4] = RPY<> (tf_big_pulley_to_circle_points_.R())[1];
      waypoint_pose_b_[5] = RPY<> (tf_big_pulley_to_circle_points_.R())[2];

      waypoints_robot_b_.push_back(waypoint_pose_b_);
    }
  }
}
bool TestExample::run()
{
  using tesseract_planning::CartesianWaypoint;
  using tesseract_planning::CompositeInstruction;
  using tesseract_planning::CompositeInstructionOrder;
  using tesseract_planning::Instruction;
  using tesseract_planning::ManipulatorInfo;
  using tesseract_planning::MoveInstruction;
  using tesseract_planning::PlanInstruction;
  using tesseract_planning::PlanInstructionType;
  using tesseract_planning::ProcessPlanningFuture;
  using tesseract_planning::ProcessPlanningRequest;
  using tesseract_planning::ProcessPlanningServer;
  using tesseract_planning::StateWaypoint;
  using tesseract_planning::Waypoint;
  using tesseract_planning_server::ROSProcessEnvironmentCache;

  /////////////////////////
  /// motion way points ///
  /////////////////////////

  //Robot A
  //in relative to big pulley
  // close to the pulley
  waypoint_pose_a_[0] = -0.130;
  waypoint_pose_a_[1] = 0;
  waypoint_pose_a_[2] = -0.02;
  waypoint_pose_a_[3] = 0*DEGREE2RADIAN;
  waypoint_pose_a_[4] = 0;
  waypoint_pose_a_[5] = 0;

  waypoints_robot_a_.push_back(waypoint_pose_a_);

  // insert belt to pulley groove

  waypoint_pose_a_[0] = -0.15;
  waypoint_pose_a_[1] = 0;
  waypoint_pose_a_[2] = -0.02;
  waypoint_pose_a_[3] = 0*DEGREE2RADIAN;
  waypoint_pose_a_[4] = 0;
  waypoint_pose_a_[5] = 0;

  waypoints_robot_a_.push_back(waypoint_pose_a_);

  double safty_margin_;
  safty_margin_ = 0.0007;
  make_circle_waypoints(1, 0.018 + safty_margin_);


  /////////////
  /// SETUP ///
  /////////////

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Pull ROS params
  std::string urdf_xml_string, srdf_xml_string;
  //  double box_side, box_x, box_y;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
  //  nh_.getParam("box_side", box_side);
  //  nh_.getParam("box_x", box_x);
  //  nh_.getParam("box_y", box_y);

  // Initialize the environment
  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init<OFKTStateSolver>(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor_->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);

  // Set default contact distance
  Command::Ptr cmd_default_dist = std::make_shared<tesseract_environment::ChangeDefaultContactMarginCommand>(0.005);
  if (!monitor_->applyCommand(*cmd_default_dist))
    return false;

  // Create plotting tool
  ROSPlottingPtr plotter = std::make_shared<tesseract_rosutils::ROSPlotting>("world", "ur10e_a");
  if (rviz_)
    plotter->waitForConnection();

  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.push_back("a_shoulder_pan_joint");
  joint_names.push_back("a_shoulder_lift_joint");
  joint_names.push_back("a_elbow_joint");
  joint_names.push_back("a_wrist_1_joint");
  joint_names.push_back("a_wrist_2_joint");
  joint_names.push_back("a_wrist_3_joint");
  //  joint_names.push_back("b_shoulder_pan_joint");
  //  joint_names.push_back("b_shoulder_lift_joint");
  //  joint_names.push_back("b_elbow_joint");
  //  joint_names.push_back("b_wrist_1_joint");
  //  joint_names.push_back("b_wrist_2_joint");
  //  joint_names.push_back("b_wrist_3_joint");
  //joint_names.push_back("a_robotiq_85_right_inner_knuckle_joint");
  //joint_names.push_back("a_robotiq_85_right_finger_tip_joint");

  Eigen::VectorXd joint_pos(6);
  joint_pos(0) = 2.97326;
  joint_pos(1) = -1.6538;
  joint_pos(2) = -2.33488;
  joint_pos(3) = -2.28384;
  joint_pos(4) = -2.53001;
  joint_pos(5) = -3.13221;
  //  joint_pos(6) = 3.87083;
  //  joint_pos(7) = -2.28049;
  //  joint_pos(8) = -1.3507;
  //  joint_pos(9) = -2.64271;
  //  joint_pos(10) = 1.08987;
  //  joint_pos(11) = 3.12731;
  //joint_pos(6) = 0;
  //joint_pos(7) = 0;
  //env_->setState(joint_names, joint_pos);
  //env_->
  // Add simulated box to environment
  //  Command::Ptr cmd = addBox(box_x, box_y, box_side);
  //  if (!monitor_->applyCommand(*cmd))
  //    return false;

  ////////////
  /// PICK ///
  ////////////
  if (rviz_)
    plotter->waitForInput();



  // Create Program b

  std::vector<std::string> joint_names_b;
  joint_names_b.push_back("b_shoulder_pan_joint");
  joint_names_b.push_back("b_shoulder_lift_joint");
  joint_names_b.push_back("b_elbow_joint");
  joint_names_b.push_back("b_wrist_1_joint");
  joint_names_b.push_back("b_wrist_2_joint");
  joint_names_b.push_back("b_wrist_3_joint");
  //  joint_names.push_back("b_shoulder_pan_joint");
  //  joint_names.push_back("b_shoulder_lift_joint");
  //  joint_names.push_back("b_elbow_joint");
  //  joint_names.push_back("b_wrist_1_joint");
  //  joint_names.push_back("b_wrist_2_joint");
  //  joint_names.push_back("b_wrist_3_joint");
  //joint_names.push_back("a_robotiq_85_right_inner_knuckle_joint");
  //joint_names.push_back("a_robotiq_85_right_finger_tip_joint");

  Eigen::VectorXd joint_pos_b(6);
  joint_pos_b(0) =  3.87083;
  joint_pos_b(1) =  -2.28049;
  joint_pos_b(2) =  -1.3507;
  joint_pos_b(3) =  -2.64271;
  joint_pos_b(4) =  1.08987;
  joint_pos_b(5) =  3.12731;

  env_->setState(joint_names, joint_pos);
  env_->setState(joint_names_b, joint_pos_b);

  // Create Program a
  CompositeInstruction pick_program("DEFAULT", CompositeInstructionOrder::ORDERED, ManipulatorInfo("ur10e_a"));

  Waypoint pick_swp = StateWaypoint(joint_names, joint_pos);
  PlanInstruction start_instruction(pick_swp, PlanInstructionType::START);

  pick_program.setStartInstruction(start_instruction);



  //path planning
  // Define the approach pose
  Eigen::Isometry3d temp_pose;
  rw::math::Transform3D<> tf_world_to_a_base_link_;
  rw::math::Transform3D<> tf_a_base_link_to_big_pulley_;
  rw::math::Transform3D<> tf_big_pulley_to_waypoints_;
  rw::math::Transform3D<> tf_world_to_waypoints_;

  tf_world_to_a_base_link_ = Transform3D<> (Vector3D<>(0, 0, 0.05), RPY<> (180*DEGREE2RADIAN,0,0).toRotation3D()); // RPY
  tf_a_base_link_to_big_pulley_ = Transform3D<> (Vector3D<>(-0.739757210413974, 0.07993900394775277, 0.2449995438351456), EAA<>(-0.7217029684216122, -1.7591780460014375, 1.7685571865188172).toRotation3D()); // RPY

  for(int num = 0; num < 2; num ++)
  {

    tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_a_[num][0], waypoints_robot_a_[num][1], waypoints_robot_a_[num][2]), RPY<>(waypoints_robot_a_[num][3],waypoints_robot_a_[num][4],waypoints_robot_a_[num][5]).toRotation3D()); // RPY
    tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_base_link_to_big_pulley_*tf_big_pulley_to_waypoints_;


    temp_pose.linear() = Eigen::Quaterniond(Quaternion<> (tf_world_to_waypoints_.R())[3],Quaternion<> (tf_world_to_waypoints_.R())[0],Quaternion<> (tf_world_to_waypoints_.R())[1], Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
    temp_pose.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

    Waypoint temp_wp = CartesianWaypoint(temp_pose);

    PlanInstruction pick_plan_a1(temp_wp, PlanInstructionType::FREESPACE, "DEFAULT");
    pick_plan_a1.setDescription("pose_1");


    pick_program.push_back(pick_plan_a1);
  }

  tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_a_[1][0], waypoints_robot_a_[1][1], waypoints_robot_a_[1][2]), RPY<>(waypoints_robot_a_[1][3],waypoints_robot_a_[1][4],waypoints_robot_a_[1][5]).toRotation3D()); // RPY
  tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_base_link_to_big_pulley_*tf_big_pulley_to_waypoints_;


  temp_pose.linear() = Eigen::Quaterniond(Quaternion<> (tf_world_to_waypoints_.R())[3],Quaternion<> (tf_world_to_waypoints_.R())[0],Quaternion<> (tf_world_to_waypoints_.R())[1], Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  temp_pose.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

  Waypoint temp_wp_0 = CartesianWaypoint(temp_pose);

  PlanInstruction pick_plan_a0(temp_wp_0, PlanInstructionType::FREESPACE, "DEFAULT");
  pick_plan_a0.setDescription("pose_1");



  tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_a_[2][0], waypoints_robot_a_[2][1], waypoints_robot_a_[2][2]), RPY<>(waypoints_robot_a_[2][3],waypoints_robot_a_[2][4],waypoints_robot_a_[2][5]).toRotation3D()); // RPY
  tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_base_link_to_big_pulley_*tf_big_pulley_to_waypoints_;


  temp_pose.linear() = Eigen::Quaterniond(Quaternion<> (tf_world_to_waypoints_.R())[3],Quaternion<> (tf_world_to_waypoints_.R())[0],Quaternion<> (tf_world_to_waypoints_.R())[1], Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  temp_pose.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

  Waypoint temp_wp_2 = CartesianWaypoint(temp_pose);

  PlanInstruction pick_plan_a2(temp_wp_2, PlanInstructionType::FREESPACE, "DEFAULT");
  pick_plan_a2.setDescription("pose_2");

  tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_a_[3][0], waypoints_robot_a_[3][1], waypoints_robot_a_[3][2]), RPY<>(waypoints_robot_a_[3][3],waypoints_robot_a_[3][4],waypoints_robot_a_[3][5]).toRotation3D()); // RPY
  tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_base_link_to_big_pulley_*tf_big_pulley_to_waypoints_;


  temp_pose.linear() = Eigen::Quaterniond(Quaternion<> (tf_world_to_waypoints_.R())[3],Quaternion<> (tf_world_to_waypoints_.R())[0],Quaternion<> (tf_world_to_waypoints_.R())[1], Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  temp_pose.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

  Waypoint temp_wp_3 = CartesianWaypoint(temp_pose);

  PlanInstruction pick_plan_a3(temp_wp_3, PlanInstructionType::FREESPACE, "DEFAULT");
  pick_plan_a3.setDescription("pose_3");


  tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_a_[4][0], waypoints_robot_a_[4][1], waypoints_robot_a_[4][2]), RPY<>(waypoints_robot_a_[4][3],waypoints_robot_a_[4][4],waypoints_robot_a_[4][5]).toRotation3D()); // RPY
  tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_base_link_to_big_pulley_*tf_big_pulley_to_waypoints_;


  temp_pose.linear() = Eigen::Quaterniond(Quaternion<> (tf_world_to_waypoints_.R())[3],Quaternion<> (tf_world_to_waypoints_.R())[0],Quaternion<> (tf_world_to_waypoints_.R())[1], Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  temp_pose.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

  Waypoint temp_wp_4 = CartesianWaypoint(temp_pose);

  PlanInstruction pick_plan_a4(temp_wp_4, PlanInstructionType::FREESPACE, "DEFAULT");
  pick_plan_a4.setDescription("pose_4");

  pick_program.push_back(pick_plan_a0);
  //pick_program.push_back(pick_plan_a2);
  //pick_program.push_back(pick_plan_a3);
  pick_program.push_back(pick_plan_a4);


  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 10);
  planning_server.loadDefaultProcessPlanners();


  // Create TrajOpt Profile
  auto trajopt_plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
  trajopt_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 10);
//
  ForceConstraint f(env_);
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> temp_function = f;
  //temp_function = force_constraint_func;
  sco::VectorOfVector::func temp_a;
  temp_a = temp_function;
  //temp_a = force_constraint_func;
  //sco::MatrixOfVector::func,
  sco::ConstraintType a = sco::ConstraintType::INEQ;
  Eigen::VectorXd error_coeff(6);
  error_coeff << 0.1,0.1,0.1,0.1,0.1,0.1 ;

  std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd> temp_tuple(temp_a,nullptr,a,error_coeff);


  std::vector<std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd>>
  constraint_error_functions_;

  constraint_error_functions_.push_back(temp_tuple);

  trajopt_plan_profile->constraint_error_functions = constraint_error_functions_;

  auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  //trajopt_composite_profile->collision_constraint_config.type=trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
  trajopt_composite_profile->collision_constraint_config.enabled = false;
  trajopt_composite_profile->collision_cost_config.safety_margin = 0.001;
  trajopt_composite_profile->collision_cost_config.coeff = 50;
  trajopt_composite_profile->constraint_error_functions = constraint_error_functions_;


  auto trajopt_solver_profile = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->opt_info.max_iter = 500;

  // Add profile to Dictionary
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("CARTESIAN", trajopt_plan_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
      trajopt_composite_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
      trajopt_solver_profile);

  //tesseract_planning::createUserDefinedTermInfo(start_index, end_index, error_function, jacobian_function, type)

  ROS_INFO("Pick plan");

  // Create Process Planning Request
  ProcessPlanningRequest pick_request;
  pick_request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  pick_request.instructions = Instruction(pick_program);

  // Print Diagnostics
  pick_request.instructions.print("Program: ");

  // Solve process plan
  ProcessPlanningFuture pick_response = planning_server.run(pick_request);
  planning_server.waitForAll();

  // Plot Process Trajectory
  if (rviz_ && plotter != nullptr && plotter->isConnected())
  {
    plotter->waitForInput();
    const auto* cp = pick_response.results->cast_const<CompositeInstruction>();
    tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(*cp, env_);
    tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(*cp);
    plotter->plotMarker(ToolpathMarker(toolpath), "ur10e_a");
    plotter->plotTrajectory(trajectory, env_->getStateSolver());
  }


  ///// ROBOT B //////

  // Create plotting tool
  ROSPlottingPtr plotter_b = std::make_shared<tesseract_rosutils::ROSPlotting>("world", "ur10e_b");
  if (rviz_)
    plotter_b->waitForConnection();

  ////////////
  /// PICK ///
  ////////////
  if (rviz_)
    plotter_b->waitForInput();


  //Robot A
  //in relative to big pulley
  // close to the pulley
  waypoint_pose_b_[0] = 0;
  waypoint_pose_b_[1] = 0;
  waypoint_pose_b_[2] = -0.02;
  waypoint_pose_b_[3] = 0*DEGREE2RADIAN;
  waypoint_pose_b_[4] = 0;
  waypoint_pose_b_[5] = 0;

  waypoints_robot_b_.push_back(waypoint_pose_b_);

  // insert belt to pulley groove

  waypoint_pose_b_[0] = 0.04;
  waypoint_pose_b_[1] = 0;
  waypoint_pose_b_[2] = -0.02;
  waypoint_pose_b_[3] = 0*DEGREE2RADIAN;
  waypoint_pose_b_[4] = 0;
  waypoint_pose_b_[5] = 0;

  waypoints_robot_b_.push_back(waypoint_pose_b_);

  safty_margin_ = 0.0007;
  make_circle_waypoints(-1, 0.036 + safty_margin_);

  env_->setState(joint_names, env_->getCurrentJointValues(joint_names));
  env_->setState(joint_names_b, joint_pos_b);
  CompositeInstruction pick_program_b("DEFAULT", CompositeInstructionOrder::ORDERED, ManipulatorInfo("ur10e_b"));

  Waypoint pick_swp_b = StateWaypoint(joint_names_b, joint_pos_b);
  PlanInstruction start_instruction_b(pick_swp_b, PlanInstructionType::START);
  pick_program_b.setStartInstruction(start_instruction_b);


  //path planning
  // Define the approach pose
  Eigen::Isometry3d temp_pose_b;

  rw::math::Transform3D<> tf_a_parts;
  rw::math::Transform3D<> tf_b_parts;
  rw::math::Transform3D<> tf_a_b;

  tf_world_to_a_base_link_ = Transform3D<> (Vector3D<>(0, 0, 0.05), RPY<> (180*DEGREE2RADIAN,0,0).toRotation3D()); // RPY

  tf_a_parts =  Transform3D<> (Vector3D<>(-0.739757210413974, 0.07993900394775277, 0.2449995438351456), EAA<>(-0.7217029684216122, -1.7591780460014375, 1.7685571865188172).toRotation3D());
  tf_b_parts =  Transform3D<> (Vector3D<>(-0.6654834316385497, -0.0844570012960042, 0.2551901723057327), EAA<>(-1.484318068681165, 0.6191402790204206, -0.6254296057933952 ).toRotation3D());

  tf_a_b = tf_a_parts * inverse(tf_b_parts);
  //tf_a_base_link_to_big_pulley_ = Transform3D<> (Vector3D<>(-0.739757210413974, 0.07993900394775277, 0.2449995438351456), EAA<>(-0.7217029684216122, -1.7591780460014375, 1.7685571865188172).toRotation3D()); // RPY

  for(int num = 0; num < 2; num ++)
  {

    tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_b_[num][0], waypoints_robot_b_[num][1], waypoints_robot_b_[num][2]), RPY<>(waypoints_robot_b_[num][3],waypoints_robot_b_[num][4],waypoints_robot_b_[num][5]).toRotation3D()); // RPY
    tf_world_to_waypoints_ = (tf_world_to_a_base_link_ * tf_a_b) * tf_b_parts * tf_big_pulley_to_waypoints_;


    temp_pose_b.linear() = Eigen::Quaterniond(Quaternion<> (tf_world_to_waypoints_.R())[3],Quaternion<> (tf_world_to_waypoints_.R())[0],Quaternion<> (tf_world_to_waypoints_.R())[1], Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
    temp_pose_b.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

    Waypoint temp_wp_b = CartesianWaypoint(temp_pose_b);

    PlanInstruction pick_plan_b1(temp_wp_b, PlanInstructionType::FREESPACE, "DEFAULT");
    pick_plan_b1.setDescription("pose_1");


    pick_program_b.push_back(pick_plan_b1);
  }

  tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_b_[2][0], waypoints_robot_b_[2][1], waypoints_robot_b_[2][2]), RPY<>(waypoints_robot_b_[2][3],waypoints_robot_b_[2][4],waypoints_robot_b_[2][5]).toRotation3D()); // RPY
  tf_world_to_waypoints_ = (tf_world_to_a_base_link_ * tf_a_b) * tf_b_parts * tf_big_pulley_to_waypoints_;


  temp_pose_b.linear() = Eigen::Quaterniond(Quaternion<> (tf_world_to_waypoints_.R())[3],Quaternion<> (tf_world_to_waypoints_.R())[0],Quaternion<> (tf_world_to_waypoints_.R())[1], Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  temp_pose_b.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

  Waypoint temp_wp_b_0 = CartesianWaypoint(temp_pose_b);

  PlanInstruction pick_plan_b0(temp_wp_b_0, PlanInstructionType::FREESPACE, "DEFAULT");
  pick_plan_b0.setDescription("pose_2");

  //std::cout << temp_pose_b.linear() << std::endl;
  //std::cout << temp_pose_b.translation() << std::endl;
  //
  //
  //
  tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_b_[3][0], waypoints_robot_b_[3][1], waypoints_robot_b_[3][2]), RPY<>(waypoints_robot_b_[3][3],waypoints_robot_b_[3][4],waypoints_robot_b_[3][5]).toRotation3D()); // RPY
  tf_world_to_waypoints_ = (tf_world_to_a_base_link_ * tf_a_b) * tf_b_parts * tf_big_pulley_to_waypoints_;


  temp_pose_b.linear() = Eigen::Quaterniond(Quaternion<> (tf_world_to_waypoints_.R())[3],Quaternion<> (tf_world_to_waypoints_.R())[0],Quaternion<> (tf_world_to_waypoints_.R())[1], Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  temp_pose_b.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

  Waypoint temp_wp_b2 = CartesianWaypoint(temp_pose_b);

  PlanInstruction pick_plan_b2(temp_wp_b2, PlanInstructionType::FREESPACE, "DEFAULT");
  pick_plan_b2.setDescription("pose_3");

  std::cout << temp_pose_b.linear() << std::endl;
  std::cout << temp_pose_b.translation() << std::endl;
  //
  //
  //
  tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_b_[4][0], waypoints_robot_b_[4][1], waypoints_robot_b_[4][2]), RPY<>(waypoints_robot_b_[4][3],waypoints_robot_b_[4][4],waypoints_robot_b_[4][5]).toRotation3D()); // RPY
  tf_world_to_waypoints_ = (tf_world_to_a_base_link_ * tf_a_b) * tf_b_parts * tf_big_pulley_to_waypoints_;


  temp_pose_b.linear() = Eigen::Quaterniond(Quaternion<> (tf_world_to_waypoints_.R())[3],Quaternion<> (tf_world_to_waypoints_.R())[0],Quaternion<> (tf_world_to_waypoints_.R())[1], Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  temp_pose_b.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

  Waypoint temp_wp_b3 = CartesianWaypoint(temp_pose_b);

  PlanInstruction pick_plan_b3(temp_wp_b3, PlanInstructionType::FREESPACE, "DEFAULT");
  pick_plan_b3.setDescription("pose_4");
  //
  //
  //
  tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_b_[4][0], waypoints_robot_b_[4][1], waypoints_robot_b_[4][2]), RPY<>(waypoints_robot_b_[4][3],waypoints_robot_b_[4][4],waypoints_robot_b_[4][5]).toRotation3D()); // RPY
  tf_world_to_waypoints_ = (tf_world_to_a_base_link_ * tf_a_b) * tf_b_parts * tf_big_pulley_to_waypoints_;


  temp_pose_b.linear() = Eigen::Quaterniond(Quaternion<> (tf_world_to_waypoints_.R())[3],Quaternion<> (tf_world_to_waypoints_.R())[0],Quaternion<> (tf_world_to_waypoints_.R())[1], Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  temp_pose_b.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

  Waypoint temp_wp_b4 = CartesianWaypoint(temp_pose_b);

  PlanInstruction pick_plan_b4(temp_wp_b4, PlanInstructionType::FREESPACE, "DEFAULT");
  pick_plan_b4.setDescription("pose_5");

  pick_program_b.push_back(pick_plan_b0);
  pick_program_b.push_back(pick_plan_b2);
  pick_program_b.push_back(pick_plan_b3);
  pick_program_b.push_back(pick_plan_b4);


  // Create Process Planning Server
  ProcessPlanningServer planning_server_b(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 10);
  planning_server_b.loadDefaultProcessPlanners();


  // Create TrajOpt Profile
  auto trajopt_plan_profile_b = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
  trajopt_plan_profile_b->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 10);

//  ForceConstraint f_b(env_);
//  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> temp_function_b = f_b;
//  //temp_function = force_constraint_func;
//  sco::VectorOfVector::func temp_b;
//  temp_b = temp_function_b;
//  //temp_a = force_constraint_func;
//  //sco::MatrixOfVector::func,
//  sco::ConstraintType b = sco::ConstraintType::INEQ;
//  Eigen::VectorXd error_coeff_b(6);
//  error_coeff_b << 0.1,0.1,0.1,0.1,0.1,0.1 ;
//
//  std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd> temp_tuple_b(temp_b,nullptr,b,error_coeff_b);
//
//
//  std::vector<std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd>>
//  constraint_error_functions_b_;
//
//  constraint_error_functions_b_.push_back(temp_tuple_b);
//
//  trajopt_plan_profile->constraint_error_functions = constraint_error_functions_b_;

  auto trajopt_composite_profile_b = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  //trajopt_composite_profile->collision_constraint_config.type=trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
  trajopt_composite_profile_b->collision_constraint_config.enabled = false;
  trajopt_composite_profile_b->collision_cost_config.safety_margin = 0.001;
  trajopt_composite_profile_b->collision_cost_config.coeff = 50;
//  trajopt_composite_profile_b->constraint_error_functions = constraint_error_functions_b_;


  auto trajopt_solver_profile_b = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
  trajopt_solver_profile_b->opt_info.max_iter = 500;

  // Add profile to Dictionary
  planning_server_b.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("CARTESIAN", trajopt_plan_profile_b);
  planning_server_b.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
      trajopt_composite_profile_b);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
      trajopt_solver_profile_b);

  //tesseract_planning::createUserDefinedTermInfo(start_index, end_index, error_function, jacobian_function, type)

  ROS_INFO("Pick plan");

  // Create Process Planning Request
  ProcessPlanningRequest pick_request_b;
  pick_request_b.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  pick_request_b.instructions = Instruction(pick_program_b);

  // Print Diagnostics
  pick_request_b.instructions.print("Program: ");

  // Solve process plan
  ProcessPlanningFuture pick_response_b = planning_server_b.run(pick_request_b);
  planning_server_b.waitForAll();

  // Plot Process Trajectory
  if (rviz_ && plotter_b != nullptr && plotter_b->isConnected())
  {
    plotter_b->waitForInput();
    const auto* cp_b = pick_response_b.results->cast_const<CompositeInstruction>();
    tesseract_common::Toolpath toolpath_b = tesseract_planning::toToolpath(*cp_b, env_);
    tesseract_common::JointTrajectory trajectory_b = tesseract_planning::toJointTrajectory(*cp_b);
    plotter_b->plotMarker(ToolpathMarker(toolpath_b), "ur10e_b");
    plotter_b->plotTrajectory(trajectory_b, env_->getStateSolver());
  }

  ROS_INFO("Done");
  return true;
}
}  // namespace tesseract_ros_examples

