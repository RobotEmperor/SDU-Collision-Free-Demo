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
void TestExample::parsing_data(const std::string &path_)
{
  Eigen::VectorXd temp_a(6);
  Eigen::VectorXd temp_b(6);

  joint_init_pos_a.resize(6);
  joint_init_pos_b.resize(6);

  YAML::Node doc; //
  try
  {
    // load yaml
    doc = YAML::LoadFile(path_.c_str()); //

  }catch(const std::exception& e) //
  {
    std::cout << "Fail to load yaml file!" << std::endl;
    return;
  }
  YAML::Node robot_a_ref_frame_node = doc["robot_a_ref_frame"];
  YAML::Node robot_a_robot_initial_pose_node = doc["robot_a_robot_initial_pose"];
  YAML::Node robot_a_initial_joint_states_node = doc["robot_a_initial_joint_states"];

  YAML::Node robot_b_ref_frame_node = doc["robot_b_ref_frame"];
  YAML::Node robot_b_robot_initial_pose_node = doc["robot_b_robot_initial_pose"];
  YAML::Node robot_b_initial_joint_states_node = doc["robot_b_initial_joint_states"];

  for(int num = 0; num <6; num ++)
  {
    joint_init_pos_a[num] = robot_a_initial_joint_states_node[num].as<double>();
    joint_init_pos_b[num] = robot_b_initial_joint_states_node[num].as<double>();

    temp_a[num] = robot_a_ref_frame_node[num].as<double>();
    temp_b[num] = robot_b_ref_frame_node[num].as<double>();
  }

  tf_a_parts =  Transform3D<> (Vector3D<>(temp_a[0],temp_a[1],temp_a[2]), EAA<>(temp_a[3],temp_a[4],temp_a[5]).toRotation3D());
  tf_b_parts =  Transform3D<> (Vector3D<>(temp_b[0],temp_b[1],temp_b[2]), EAA<>(temp_b[3],temp_b[4],temp_b[5]).toRotation3D());

  rw::math::Transform3D<> t_a_b;

  t_a_b = tf_a_parts * inverse(tf_b_parts);

  std::cout <<" joint_init_pos_a :: "<< joint_init_pos_a << std::endl;
  std::cout <<" joint_init_pos_b :: "<< joint_init_pos_b << std::endl;

  std::cout << "tf_a_b_parts :: ---------------------------- " << std::endl;
  std::cout << (t_a_b.P())[0] << std::endl;
  std::cout << (t_a_b.P())[1] << std::endl;
  std::cout << (t_a_b.P())[2] << std::endl;

  std::cout << EAA<> (t_a_b.R())[0] << std::endl;
  std::cout << EAA<> (t_a_b.R())[1] << std::endl;
  std::cout << EAA<> (t_a_b.R())[2] << std::endl;

  std::cout << "tf_b_parts :: ---------------------------- " << std::endl;
  std::cout << (tf_b_parts.P())[0] << std::endl;
  std::cout << (tf_b_parts.P())[1] << std::endl;
  std::cout << (tf_b_parts.P())[2] << std::endl;

  std::cout << EAA<> (tf_b_parts.R())[0] << std::endl;
  std::cout << EAA<> (tf_b_parts.R())[1] << std::endl;
  std::cout << EAA<> (tf_b_parts.R())[2] << std::endl;
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
  offset_ = 70*DEGREE2RADIAN;

  rw::math::Transform3D<> tf_small_pulley_to_circle_points_;
  rw::math::Transform3D<> tf_big_pulley_to_circle_points_;
  rw::math::Transform3D<> tf_big_pulley_to_small_pulley_;

  tf_big_pulley_to_small_pulley_ =  Transform3D<> (Vector3D<>(-0.130, 0, 0), EAA<>(0,0,0).toRotation3D()); // EAA


  // output is relative to pulley's center point.
  if(direction_ > 0)
  {
    for(int num = 1; num <= 3; num ++)
    {
      x_ = -radious_*sin(theta_*num + offset_);
      y_ = radious_*cos(theta_*num + offset_);
      rotation_y_ = 25*DEGREE2RADIAN;
      rotation_z_ = 5*DEGREE2RADIAN*num;
      rotation_x_ = -5*DEGREE2RADIAN*num;

      tf_small_pulley_to_circle_points_ = Transform3D<> (Vector3D<>(x_, y_, 0.01), RPY<>(rotation_z_,rotation_y_,rotation_x_).toRotation3D()); // RPY
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
      rotation_y_ = -35*DEGREE2RADIAN;
      rotation_z_ = -15*DEGREE2RADIAN*num;
      rotation_x_ = 0*DEGREE2RADIAN*num;

      tf_big_pulley_to_circle_points_ = Transform3D<> (Vector3D<>(x_, y_, -0.008), RPY<>(rotation_z_,rotation_y_,rotation_x_).toRotation3D()); // RPY


      waypoint_pose_b_[0] = tf_big_pulley_to_circle_points_.P()[0];
      waypoint_pose_b_[1] = tf_big_pulley_to_circle_points_.P()[1];
      waypoint_pose_b_[2] = tf_big_pulley_to_circle_points_.P()[2];
      waypoint_pose_b_[3] = RPY<> (tf_big_pulley_to_circle_points_.R())[0];
      waypoint_pose_b_[4] = RPY<> (tf_big_pulley_to_circle_points_.R())[1];
      waypoint_pose_b_[5] = RPY<> (tf_big_pulley_to_circle_points_.R())[2];

      std::cout << waypoint_pose_b_ << std::endl;

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

  bool robot_a_active = true;
  bool robot_b_active = true;

  double safty_margin_;
  safty_margin_ = 0.0007;

  // Set the robot initial state
  std::vector<std::string> joint_names_a;
  std::vector<std::string> joint_names_b;
  rw::math::Transform3D<> tf_a_b;

  rw::math::Transform3D<> tf_world_to_a_base_link_;
  rw::math::Transform3D<> tf_a_base_link_to_big_pulley_;
  rw::math::Transform3D<> tf_big_pulley_to_waypoints_;
  rw::math::Transform3D<> tf_world_to_waypoints_;

  tf_world_to_a_base_link_ = Transform3D<> (Vector3D<>(0, 0, 0.05), RPY<> (180*DEGREE2RADIAN,0,0).toRotation3D()); // RPY

  tf_a_b = tf_a_parts * inverse(tf_b_parts);

  //  joint_init_pos_a(0) = 2.97691;
  //  joint_init_pos_a(1) = -1.65511;
  //  joint_init_pos_a(2) = -2.33346;
  //  joint_init_pos_a(3) = -2.28544;
  //  joint_init_pos_a(4) = -2.52771;
  //  joint_init_pos_a(5) = -3.13402;
  //
  //  joint_init_pos_b(0) =  3.86493;
  //  joint_init_pos_b(1) =  -2.32467;
  //  joint_init_pos_b(2) =  -1.28264;
  //  joint_init_pos_b(3) =  -2.67925;
  //  joint_init_pos_b(4) =  1.07453;
  //  joint_init_pos_b(5) =  3.13593;

  joint_names_a.push_back("a_shoulder_pan_joint");
  joint_names_a.push_back("a_shoulder_lift_joint");
  joint_names_a.push_back("a_elbow_joint");
  joint_names_a.push_back("a_wrist_1_joint");
  joint_names_a.push_back("a_wrist_2_joint");
  joint_names_a.push_back("a_wrist_3_joint");

  joint_names_b.push_back("b_shoulder_pan_joint");
  joint_names_b.push_back("b_shoulder_lift_joint");
  joint_names_b.push_back("b_elbow_joint");
  joint_names_b.push_back("b_wrist_1_joint");
  joint_names_b.push_back("b_wrist_2_joint");
  joint_names_b.push_back("b_wrist_3_joint");

  /////////////
  /// SETUP ///
  /////////////

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Pull ROS params
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  // Initialize the environment
  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init<OFKTStateSolver>(urdf_xml_string, srdf_xml_string, locator))
    return false;

  env_->setState(joint_names_a, joint_init_pos_a);
  env_->setState(joint_names_b, joint_init_pos_b);


  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, EXAMPLE_MONITOR_NAMESPACE);
  //if (rviz_)
  //  monitor_->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);

  // Set default contact distance
  Command::Ptr cmd_default_dist = std::make_shared<tesseract_environment::ChangeDefaultContactMarginCommand>(0.005);
  if (!monitor_->applyCommand(*cmd_default_dist))
    return false;

  /////////////////////////
  /// motion way points ///
  /////////////////////////
  for(int num = 0; num < 5; num++)
  {

    if(robot_a_active)
    {
      waypoints_robot_a_.clear();
      //Robot A
      //in relative to big pulley
      // close to the pulley
      waypoint_pose_a_[0] = -0.130;
      waypoint_pose_a_[1] = 0.018;
      waypoint_pose_a_[2] = -0.01;
      waypoint_pose_a_[3] = 0*DEGREE2RADIAN;
      waypoint_pose_a_[4] = 25*DEGREE2RADIAN;
      waypoint_pose_a_[5] = 0;

      waypoints_robot_a_.push_back(waypoint_pose_a_);

      // insert belt to pulley groove

      waypoint_pose_a_[0] = -0.15;
      waypoint_pose_a_[1] = 0.014;
      waypoint_pose_a_[2] = 0.01;
      waypoint_pose_a_[3] = 0*DEGREE2RADIAN;
      waypoint_pose_a_[4] = 25*DEGREE2RADIAN;
      waypoint_pose_a_[5] = 0;

      waypoints_robot_a_.push_back(waypoint_pose_a_);

      //make_circle_waypoints(1, 0.018 + safty_margin_);

      // Create plotting tool
      ROSPlottingPtr plotter = std::make_shared<tesseract_rosutils::ROSPlotting>("world", "ur10e_a");
      if (rviz_)
        plotter->waitForConnection();

      ////////////
      /// Robot A ///
      ////////////
      //    if (rviz_)
      //      plotter->waitForInput();

      env_->setState(joint_names_a, joint_init_pos_a);
      env_->setState(joint_names_b, joint_init_pos_b);

      // Create Program a
      CompositeInstruction insert_program("DEFAULT", CompositeInstructionOrder::ORDERED, ManipulatorInfo("ur10e_a"));

      Waypoint insert_swp = StateWaypoint(joint_names_a, joint_init_pos_a);
      PlanInstruction start_instruction(insert_swp, PlanInstructionType::START);

      insert_program.setStartInstruction(start_instruction);


      //path planning
      // Define the approach pose
      Eigen::Isometry3d temp_pose;

      tf_world_to_a_base_link_ = Transform3D<> (Vector3D<>(0, 0, 0.05), RPY<> (180*DEGREE2RADIAN,0,0).toRotation3D()); // RPY

      tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_a_[1][0], waypoints_robot_a_[1][1], waypoints_robot_a_[1][2]), RPY<>(waypoints_robot_a_[1][3],waypoints_robot_a_[1][4],waypoints_robot_a_[1][5]).toRotation3D()); // RPY
      tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_parts*tf_big_pulley_to_waypoints_;


      temp_pose.linear() = Eigen::Quaterniond(rw::math::Quaternion<> (tf_world_to_waypoints_.R())[3],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[0],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[1], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
      temp_pose.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

      Waypoint temp_wp_0 = CartesianWaypoint(temp_pose);

      PlanInstruction inesrt_plan_a0(temp_wp_0, PlanInstructionType::FREESPACE, "DEFAULT");
      inesrt_plan_a0.setDescription("pose_1");



      insert_program.push_back(inesrt_plan_a0);
      //pick_program.push_back(pick_plan_a2);
      //pick_program.push_back(pick_plan_a3);
      //pick_program.push_back(pick_plan_a4);


      // Create Process Planning Server
      ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 10);
      planning_server.loadDefaultProcessPlanners();


      // Create TrajOpt Profile
      auto trajopt_plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
      trajopt_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 10);
      trajopt_plan_profile->set_term_type(trajopt::TermType::TT_COST);
      //
      ForceConstraint f(env_);
      f.set_data_(joint_init_pos_a, joint_init_pos_b, tf_a_parts, tf_b_parts);
      std::function<Eigen::VectorXd(const Eigen::VectorXd&)> temp_function = f;
      //temp_function = force_constraint_func;
      sco::VectorOfVector::func temp_a;
      temp_a = temp_function;

      sco::ConstraintType a = sco::ConstraintType::EQ;
      Eigen::VectorXd error_coeff(1);
      error_coeff << 0.2 ;

      std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd> temp_tuple(temp_a,nullptr,a,error_coeff);


      std::vector<std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd>>
      constraint_error_functions_;

      constraint_error_functions_.push_back(temp_tuple);

      auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
      trajopt_composite_profile->collision_constraint_config.enabled = false;
      trajopt_composite_profile->collision_cost_config.safety_margin = 0.001;
      trajopt_composite_profile->collision_cost_config.coeff = 50;
      trajopt_composite_profile->constraint_error_functions = constraint_error_functions_;


      auto trajopt_solver_profile = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
      trajopt_solver_profile->opt_info.max_iter = 500;

      // Add profile to Dictionary
      planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("DEFAULT", trajopt_plan_profile);
      planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
          trajopt_composite_profile);
      planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
          trajopt_solver_profile);


      ROS_INFO("Insert belt");

      // Create Process Planning Request
      ProcessPlanningRequest inesrt_request;
      inesrt_request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
      inesrt_request.instructions = Instruction(insert_program);

      // Print Diagnostics
      inesrt_request.instructions.print("Program: ");

      // Solve process plan
      ProcessPlanningFuture insert_response = planning_server.run(inesrt_request);

      planning_server.waitForAll();

      // Plot Process Trajectory
      if (rviz_ && plotter != nullptr && plotter->isConnected())
      {
        //plotter->waitForInput();
        const auto* cp = insert_response.results->cast_const<CompositeInstruction>();
        tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(*cp, env_);
        tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(*cp);
        plotter->plotMarker(ToolpathMarker(toolpath), "ur10e_a");
        plotter->plotTrajectory(trajectory, env_->getStateSolver());
      }
      if(num < 3)
        plotter->waitForInput();

      // retreat
      // Get the last move instruction
      trajopt_plan_profile->set_term_type(trajopt::TermType::TT_CNT);
      const CompositeInstruction* insert_composite = insert_response.results->cast_const<CompositeInstruction>();
      const MoveInstruction* inesrt_final_state = tesseract_planning::getLastMoveInstruction(*insert_composite);

      // Create Program
      CompositeInstruction retreat_program("DEFAULT", CompositeInstructionOrder::ORDERED, ManipulatorInfo("ur10e_a"));

      PlanInstruction retreat_start_instruction(inesrt_final_state->getWaypoint(), PlanInstructionType::START);
      retreat_program.setStartInstruction(retreat_start_instruction);

      // Define the retreat pose
      waypoints_robot_a_.clear();
      ///////////////
      waypoint_pose_a_[0] = -0.13;
      waypoint_pose_a_[1] = -0.02;
      waypoint_pose_a_[2] = -0.01;
      waypoint_pose_a_[3] = 50*DEGREE2RADIAN;
      waypoint_pose_a_[4] = 0*DEGREE2RADIAN;
      waypoint_pose_a_[5] = 0*DEGREE2RADIAN;;

      waypoints_robot_a_.push_back(waypoint_pose_a_);

      tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_a_[0][0], waypoints_robot_a_[0][1], waypoints_robot_a_[0][2]), RPY<>(waypoints_robot_a_[0][3],waypoints_robot_a_[0][4],waypoints_robot_a_[0][5]).toRotation3D()); // RPY
      tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_parts*tf_big_pulley_to_waypoints_;


      temp_pose.linear() = Eigen::Quaterniond(rw::math::Quaternion<> (tf_world_to_waypoints_.R())[3],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[0],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[1], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
      temp_pose.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

      Waypoint retreat_temp_wp_a0 = CartesianWaypoint(temp_pose);

      PlanInstruction retreat_plan_a0(retreat_temp_wp_a0, PlanInstructionType::FREESPACE, "DEFAULT");

      // Add Instructions to program
      //retreat_program.push_back(retreat_plan_a0);
      ///////////////////




      //   waypoints_robot_a_.clear();

      waypoint_pose_a_[0] = -0.13;
      waypoint_pose_a_[1] = -0.02;
      waypoint_pose_a_[2] = -0.02;
      waypoint_pose_a_[3] = 0*DEGREE2RADIAN;
      waypoint_pose_a_[4] = 0*DEGREE2RADIAN;
      waypoint_pose_a_[5] = 0*DEGREE2RADIAN;;

      waypoints_robot_a_.push_back(waypoint_pose_a_);

      tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_a_[1][0], waypoints_robot_a_[1][1], waypoints_robot_a_[1][2]), RPY<>(waypoints_robot_a_[1][3],waypoints_robot_a_[1][4],waypoints_robot_a_[1][5]).toRotation3D()); // RPY
      tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_parts*tf_big_pulley_to_waypoints_;


      temp_pose.linear() = Eigen::Quaterniond(rw::math::Quaternion<> (tf_world_to_waypoints_.R())[3],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[0],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[1], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
      temp_pose.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

      Waypoint retreat_temp_wp_a1 = CartesianWaypoint(temp_pose);

      PlanInstruction retreat_plan_a1(retreat_temp_wp_a1, PlanInstructionType::FREESPACE, "DEFAULT");
      safty_margin_ += 0.0025;
      make_circle_waypoints(1, 0.018 + safty_margin_);
      //
      tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_a_[2][0], waypoints_robot_a_[2][1], waypoints_robot_a_[2][2]), RPY<>(waypoints_robot_a_[2][3],waypoints_robot_a_[2][4],waypoints_robot_a_[2][5]).toRotation3D()); // RPY
      tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_parts *tf_big_pulley_to_waypoints_;


      temp_pose.linear() = Eigen::Quaterniond(rw::math::Quaternion<> (tf_world_to_waypoints_.R())[3],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[0],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[1], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
      temp_pose.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

      Waypoint temp_wp_1 = CartesianWaypoint(temp_pose);

      PlanInstruction inesrt_plan_a1(temp_wp_1, PlanInstructionType::FREESPACE, "DEFAULT");
      inesrt_plan_a1.setDescription("pose_2");

      tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_a_[3][0], waypoints_robot_a_[3][1], waypoints_robot_a_[3][2]), RPY<>(waypoints_robot_a_[3][3],waypoints_robot_a_[3][4],waypoints_robot_a_[3][5]).toRotation3D()); // RPY
      tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_parts *tf_big_pulley_to_waypoints_;


      temp_pose.linear() = Eigen::Quaterniond(rw::math::Quaternion<> (tf_world_to_waypoints_.R())[3],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[0],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[1], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
      temp_pose.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

      Waypoint temp_wp_2 = CartesianWaypoint(temp_pose);

      PlanInstruction inesrt_plan_a2(temp_wp_2, PlanInstructionType::FREESPACE, "DEFAULT");
      inesrt_plan_a2.setDescription("pose_3");

      tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_a_[4][0], waypoints_robot_a_[4][1], waypoints_robot_a_[4][2]), RPY<>(waypoints_robot_a_[4][3],waypoints_robot_a_[4][4],waypoints_robot_a_[4][5]).toRotation3D()); // RPY
      tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_parts*tf_big_pulley_to_waypoints_;


      temp_pose.linear() = Eigen::Quaterniond(rw::math::Quaternion<> (tf_world_to_waypoints_.R())[3],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[0],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[1], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
      temp_pose.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

      Waypoint temp_wp_3 = CartesianWaypoint(temp_pose);

      PlanInstruction inesrt_plan_a3(temp_wp_3, PlanInstructionType::FREESPACE, "DEFAULT");
      inesrt_plan_a3.setDescription("pose_4");

      // Add Instructions to program

      retreat_program.push_back(inesrt_plan_a1);
      retreat_program.push_back(inesrt_plan_a2);
      retreat_program.push_back(inesrt_plan_a3);
      retreat_program.push_back(retreat_plan_a1);

      //////////////////

      // Create Process Planning Request
      ProcessPlanningRequest retreat_request;
      retreat_request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
      retreat_request.instructions = Instruction(retreat_program);

      // Print Diagnostics
      retreat_request.instructions.print("Program: ");

      // Create TrajOpt Profile

      trajopt_composite_profile->constraint_error_functions.clear();


      // Add profile to Dictionary
      planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("DEFAULT", trajopt_plan_profile);
      planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
          trajopt_composite_profile);
      planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
          trajopt_solver_profile);


      // Solve process plan
      ProcessPlanningFuture retreat_response = planning_server.run(retreat_request);
      planning_server.waitForAll();

      // Plot Process Trajectory
      if (rviz_ && plotter != nullptr && plotter->isConnected())
      {
        //plotter->waitForInput();
        const auto* ci = retreat_response.results->cast_const<tesseract_planning::CompositeInstruction>();
        tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(*ci, env_);
        tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(*ci);
        plotter->plotMarker(ToolpathMarker(toolpath), "ur10e_a");
        plotter->plotTrajectory(trajectory, env_->getStateSolver());

        //get final position
        for(int num=0; num < 6; num ++)
          joint_init_pos_a(num) = trajectory[trajectory.size()-1].position[num];
      }
    }
  }

  //  if(robot_b_active)
  //  {
  //    ///// ROBOT B //////
  //    // Create plotting tool
  //    ROSPlottingPtr plotter_b = std::make_shared<tesseract_rosutils::ROSPlotting>("world", "ur10e_b");
  //    if (rviz_)
  //      plotter_b->waitForConnection();
  //
  //    //    if (rviz_)
  //    //      plotter_b->waitForInput();
  //
  //    //Robot B
  //    //in relative to big pulley
  //    // close to the pulley
  //    waypoint_pose_b_[0] = 0.04;
  //    waypoint_pose_b_[1] = 0.036;
  //    waypoint_pose_b_[2] = -0.02;
  //    waypoint_pose_b_[3] = 0*DEGREE2RADIAN;
  //    waypoint_pose_b_[4] = -25*DEGREE2RADIAN;
  //    waypoint_pose_b_[5] = 0;
  //
  //    waypoints_robot_b_.push_back(waypoint_pose_b_);
  //
  //    // insert belt to pulley groove
  //
  //    waypoint_pose_b_[0] = 0.04;
  //    waypoint_pose_b_[1] = 0.036;
  //    waypoint_pose_b_[2] = -0.02;
  //    waypoint_pose_b_[3] = 0*DEGREE2RADIAN;
  //    waypoint_pose_b_[4] = -25*DEGREE2RADIAN;
  //    waypoint_pose_b_[5] = 0;
  //
  //    waypoints_robot_b_.push_back(waypoint_pose_b_);
  //
  //    safty_margin_ = 0.0007;
  //    make_circle_waypoints(-1, 0.036 + safty_margin_);
  //
  //    std::cout << "!!!!!!!!!" <<joint_init_pos_a << std::endl;
  //
  //    env_->setState(joint_names_a, joint_init_pos_a);
  //    env_->setState(joint_names_b, joint_init_pos_b);
  //
  //    CompositeInstruction insert_program("DEFAULT", CompositeInstructionOrder::ORDERED, ManipulatorInfo("ur10e_b"));
  //
  //    Waypoint insert_swp = StateWaypoint(joint_names_b, joint_init_pos_b);
  //    PlanInstruction start_instruction(insert_swp, PlanInstructionType::START);
  //    insert_program.setStartInstruction(start_instruction);
  //
  //    //path planning
  //    // Define the approach pose
  //    Eigen::Isometry3d temp_pose_b;
  //
  //    for(int num = 0; num < 2; num ++)
  //    {
  //
  //      tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_b_[num][0], waypoints_robot_b_[num][1], waypoints_robot_b_[num][2]), RPY<>(waypoints_robot_b_[num][3],waypoints_robot_b_[num][4],waypoints_robot_b_[num][5]).toRotation3D()); // RPY
  //      tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_parts * tf_big_pulley_to_waypoints_;
  //
  //
  //      temp_pose_b.linear() = Eigen::Quaterniond(rw::math::Quaternion<> (tf_world_to_waypoints_.R())[3],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[0],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[1], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  //      temp_pose_b.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world
  //
  //      Waypoint temp_wp_b0 = CartesianWaypoint(temp_pose_b);
  //
  //      PlanInstruction insert_plan_b0(temp_wp_b0, PlanInstructionType::FREESPACE, "DEFAULT");
  //      insert_plan_b0.setDescription("pose_1");
  //
  //
  //      insert_program.push_back(insert_plan_b0);
  //    }
  //
  //    tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_b_[2][0], waypoints_robot_b_[2][1], waypoints_robot_b_[2][2]), RPY<>(waypoints_robot_b_[2][3],waypoints_robot_b_[2][4],waypoints_robot_b_[2][5]).toRotation3D()); // RPY
  //    tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_parts * tf_big_pulley_to_waypoints_;
  //
  //
  //    temp_pose_b.linear() = Eigen::Quaterniond(rw::math::Quaternion<> (tf_world_to_waypoints_.R())[3], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[0], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[1], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  //    temp_pose_b.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world
  //
  //    Waypoint temp_wp_b1 = CartesianWaypoint(temp_pose_b);
  //
  //    PlanInstruction insert_plan_b1(temp_wp_b1, PlanInstructionType::FREESPACE, "DEFAULT");
  //    insert_plan_b1.setDescription("pose_2");
  //
  //    tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_b_[3][0], waypoints_robot_b_[3][1], waypoints_robot_b_[3][2]), RPY<>(waypoints_robot_b_[3][3],waypoints_robot_b_[3][4],waypoints_robot_b_[3][5]).toRotation3D()); // RPY
  //    tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_parts * tf_big_pulley_to_waypoints_;
  //
  //
  //    temp_pose_b.linear() = Eigen::Quaterniond(rw::math::Quaternion<> (tf_world_to_waypoints_.R())[3], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[0], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[1], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  //    temp_pose_b.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world
  //
  //    Waypoint temp_wp_b2 = CartesianWaypoint(temp_pose_b);
  //
  //    PlanInstruction insert_plan_b2(temp_wp_b2, PlanInstructionType::FREESPACE, "DEFAULT");
  //    insert_plan_b2.setDescription("pose_3");
  //
  //    tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_b_[4][0], waypoints_robot_b_[4][1], waypoints_robot_b_[4][2]), RPY<>(waypoints_robot_b_[4][3],waypoints_robot_b_[4][4],waypoints_robot_b_[4][5]).toRotation3D()); // RPY
  //    tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_parts * tf_big_pulley_to_waypoints_;
  //
  //    temp_pose_b.linear() = Eigen::Quaterniond(rw::math::Quaternion<> (tf_world_to_waypoints_.R())[3],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[0],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[1], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  //    temp_pose_b.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world
  //
  //    Waypoint temp_wp_b3 = CartesianWaypoint(temp_pose_b);
  //
  //    PlanInstruction insert_plan_b3(temp_wp_b3, PlanInstructionType::FREESPACE, "DEFAULT");
  //    insert_plan_b3.setDescription("pose_4");
  //
  //    tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_b_[4][0], waypoints_robot_b_[4][1], waypoints_robot_b_[4][2]), RPY<>(waypoints_robot_b_[4][3],waypoints_robot_b_[4][4],waypoints_robot_b_[4][5]).toRotation3D()); // RPY
  //    tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_parts * tf_big_pulley_to_waypoints_;
  //
  //
  //    temp_pose_b.linear() = Eigen::Quaterniond(rw::math::Quaternion<> (tf_world_to_waypoints_.R())[3],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[0],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[1], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  //    temp_pose_b.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world
  //
  //    Waypoint temp_wp_b4 = CartesianWaypoint(temp_pose_b);
  //
  //    PlanInstruction insert_plan_b4(temp_wp_b4, PlanInstructionType::FREESPACE, "DEFAULT");
  //    insert_plan_b4.setDescription("pose_5");
  //
  //    insert_program.push_back(insert_plan_b1);
  //    insert_program.push_back(insert_plan_b2);
  //    insert_program.push_back(insert_plan_b3);
  //    insert_program.push_back(insert_plan_b4);
  //
  //
  //    // Create Process Planning Server
  //    ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 10);
  //    planning_server.loadDefaultProcessPlanners();
  //
  //
  //    // Create TrajOpt Profile
  //    auto trajopt_plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
  //    trajopt_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 10);
  //    trajopt_plan_profile->set_term_type(trajopt::TermType::TT_COST);
  //
  //    ForceConstraint f_b(env_);
  //    f_b.set_robot_(1);
  //    f_b.set_data_(joint_init_pos_a, joint_init_pos_b, tf_a_parts, tf_b_parts);
  //    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> temp_function_b = f_b;
  //    //temp_function = force_constraint_func;
  //    sco::VectorOfVector::func temp_b;
  //    temp_b = temp_function_b;
  //    //temp_a = force_constraint_func;
  //    //sco::MatrixOfVector::func,
  //    sco::ConstraintType b = sco::ConstraintType::EQ;
  //    Eigen::VectorXd error_coeff_b(1);
  //    error_coeff_b << 0.2 ;
  //
  //    std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd> temp_tuple(temp_b,nullptr,b,error_coeff_b);
  //
  //
  //    std::vector<std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd>>
  //    constraint_error_functions;
  //
  //    constraint_error_functions.push_back(temp_tuple);
  //
  //    auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  //    trajopt_composite_profile->collision_constraint_config.enabled = false;
  //    trajopt_composite_profile->collision_cost_config.safety_margin = 0.001;
  //    trajopt_composite_profile->collision_cost_config.coeff = 50;
  //    trajopt_composite_profile->constraint_error_functions = constraint_error_functions;
  //
  //
  //    auto trajopt_solver_profile_b = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
  //    trajopt_solver_profile_b->opt_info.max_iter = 500;
  //
  //    // Add profile to Dictionary
  //    planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("DEFAULT", trajopt_plan_profile);
  //    planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
  //        trajopt_composite_profile);
  //    planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
  //        trajopt_solver_profile_b);
  //
  //    ROS_INFO("Insert plan");
  //
  //    // Create Process Planning Request
  //    ProcessPlanningRequest insert_request;
  //    insert_request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  //    insert_request.instructions = Instruction(insert_program);
  //
  //    // Print Diagnostics
  //    insert_request.instructions.print("Program: ");
  //
  //    // Solve process plan
  //    ProcessPlanningFuture insert_response = planning_server.run(insert_request);
  //    planning_server.waitForAll();
  //
  //    // Plot Process Trajectory
  //    if (rviz_ && plotter_b != nullptr && plotter_b->isConnected())
  //    {
  //      //plotter_b->waitForInput();
  //      const auto* cp_b = insert_response.results->cast_const<CompositeInstruction>();
  //      tesseract_common::Toolpath toolpath_b = tesseract_planning::toToolpath(*cp_b, env_);
  //      tesseract_common::JointTrajectory trajectory_b = tesseract_planning::toJointTrajectory(*cp_b);
  //      plotter_b->plotMarker(ToolpathMarker(toolpath_b), "ur10e_b");
  //      plotter_b->plotTrajectory(trajectory_b, env_->getStateSolver());
  //    }
  //
  //    // retreat
  //    // Get the last move instruction
  //    trajopt_plan_profile->set_term_type(trajopt::TermType::TT_CNT);
  //    const CompositeInstruction* insert_composite = insert_response.results->cast_const<CompositeInstruction>();
  //    const MoveInstruction* insert_final_state = tesseract_planning::getLastMoveInstruction(*insert_composite);
  //
  //    // Create Program
  //    CompositeInstruction retreat_program("DEFAULT", CompositeInstructionOrder::ORDERED, ManipulatorInfo("ur10e_b"));
  //
  //    PlanInstruction insert_start_instruction(insert_final_state->getWaypoint(), PlanInstructionType::START);
  //    retreat_program.setStartInstruction(insert_start_instruction);
  //
  //    // Define the retreat pose
  //    waypoints_robot_b_.clear();
  //
  //    waypoint_pose_b_[0] = -0.02;
  //    waypoint_pose_b_[1] = -0.03;
  //    waypoint_pose_b_[2] = -0.03;
  //    waypoint_pose_b_[3] = -50*DEGREE2RADIAN;
  //    waypoint_pose_b_[4] = -25*DEGREE2RADIAN;
  //    waypoint_pose_b_[5] = 0*DEGREE2RADIAN;
  //
  //    waypoints_robot_b_.push_back(waypoint_pose_b_);
  //
  //    tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_b_[0][0], waypoints_robot_b_[0][1], waypoints_robot_b_[0][2]), RPY<>(waypoints_robot_b_[0][3],waypoints_robot_b_[0][4],waypoints_robot_b_[0][5]).toRotation3D()); // RPY
  //    tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_parts * tf_big_pulley_to_waypoints_;
  //
  //    temp_pose_b.linear() = Eigen::Quaterniond(rw::math::Quaternion<> (tf_world_to_waypoints_.R())[3],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[0],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[1], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  //    temp_pose_b.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world
  //
  //    Waypoint retreat_temp_wp_b0 = CartesianWaypoint(temp_pose_b);
  //
  //    PlanInstruction retreat_plan_b0(retreat_temp_wp_b0, PlanInstructionType::FREESPACE, "DEFAULT");
  //
  //    // Add Instructions to program
  //    retreat_program.push_back(retreat_plan_b0);
  //
  //    // Define the retreat pose
  //    waypoints_robot_b_.clear();
  //
  //    waypoint_pose_b_[0] = 0.03;
  //    waypoint_pose_b_[1] = 0;
  //    waypoint_pose_b_[2] = -0.04;
  //    waypoint_pose_b_[3] = 0*DEGREE2RADIAN;
  //    waypoint_pose_b_[4] = -25*DEGREE2RADIAN;
  //    waypoint_pose_b_[5] = 0*DEGREE2RADIAN;
  //
  //    waypoints_robot_b_.push_back(waypoint_pose_b_);
  //
  //    tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_b_[0][0], waypoints_robot_b_[0][1], waypoints_robot_b_[0][2]), RPY<>(waypoints_robot_b_[0][3],waypoints_robot_b_[0][4],waypoints_robot_b_[0][5]).toRotation3D()); // RPY
  //    tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_parts * tf_big_pulley_to_waypoints_;
  //
  //
  //    temp_pose_b.linear() = Eigen::Quaterniond(rw::math::Quaternion<> (tf_world_to_waypoints_.R())[3],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[0],rw::math::Quaternion<> (tf_world_to_waypoints_.R())[1], rw::math::Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
  //    temp_pose_b.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world
  //
  //    Waypoint retreat_temp_wp_b1 = CartesianWaypoint(temp_pose_b);
  //
  //    PlanInstruction retreat_plan_b1(retreat_temp_wp_b1, PlanInstructionType::FREESPACE, "DEFAULT");
  //
  //    // Add Instructions to program
  //    retreat_program.push_back(retreat_plan_b1);
  //
  //
  //    // Create Process Planning Request
  //    ProcessPlanningRequest retreat_request;
  //    retreat_request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  //    retreat_request.instructions = Instruction(retreat_program);
  //
  //    // Print Diagnostics
  //    retreat_request.instructions.print("Program: ");
  //
  //    // Create TrajOpt Profile
  //
  //    trajopt_composite_profile->constraint_error_functions.clear();
  //
  //    // Add profile to Dictionary
  //    planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("CARTESIAN", trajopt_plan_profile);
  //    planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
  //        trajopt_composite_profile);
  //    planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
  //        trajopt_solver_profile_b);
  //
  //
  //    // Solve process plan
  //    ProcessPlanningFuture retreat_response = planning_server.run(retreat_request);
  //    planning_server.waitForAll();
  //
  //    // Plot Process Trajectory
  //    if (rviz_ && plotter_b != nullptr && plotter_b->isConnected())
  //    {
  //      plotter_b->waitForInput();
  //      const auto* cp_b = retreat_response.results->cast_const<CompositeInstruction>();
  //      tesseract_common::Toolpath toolpath_b = tesseract_planning::toToolpath(*cp_b, env_);
  //      tesseract_common::JointTrajectory trajectory_b = tesseract_planning::toJointTrajectory(*cp_b);
  //      plotter_b->plotMarker(ToolpathMarker(toolpath_b), "ur10e_b");
  //      plotter_b->plotTrajectory(trajectory_b, env_->getStateSolver());
  //    }
  //    if (rviz_)
  //      plotter_b->waitForConnection();
  //  }
  //
  if(!robot_a_active && !robot_b_active)
  {
    ROS_INFO("No active robots");
    return true;
  }

  ROS_INFO("Done");
  return true;
}
}  // namespace tesseract_ros_examples

