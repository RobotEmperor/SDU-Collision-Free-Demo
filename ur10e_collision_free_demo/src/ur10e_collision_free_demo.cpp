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

//PickAndPlaceExample::TrajoptPickAndPlaceConstructor(tesseract::BasicEnvConstPtr env,
//                                                               std::string manipulator,
//                                                               std::string ee_link,
//                                                               std::string pick_object,
//                                                               Isometry3d tcp)
//  : manipulator_(manipulator), ee_link_(ee_link), pick_object_(pick_object), tcp_(tcp), env_(env)
//{
//  kin_ = env->getManipulator(manipulator_);
//}

//void PickAndPlaceExample::addTotalTimeCost(ProblemConstructionInfo& pci, double coeff)
//{
//  //std::shared_ptr<TotalTimeTermInfo> time_cost(new TotalTimeTermInfo);
//  //time_cost->name = "time_cost";
//  //time_cost->term_type = TT_COST;
//  //pci.cost_infos.push_back(time_cost);
//}

void TestExample::addSingleWaypoint(trajopt::ProblemConstructionInfo& pci,
    Eigen::Isometry3d pose,
    int time_step)
{
  Eigen::Quaterniond rotation(pose.linear());

  auto pose_constraint = std::make_shared<trajopt::CartPoseTermInfo>();
  pose_constraint->term_type = trajopt::TT_CNT;
  pose_constraint->link = "a_final_link_2";
  pose_constraint->target = "world";
  pose_constraint->timestep = time_step;
  pose_constraint->xyz = pose.translation();

  pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
  pose_constraint->pos_coeffs = Eigen::Vector3d(30.0, 30.0, 30.0);
  pose_constraint->rot_coeffs = Eigen::Vector3d(30.0, 30.0, 30.0);
  pose_constraint->name = "pose_" + std::to_string(time_step);
  pci.cnt_infos.push_back(pose_constraint);

}
void TestExample::addLinearMotion(trajopt::ProblemConstructionInfo& pci,
    Eigen::Isometry3d start_pose,
    Eigen::Isometry3d end_pose,
    int num_steps,
    int first_time_step)
{
  // linear delta
  Eigen::Vector3d xyz_delta = (end_pose.translation() - start_pose.translation()) / (num_steps - 1);

  Eigen::Quaterniond approach_rotation(start_pose.linear());
  Eigen::Matrix3d rotation_diff = (start_pose.linear().inverse() * end_pose.linear());
  Eigen::AngleAxisd aa_rotation_diff(rotation_diff);
  double angle_delta = aa_rotation_diff.angle() / (num_steps - 1);
  Eigen::Vector3d delta_axis = aa_rotation_diff.axis();

  // Create a series of pose constraints for linear pick motion
  for (int i = 0; i < num_steps; i++)
  {
    /* Fill Code:
         . Create a new shared_ptr<StaticPoseCostInfo>
         . Define the term type (This is a constraint)
         . Set the link constrained as the end effector (see class members)
         . Set the correct time step for this pose
         . Set the pose xyz translation
     */
    /* ========  ENTER CODE HERE ======== */
    auto pose_constraint = std::make_shared<trajopt::CartPoseTermInfo>();
    pose_constraint->term_type = trajopt::TT_CNT;
    pose_constraint->link = "a_final_link_2";
    pose_constraint->timestep = i + first_time_step;
    //pose_constraint->target = "a_final_link_2";
    // pose_constraint->xyz = end_pose.translation() //+ xyz_delta * i;
    //pose_constraint->tcp.translation() = start_pose.translation() + xyz_delta * (i);
    //pose_constraint->target_tcp.translation() = start_pose.translation() + xyz_delta * i;

    Eigen::Quaterniond rotation_delta(cos(0.5 * angle_delta * i),
        delta_axis.x() * sin(0.5 * angle_delta * i),
        delta_axis.y() * sin(0.5 * angle_delta * i),
        delta_axis.z() * sin(0.5 * angle_delta * i));
    Eigen::Quaterniond rotation = rotation_delta * approach_rotation;

    /* Fill Code:
         . Set the pose rotation
         . Set pos_coeffs to all 10s
         . Set rot_coeffs to all 10s
         . Define the pose name as pose_[timestep]
         . pushback the constraint to cnt_infos
     */
    /* ========  ENTER CODE HERE ======== */
    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());


    //pose_constraint->tcp.linear() = Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z()).matrix();
    //pose_constraint->target_tcp.linear() = Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z()).matrix();
    pose_constraint->pos_coeffs = Eigen::Vector3d(10, 10, 10);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10, 10, 10);
    //pose_constraint->
    pose_constraint->name = "pose_" + std::to_string(i + first_time_step);
    pci.cnt_infos.push_back(pose_constraint);
  }

  //  auto collision = std::make_shared<trajopt::CollisionTermInfo>();
  //  collision->name =  "collision_" + std::to_string(first_time_step);
  //  collision->term_type = trajopt::TT_CNT;
  //  collision->evaluator_type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
  //  collision->first_step = 0;
  //  collision->last_step = pci.basic_info.n_steps - 1;
  //  collision->contact_test_type = tesseract_collision::ContactTestType::ALL;
  //  collision->longest_valid_segment_length = LONGEST_VALID_SEGMENT_LENGTH;
  //  collision->info = trajopt::createSafetyMarginDataVector(pci.basic_info.n_steps , 0.01, 10);
  //  collision->safety_margin_buffer = 0.01;
  //  pci.cnt_infos.push_back(collision);
}

//trajopt::TrajOptProbPtr TestExample::generatePickProblem(Eigen::Isometry3d& approach_pose,
//    Eigen::Isometry3d& final_pose,
//                                                                   int steps_per_phase)
//{
//  //---------------------------------------------------------
//  // ---------------- Fill Basic Info -----------------------
//  //---------------------------------------------------------
//  trajopt::ProblemConstructionInfo pci(env_);
//  // Add kinematics
//  pci.kin = kin_;
//
//  /* Fill Code: Define the basic info
//       . Set the pci number of steps
//       . Set the start_fixed to false
//       . Set the manipulator name (see class members)
//       . Set dt lower limit
//       . Set dt upper limit
//       . Set use_time to false
//  */
//  /* ========  ENTER CODE HERE ======== */
//  pci.basic_info.n_steps = steps_per_phase * 2;
//  pci.basic_info.manip = manipulator_;
//  pci.basic_info.dt_lower_lim = 0.2;
//  pci.basic_info.dt_upper_lim = .5;
//  pci.basic_info.start_fixed = true;
//  pci.basic_info.use_time = false;
//
//  //---------------------------------------------------------
//  // ---------------- Fill Init Info ------------------------
//  //---------------------------------------------------------
//
//  // To use JOINT_INTERPOLATED - a linear interpolation of joint values
//  //  pci.init_info.type = InitInfo::JOINT_INTERPOLATED;
//  //  pci.init_info.data = numericalIK(final_pose);  // Note, take the last value off if using time (just want jnt
//  //  values)
//
//  // To use STATIONARY - all jnts initialized to the starting value
//  pci.init_info.type = InitInfo::STATIONARY;
//  pci.init_info.dt = 0.5;
//
//  //---------------------------------------------------------
//  // ---------------- Fill Term Infos -----------------------
//  //---------------------------------------------------------
//
//  // ================= Collision cost =======================
//  std::shared_ptr<CollisionTermInfo> collision(new CollisionTermInfo);
//  /* Fill Code:
//       . Define the cost name
//       . Define the term type (This is a cost)
//       . Define this cost as continuous
//       . Define the first time step
//       . Define the last time step
//       . Set the cost gap to be 1
//  */
//  /* ========  ENTER CODE HERE ======== */
//  collision->name = "collision";
//  collision->term_type = TT_COST;
//  collision->continuous = true;
//  collision->first_step = 0;
//  collision->last_step = pci.basic_info.n_steps - 1;
//  collision->gap = 1;
//  collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 40);
//
//  pci.cost_infos.push_back(collision);
//
//  // ================= Velocity cost =======================
//  std::shared_ptr<JointVelTermInfo> jv(new JointVelTermInfo);
//
//  // Taken from iiwa documentation (radians/s) and scaled by 0.8
//  std::vector<double> vel_lower_lim{ 1.71 * -0.8, 1.71 * -0.8, 1.75 * -0.8, 2.27 * -0.8,
//                                     2.44 * -0.8, 3.14 * -0.8, 3.14 * -0.8 };
//  std::vector<double> vel_upper_lim{
//    1.71 * 0.8, 1.71 * 0.8, 1.75 * 0.8, 2.27 * 0.8, 2.44 * 0.8, 3.14 * 0.8, 3.14 * 0.8
//  };
//
//  /* Fill Code:
//       . Define the term time (This is a cost)
//       . Define the first time step
//       . Define the last time step
//       . Define vector of target velocities. Length = DOF. Value = 0
//       . Define vector of coefficients. Length = DOF. Value = 5
//       . Define the term name
//  */
//  /* ========  ENTER CODE HERE ======== */
//  jv->term_type = TT_COST;
//  jv->first_step = 0;
//  jv->last_step = pci.basic_info.n_steps - 1;
//  jv->targets = std::vector<double>(7, 0.0);
//  jv->coeffs = std::vector<double>(7, 5.0);
//  jv->name = "joint_velocity_cost";
//
//  pci.cost_infos.push_back(jv);
//
//  // ================= Path waypoints =======================
//  this->addLinearMotion(pci, approach_pose, final_pose, steps_per_phase, steps_per_phase);
//
//  TrajOptProbPtr result = ConstructProblem(pci);
//  return result;
//}


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
      rotation_y_ = 33*DEGREE2RADIAN*num;
      rotation_z_ = 0*DEGREE2RADIAN*num;
      rotation_x_ = -33*DEGREE2RADIAN*num;

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
    theta_ =-15*DEGREE2RADIAN - offset_;
    for(int num = 1; num < 10; num ++)
    {
      x_ = radious_*sin(theta_*num - offset_);
      y_ = radious_*cos(theta_*num - offset_);
      rotation_z_ = -15*DEGREE2RADIAN*num - offset_ ;

      tf_big_pulley_to_circle_points_ = Transform3D<> (Vector3D<>(x_, y_, -0.02), RPY<>(rotation_z_,0,0).toRotation3D()); // RPY

      waypoint_pose_a_[0] = tf_big_pulley_to_circle_points_.P()[0];
      waypoint_pose_a_[1] = tf_big_pulley_to_circle_points_.P()[1];
      waypoint_pose_a_[2] = tf_big_pulley_to_circle_points_.P()[2];
      waypoint_pose_a_[3] = RPY<> (tf_big_pulley_to_circle_points_.R())[0];
      waypoint_pose_a_[4] = RPY<> (tf_big_pulley_to_circle_points_.R())[1];
      waypoint_pose_a_[5] = RPY<> (tf_big_pulley_to_circle_points_.R())[2];

      waypoints_robot_b_.push_back(waypoint_pose_a_);
    }
  }
}
//
Eigen::VectorXd TestExample::force_constraint_func(const Eigen::VectorXd& current_joints_pos)
{
  static Eigen::VectorXd previous_joints_pos(6);
  //initial joint pose
  Eigen::VectorXd joint_init_pos(6);
  joint_init_pos(0) = 2.97326;
  joint_init_pos(1) = -1.6538;
  joint_init_pos(2) = -2.33488;
  joint_init_pos(3) = -2.28384;
  joint_init_pos(4) = -2.53001;
  joint_init_pos(5) = -3.13221;

  std::vector<std::string> joint_names;
  joint_names.push_back("a_shoulder_pan_joint");
  joint_names.push_back("a_shoulder_lift_joint");
  joint_names.push_back("a_elbow_joint");
  joint_names.push_back("a_wrist_1_joint");
  joint_names.push_back("a_wrist_2_joint");
  joint_names.push_back("a_wrist_3_joint");

  std::string link = "a_final_link_2";
  Eigen::Isometry3d current_pose = env_->getCurrentState()->link_transforms.at(link);
  Eigen::Isometry3d initial_pose_;

  auto kin = env_->getManipulatorManager()->getFwdKinematicSolver("ur10e_a");
  kin->calcFwdKin(initial_pose_, joint_init_pos);

  // constraint function

  double spring_constant_k_ = 10;
  double force_magnitude_ = 0;
  double desired_force_magnitude_ = 10;
  static Eigen::VectorXd violation_joints(6);
  violation_joints << 0,0,0,0,0,0;

  static std::vector<double> force_unit_vector_;

  force_magnitude_ = sqrt(spring_constant_k_*spring_constant_k_*(pow(2,initial_pose_.translation()(0) - current_pose.translation()(0))
      + pow(2,initial_pose_.translation()(1) - current_pose.translation()(1))
      + pow(2,initial_pose_.translation()(2) - current_pose.translation()(2))));

  force_unit_vector_.push_back(initial_pose_.translation()(0) - current_pose.translation()(0));
  force_unit_vector_.push_back(initial_pose_.translation()(1) - current_pose.translation()(1));
  force_unit_vector_.push_back(initial_pose_.translation()(2) - current_pose.translation()(2));

  //    for(long unsigned int num_inner_ = 0; num_inner_ < 3; num_inner_ ++)
  //    {
  //      current_object_force_torque_vector_[num_+1][num_inner_]=force_unit_vector_[num_inner_]*force_magnitude_;
  //    }
  //check
  std::cout << force_unit_vector_ << std::endl;
  //std::cout << objects_force_magnitude_ << std::endl;
  force_unit_vector_.clear();

  if(force_magnitude_ - desired_force_magnitude_+2 <= 0 )
  {
    violation_joints << 0,0,0,0,0,0;
    previous_joints_pos = current_joints_pos;
    return violation_joints;
  }
  if(desired_force_magnitude_- force_magnitude_ <= 0)
  {
    violation_joints << 0,0,0,0,0,0;
    previous_joints_pos = current_joints_pos;
    return violation_joints;
  }
  else
  {
    violation_joints = previous_joints_pos - current_joints_pos;
  }

  previous_joints_pos = current_joints_pos;
  return violation_joints;
}
//
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

  waypoint_pose_a_[0] = -0.155;
  waypoint_pose_a_[1] = 0;
  waypoint_pose_a_[2] = -0.015;
  waypoint_pose_a_[3] = 0*DEGREE2RADIAN;
  waypoint_pose_a_[4] = 0;
  waypoint_pose_a_[5] = 0;

  waypoints_robot_a_.push_back(waypoint_pose_a_);

  double safty_margin_;
  safty_margin_ = 0.0007;
  make_circle_waypoints(1, 0.03 + safty_margin_);

  // std::cout << waypoints_robot_a_.size() << std::endl;
  /*
  for(int num = 0; num < waypoints_robot_a_.size(); num ++)
  {
    std::cout << "NUM: " <<num << "Values 0 : "<< waypoints_robot_a_[num][0] << std::endl;
    std::cout << "NUM: " <<num << "Values 1 : "<< waypoints_robot_a_[num][1] << std::endl;
    std::cout << "NUM: " <<num << "Values 2 : "<< waypoints_robot_a_[num][2] << std::endl;
    std::cout << "NUM: " <<num << "Values 3 : "<< waypoints_robot_a_[num][3] << std::endl;
    std::cout << "NUM: " <<num << "Values 4 : "<< waypoints_robot_a_[num][4] << std::endl;
    std::cout << "NUM: " <<num << "Values 5 : "<< waypoints_robot_a_[num][5] << std::endl;
  }
   */
  //tf_big_pulley_to_small_pulley_


  //Robot B


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
  ROSPlottingPtr plotter = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getSceneGraph()->getRoot());
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
  //joint_names.push_back("a_robotiq_85_right_inner_knuckle_joint");
  //joint_names.push_back("a_robotiq_85_right_finger_tip_joint");

  Eigen::VectorXd joint_pos(6);
  joint_pos(0) = 2.97326;
  joint_pos(1) = -1.6538;
  joint_pos(2) = -2.33488;
  joint_pos(3) = -2.28384;
  joint_pos(4) = -2.53001;
  joint_pos(5) = -3.13221;
  //joint_pos(6) = 0;
  //joint_pos(7) = 0;


  env_->setState(joint_names, joint_pos);

  // Add simulated box to environment
  //  Command::Ptr cmd = addBox(box_x, box_y, box_side);
  //  if (!monitor_->applyCommand(*cmd))
  //    return false;

  ////////////
  /// PICK ///
  ////////////
  if (rviz_)
    plotter->waitForInput();

  // Create Program
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
  pick_program.push_back(pick_plan_a2);
  pick_program.push_back(pick_plan_a3);
  pick_program.push_back(pick_plan_a4);



  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 10);
  planning_server.loadDefaultProcessPlanners();


  // Create TrajOpt Profile
  auto trajopt_plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
  trajopt_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 10);

  std::function<Eigen::VectorXd(TestExample&, const Eigen::VectorXd&)> temp_function = &TestExample::force_constraint_func;
  //temp_function = force_constraint_func;
  sco::VectorOfVector::func temp_a;
  //temp_a = force_constraint_func;
  //sco::MatrixOfVector::func,
  sco::ConstraintType a = sco::ConstraintType::EQ;
  Eigen::VectorXd error_coeff(6);
  error_coeff << 1,1,1,1,1,1 ;

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
    plotter->plotMarker(ToolpathMarker(toolpath));
    plotter->plotTrajectory(trajectory, env_->getStateSolver());
  }

  ROS_INFO("Done");
  return true;
}
}  // namespace tesseract_ros_examples

