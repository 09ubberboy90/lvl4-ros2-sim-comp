#include "moveit.hpp"
#include "geometry_msgs/msg/pose.hpp"

using std::placeholders::_1;
using namespace Eigen;
using namespace std::chrono_literals;

enum gripper_state
{
    opened = 35,
    closed = 0
};

bool wait_for_exec(moveit::planning_interface::MoveGroupInterface *move_group)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    for (int i = 0; i < 10; i++)
    {
        // 10 tries to plan otherwise give up
        bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            std::thread([move_group, plan]() { move_group->execute(plan); }).detach();
            //move_group->asyncExecute(plan);
            return true;
        }
    }
    throw "Couldn't plan a path";
    return false;
}

bool change_gripper(moveit::planning_interface::MoveGroupInterface *hand_move_group, gripper_state state)
{
    auto current_state = hand_move_group->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(current_state->getJointModelGroup("hand"), joint_group_positions);
    float pose = (float)state / 1000;
    joint_group_positions[0] = pose;
    joint_group_positions[1] = pose;
    hand_move_group->setJointValueTarget(joint_group_positions);
    return wait_for_exec(hand_move_group);
}

bool goto_pose(moveit::planning_interface::MoveGroupInterface *move_group, geometry_msgs::msg::Pose pose)
{
    move_group->setPoseTarget(pose);
    return wait_for_exec(move_group);
}
bool goto_joint_pose(moveit::planning_interface::MoveGroupInterface *move_group, sensor_msgs::msg::JointState joints)
{
    move_group->setJointValueTarget(joints);
    return wait_for_exec(move_group);
}
class GetPose : public rclcpp::Node
{
public:
    GetPose() : Node("get_pose")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/model/throwing_object/pose", 10, std::bind(&GetPose::listener_callback, this, std::placeholders::_1));

    };
    geometry_msgs::msg::Pose pose;

private:
    void listener_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        pose = *msg;
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);
    auto pose_node = std::make_shared<GetPose>();

    // For current state monitor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(pose_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface hand_move_group(move_group_node, "hand");
    move_group.allowReplanning(true);
    move_group.setNumPlanningAttempts(10);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface("");

    bool use_spawn_obj;
    if (!move_group_node->get_parameter("use_spawn_obj", use_spawn_obj))
    {
        // In case the parameter was not created use default
        use_spawn_obj = false;
    }


    // Planning to a Pose goal

    if (use_spawn_obj)
    {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opening Hand");
        change_gripper(&hand_move_group, gripper_state::opened);

        // Move above object
        Quaternionf q = AngleAxisf(3.14, Vector3f::UnitX()) * AngleAxisf(0, Vector3f::UnitY()) * AngleAxisf(0.785, Vector3f::UnitZ());
        auto pose = pose_node->pose;
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going to object pose");
        goto_pose(&move_group, pose);

        // Close gripper
        change_gripper(&hand_move_group, gripper_state::closed);

        // Move above object again
        pose.position.z += 0.1;
        goto_pose(&move_group, pose);

        sensor_msgs::msg::JointState joints;
        joints.name = {"panda_joint1",
                   "panda_joint2",
                   "panda_joint3",
                   "panda_joint4",
                   "panda_joint5",
                   "panda_joint6",
                   "panda_joint7"};
        joints.position= {0.0,
                           -1.75,
                           0.0,
                           -0.1,
                           0.0,
                           3.6,
                           0.8};
        goto_joint_pose(&move_group, joints);

        // Arm trajectory
        joints.position= {0.0,
                           1.0,
                           0.0,
                           -1.1,
                           0.0,
                           1.9,
                           0.8};

        move_group.setJointValueTarget(joints);

        auto current_state = hand_move_group.getCurrentState();
        float gripper_pose = (float)gripper_state::opened / 1000;
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(current_state->getJointModelGroup("hand"), joint_group_positions);
        joint_group_positions[0] = gripper_pose;
        joint_group_positions[1] = gripper_pose;
        hand_move_group.setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        for (int i = 0; i < 10; i++)
        {
            // 10 tries to plan otherwise give up
            bool success = (hand_move_group.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            RCLCPP_INFO(move_group_node->get_logger(), "Plan %d %s", i, success ? "SUCCEEDED" : "FAILED");

            if (success)
            {        
                break;
            }
        }

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

        for (int i = 0; i < 10; i++)
        {
            // 10 tries to plan otherwise give up
            bool success = (move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            RCLCPP_INFO(move_group_node->get_logger(), "Plan %d %s", i, success ? "SUCCEEDED" : "FAILED");

            if (success)
            {
                break;
            }
        }

        auto gripper_traj = gripper_plan.trajectory_.joint_trajectory;
        auto arm_traj = arm_plan.trajectory_.joint_trajectory;

        // Merge hand opening into arm trajectory, such that it is timed for release (at 50%)
        auto release_index = round(0.5*arm_traj.points.size());
        std::cout << arm_traj.joint_names.size() << std::endl;
        for (auto finger_joint : gripper_traj.joint_names)
        {
            arm_traj.joint_names.push_back(finger_joint);
        }
        std::cout << arm_traj.joint_names.size() << std::endl;

        while (arm_traj.points[release_index].effort.size() < 9){
            arm_traj.points[release_index].effort.push_back(0.0);
        }

        for(int i = 0; i < 2; i++)
        { 
            arm_traj.points[release_index].positions.push_back(
                gripper_traj.points[gripper_traj.points.size()-1].positions[i]);
            arm_traj.points[release_index].velocities.push_back(
                gripper_traj.points[gripper_traj.points.size()-1].velocities[i]);
            arm_traj.points[release_index].accelerations.push_back(
                gripper_traj.points[gripper_traj.points.size()-1].accelerations[i]);
        }

        std::thread([&move_group, arm_plan]() { move_group.execute(arm_plan); }).detach();
        //move_group->asyncExecute(plan);

        // Move to default position
        joints.position = {0.0,
                           0.0,
                           0.0,
                           -1.57,
                           0.0,
                           1.57,
                           0.79};
        goto_joint_pose(&move_group, joints);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task completed Succesfully");
    }
    rclcpp::shutdown();
    return 0;
}