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
            "/joint_states", 10, std::bind(&GetPose::listener_callback, this, std::placeholders::_1));

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

    bool use_spawn_obj;
    if (!move_group_node->get_parameter("use_spawn_obj", use_spawn_obj))
    {
        // In case the parameter was not created use default
        use_spawn_obj = false;
    }

    // Planning to a Pose goal
    if (use_spawn_obj)
    {
        auto start_pose = move_group.getCurrentPose().pose;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opening Hand");
        change_gripper(&hand_move_group, gripper_state::opened);

        auto pose = pose_node->pose;
        Quaternionf q = AngleAxisf(3.14, Vector3f::UnitX()) * AngleAxisf(0, Vector3f::UnitY()) * AngleAxisf(0.785, Vector3f::UnitZ());

        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going to object pose");
        goto_pose(&move_group, pose);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Closing hand");
        change_gripper(&hand_move_group,  gripper_state::closed);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lifting");
        pose.position.z += 0.1;
        goto_pose(&move_group,  pose);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going to target pose");
        pose.position.x += 0.2;
        goto_pose(&move_group,  pose);

        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lowering");
        // pose.position.z -= 0.1;
        // goto_pose(&move_group, server, pose);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opening Hand");
        change_gripper(&hand_move_group,  gripper_state::opened);


        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going to start pose");
        goto_pose(&move_group,  start_pose);
        auto new_pose = pose_node->pose;
        std::cout << new_pose.position.x << "," << pose.position.x << std::endl;
        std::cout << new_pose.position.y << "," << pose.position.y << std::endl;
        std::cout << new_pose.position.z << "," << pose.position.z << std::endl;

        if ((new_pose.position.x < pose.position.x - 0.05) || (pose.position.x + 0.05 < new_pose.position.x))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cube is not in bound");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task completed Succesfully");
        }

    }
    rclcpp::shutdown();
    return 0;
}