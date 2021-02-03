#include "moveit.hpp"
using std::placeholders::_1;
using namespace Eigen;
using namespace std::chrono_literals;

enum gripper_state {opened=35, closed=0};

bool wait_for_exec(moveit::planning_interface::MoveGroupInterface * move_group, std::shared_ptr<sim_action_server::ActionServer> server)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    for (int i = 0; i < 10; i++)    
    {
        // 10 tries to plan otherwise give up
        bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(server->get_logger(), "Plan %d %s",i,success ? "SUCCEEDED" : "FAILED");

        if (success)
        {
            std::thread([move_group]() { move_group->asyncMove(); }).detach();
            server->execute_plan(plan.trajectory_.joint_trajectory);
            return true;
        }
    }
    auto pose = move_group->getPoseTarget().pose.position;
    RCLCPP_ERROR(server->get_logger(), "Failed to find a valid path to %f, %f, %f", pose.x, pose.y, pose.z);
    throw -1;
    return false;
}

bool change_gripper(moveit::planning_interface::MoveGroupInterface * hand_move_group, std::shared_ptr<sim_action_server::ActionServer> server, gripper_state state)
{
    auto current_state = hand_move_group->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(current_state->getJointModelGroup("hand"), joint_group_positions);
    float pose = (float) state/1000;
    std::cout << state << "," << pose << std::endl;
    joint_group_positions[0] = pose; 
    joint_group_positions[1] = pose; 
    hand_move_group->setJointValueTarget(joint_group_positions);
    return wait_for_exec(hand_move_group, server);

}

bool goto_pose(moveit::planning_interface::MoveGroupInterface * move_group, std::shared_ptr<sim_action_server::ActionServer> server, geometry_msgs::msg::Pose pose)
{   
    move_group->setPoseTarget(pose);
    return wait_for_exec(move_group, server);
}
class SetParam : public rclcpp::Node
{
public:
    SetParam() : Node("set_param")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("stop_updating_obj", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&SetParam::timer_callback, this));

    };
    void set_param(bool new_param)
    {
        param = new_param;
    }

private:
    bool param = false;
    void timer_callback()
    {
        auto message = std_msgs::msg::Bool();
        message.data = param;
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);
    auto parameter_server = std::make_shared<SetParam>();

    // For current state monitor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(parameter_server);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface hand_move_group(move_group_node, "hand");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface("");

    std::string action_node_name;
    bool use_spawn_obj;
    if (!move_group_node->get_parameter("action_node_name", action_node_name))
    {
        // In case the parameter was not created use default
        action_node_name = "/follow_joint_trajectory";
    }
    auto server = std::make_shared<sim_action_server::ActionServer>("trajectory_control", action_node_name);
    // Planning to a Pose goal

    if (move_group_node->get_parameter("use_spawn_obj", use_spawn_obj))
    {
        auto start_pose = move_group.getCurrentPose().pose;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opening Hand");
        change_gripper(&hand_move_group, server, gripper_state::opened);

        std::vector<std::string> targets = {"target"};
        auto collision_objects = planning_scene_interface.getObjects(targets);
        auto collision_object = collision_objects["target"];
        std::vector<std::string> ids = {collision_object.id};
        auto pose = collision_object.primitive_poses[0];
        auto size = collision_object.primitives[0];
        //planning_scene_interface.removeCollisionObjects(ids);
        Quaternionf q = AngleAxisf(3.14, Vector3f::UnitX()) * AngleAxisf(0, Vector3f::UnitY()) * AngleAxisf(0.785, Vector3f::UnitZ());
        
        pose.position.z += 0.1;
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going to pose");
        goto_pose(&move_group, server, pose);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Closing hand");
        parameter_server->set_param(true);
        collision_object.operation = collision_object.REMOVE;
        planning_scene_interface.applyCollisionObject(collision_object);
        move_group.attachObject("target");
        change_gripper(&hand_move_group, server, gripper_state::closed);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going to pose");        
        goto_pose(&move_group, server, start_pose);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opening Hand");
        change_gripper(&hand_move_group, server, gripper_state::opened);
        move_group.detachObject("target");
        parameter_server->set_param(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    rclcpp::shutdown();
    return 0;
}