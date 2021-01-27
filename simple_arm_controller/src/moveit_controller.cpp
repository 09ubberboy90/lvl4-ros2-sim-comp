#include "moveit_controller.hpp"

void grip_obj(std::shared_ptr<rclcpp::Node> move_group_node, moveit_msgs::msg::CollisionObject obj)
{
    moveit::planning_interface::MoveGroupInterface hand_move_group(move_group_node, "hand");

}
class ObjectiveSubscriber : public rclcpp::Node
{
public:
    ObjectiveSubscriber()
        : Node("objective_subscribe")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>("pose_target", 10, std::bind(&ObjectiveSubscriber::topic_callback, this));
    }
    geometry_msgs::msg::Pose pose;

private:
    void topic_callback(const geometry_msgs::msg::Pose pose)
    {
        this->pose = pose;
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);
    auto objective_subscriber_node = std::make_shared<ObjectiveSubscriber>();
    // For current state monitor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(objective_subscriber_node);
    executor.spin();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface hand_move_group(move_group_node, "hand");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface(move_group_node->get_name());

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

        auto start_pose = move_group.getCurrentPose();
        move_group.setPoseTarget(objective_subscriber_node->pose);
//        collision_object[1].header.frame_id = move_group.getEndEffectorLink();
//        move_group.attachObject(collision_object[1].id, "hand");


        // Now, we call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking move_group
        // to actually move the robot.
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        while (true)
        {
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            RCLCPP_INFO(server->get_logger(), "Plan 1 (pose goal) %s", success ? "SUCCEEDED" : "FAILED");

            if (success)
            {
                std::thread([&move_group]() { move_group.asyncMove(); }).detach();
                server->execute_plan(my_plan.trajectory_.joint_trajectory);
                break;
            }

        }
        
        // move_group.setPoseTarget(start_pose);
        // move_group.detachObject(collision_object[1].id);

        // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // RCLCPP_INFO(server->get_logger(), "Plan 1 (pose goal) %s", success ? "SUCCEEDED" : "FAILED");

        // if (success)
        // {
        //     std::thread([&move_group]() { move_group.asyncMove(); }).detach();
        //     server->execute_plan(my_plan.trajectory_.joint_trajectory);
        // }
    }

    rclcpp::shutdown();
    return 0;
}