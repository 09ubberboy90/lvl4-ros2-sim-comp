# include "moveit_controller.hpp"

void namer(std::shared_ptr<gazebo_msgs::srv::GetEntityState_Request> request, std::string arg) { 
    request->name = arg; 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), request->name);
}
void namer(std::shared_ptr<gazebo_msgs::srv::GetModelList_Request> request, std::string arg) {}

void result_handler(std::shared_future<std::shared_ptr<gazebo_msgs::srv::GetModelList_Response>> result, geometry_msgs::msg::PoseArray *poses)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), std::to_string(result.get()->model_names.size()));
    for (auto name : result.get()->model_names)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), name);
        service_caller<gazebo_msgs::srv::GetEntityState>("get_entity_state", poses, name);
    }
}

void result_handler(std::shared_future<std::shared_ptr<gazebo_msgs::srv::GetEntityState_Response>> result, geometry_msgs::msg::PoseArray *poses)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), std::to_string(result.get()->state.pose.position.x));
    poses->poses.push_back(result.get()->state.pose);
}

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);
    // For current state monitor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface(move_group_node->get_name());

    std::string action_node_name;
    bool use_spawn_obj;
    if (!move_group_node->get_parameter("action_node_name", action_node_name))
    {
        // In case the parameter was not created use default
        action_node_name = "/follow_joint_trajectory";
    }
    auto server = std::make_shared<sim_action_server::ActionServer>("trajectory_control", action_node_name);
    std::vector<std::string> joint_names = {"panda_joint1",
                                            "panda_joint2",
                                            "panda_joint3",
                                            "panda_joint4",
                                            "panda_joint5",
                                            "panda_joint6",
                                            "panda_joint7"};
    // Planning to a Pose goal

    if (move_group_node->get_parameter("use_spawn_obj", use_spawn_obj))
    {
        //     moveit_msgs::msg::CollisionObject collision_object[result.get()->poses.poses.size()];
        //     for (int i=0; i< result.get()->poses.poses.size(); i++)
        //     {
        //     }

        // }

        geometry_msgs::msg::PoseArray poses;
        service_caller<gazebo_msgs::srv::GetModelList>("get_model_list", &poses);
        moveit_msgs::msg::CollisionObject collision_object[poses.poses.size()];

        for (int i=0; i< poses.poses.size(); i++)
        {
            auto pose = poses.poses[i];
            collision_object[i].header.frame_id = move_group.getPlanningFrame();
            collision_object[i].id = "Box"+i;
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            if (i == 0) // table
            {
                primitive.dimensions[0] = 0.913;
                primitive.dimensions[1] = 0.913;
                primitive.dimensions[2] = 0.82;
            }
            else // cube
            {
                primitive.dimensions[0] = 0.05;
                primitive.dimensions[1] = 0.05;
                primitive.dimensions[2] = 0.05;

            }

            // Define a pose for the box (specified relative to frame_id)
            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation = pose.orientation;
            box_pose.position = pose.position;
            if (i == 0) // table
            {
                box_pose.position.z += 0.45;
            }
            collision_object[i].primitives.push_back(primitive);
            collision_object[i].primitive_poses.push_back(box_pose);
            collision_object[i].operation = collision_object[i].ADD;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Add an object into the world");
            planning_scene_interface.applyCollisionObject(collision_object[i]);

        }

        // Now, we call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking move_group
        // to actually move the robot.
        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

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