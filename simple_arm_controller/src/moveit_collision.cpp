#include "moveit_controller.hpp"

using namespace std::chrono_literals;

void namer(std::shared_ptr<gazebo_msgs::srv::GetEntityState_Request> request, std::string arg)
{
    request->name = arg;
}
void namer(std::shared_ptr<gazebo_msgs::srv::GetModelList_Request> request, std::string arg) {}

void result_handler(std::shared_future<std::shared_ptr<gazebo_msgs::srv::GetModelList_Response>> result, geometry_msgs::msg::PoseArray *poses)
{
    std::set<std::string> banned{"panda", "ground_plane"};
    for (auto name : result.get()->model_names)
    {
        if (banned.count(name) == 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), name);
            service_caller<gazebo_msgs::srv::GetEntityState>("get_entity_state", poses, name);
        }
    }
}

void result_handler(std::shared_future<std::shared_ptr<gazebo_msgs::srv::GetEntityState_Response>> result, geometry_msgs::msg::PoseArray *poses)
{
    poses->poses.push_back(result.get()->state.pose);
}

void grip_obj(std::shared_ptr<rclcpp::Node> move_group_node, moveit_msgs::msg::CollisionObject obj)
{
    moveit::planning_interface::MoveGroupInterface hand_move_group(move_group_node, "hand");
}

void timer_callback()
{
}
class ObjectivePublisher : public rclcpp::Node
{
public:
    ObjectivePublisher()
        : Node("objective_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("pose_target", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&ObjectivePublisher::timer_callback, this));
    }
    geometry_msgs::msg::Pose pose;

private:
    void timer_callback()
    {

        if (this->count_publishers("pose_target"))
        {
            publisher_->publish(pose);
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("moveit_collision", node_options);
    auto objective_publisher_node = std::make_shared<ObjectivePublisher>();
    // For current state monitor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(objective_publisher_node);
    executor.spin();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface(move_group_node->get_name());
    bool use_spawn_obj;
    if (move_group_node->get_parameter("use_spawn_obj", use_spawn_obj))
    {
        geometry_msgs::msg::PoseArray poses;
        service_caller<gazebo_msgs::srv::GetModelList>("get_model_list", &poses);
        moveit_msgs::msg::CollisionObject collision_object[poses.poses.size()];
        while (true)
        {
            /* code */

            for (int i = 0; i < poses.poses.size(); i++)
            {
                auto pose = poses.poses[i];
                collision_object[i].header.frame_id = "panda_arm";
                collision_object[i].id = "Box" + i;
                shape_msgs::msg::SolidPrimitive primitive;
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                if (i == 0) // table
                {
                    primitive.dimensions[0] = 0.914;
                    primitive.dimensions[1] = 0.5;
                    primitive.dimensions[2] = 0.914;
                    pose.position.z += primitive.dimensions[1] / 2;
                }
                else // cube
                {
                    primitive.dimensions[0] = 0.05;
                    primitive.dimensions[1] = 0.05;
                    primitive.dimensions[2] = 0.05;
                    // pose.position.z += primitive.dimensions[1] / 2;
                }
                if (i == 1)
                {
                    objective_publisher_node->pose = pose;
                }
                

                collision_object[i].primitives.push_back(primitive);
                collision_object[i].primitive_poses.push_back(pose);
                collision_object[i].operation = collision_object[i].ADD;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Add an object into the world");
                planning_scene_interface.applyCollisionObject(collision_object[i]);
            }
        }
    }

    rclcpp::shutdown();
    return 0;
}