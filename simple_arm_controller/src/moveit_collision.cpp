#include "moveit.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

void namer(std::shared_ptr<gazebo_msgs::srv::GetEntityState_Request> request, std::string arg)
{
    request->name = arg;
}
void namer(std::shared_ptr<gazebo_msgs::srv::GetModelList_Request> request, std::string arg) {}

void result_handler(std::shared_ptr<rclcpp::Node> node, std::shared_future<std::shared_ptr<gazebo_msgs::srv::GetModelList_Response>> result, geometry_msgs::msg::PoseArray *poses)
{
    std::set<std::string> banned{"panda", "ground_plane"};
    for (auto name : result.get()->model_names)
    {
        if (banned.count(name) == 0)
        {
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), name);
            service_caller<gazebo_msgs::srv::GetEntityState>(node, "get_entity_state", poses, name);
        }
    }
}

void result_handler(std::shared_ptr<rclcpp::Node> node, std::shared_future<std::shared_ptr<gazebo_msgs::srv::GetEntityState_Response>> result, geometry_msgs::msg::PoseArray *poses)
{
    poses->poses.push_back(result.get()->state.pose);
}

class GetParam : public rclcpp::Node
{
public:
    GetParam() : Node("get_global_param")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Bool>("stop_updating_obj", 10, std::bind(&GetParam::topic_callback, this, _1));
    };
    bool get_param()
    {
        return param;
    }

private:
    bool param = false;
    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        param = msg->data;
    }
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("moveit_collision", node_options);
    auto parameter_server = std::make_shared<GetParam>();
    auto service_node = rclcpp::Node::make_shared("service_handler");
    // For current state monitor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(parameter_server);
    std::thread([&executor]() { executor.spin(); }).detach();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface("");

    bool use_spawn_obj;
    if (move_group_node->get_parameter("use_spawn_obj", use_spawn_obj))
    {

        while (true)
        {

            geometry_msgs::msg::PoseArray poses;
            std::vector<moveit_msgs::msg::CollisionObject> collision_object;
            service_caller<gazebo_msgs::srv::GetModelList>(service_node, "get_model_list", &poses);
            for (int i = 0; i < poses.poses.size(); i++)
            {
                moveit_msgs::msg::CollisionObject obj;
                auto pose = poses.poses[i];
                obj.header.frame_id = "world";
                obj.id = "Box" + i;
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
                    obj.id = "target";
                    if (parameter_server->get_param())
                    {
                        continue;
                    }
                }

                obj.primitives.push_back(primitive);
                obj.primitive_poses.push_back(pose);
                obj.operation = obj.ADD;
                collision_object.push_back(obj);
            }
            planning_scene_interface.addCollisionObjects(collision_object);
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Add an object into the world");
        }
    }

    rclcpp::shutdown();
    return 0;
}