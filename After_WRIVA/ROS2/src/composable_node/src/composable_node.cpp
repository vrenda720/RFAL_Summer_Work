namespace palomino
{
    VincentDriver(const rclcpp::NodeOptions & options) : Node("vincent_driver", options)
    {
        // ...
    };
}

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<palomino::VincentDriver>());
//     rclcpp::shutdown();
//     return 0;
// }

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(palomino::VincentDriver)