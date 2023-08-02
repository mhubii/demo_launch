from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


class TurtleSimMixin:
    @staticmethod
    def arg_node_name() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="node_name",
            default_value="turtlesim_node",
            description="Node name.",
        )

    @staticmethod
    def node_turtlesim(**kwargs) -> Node:
        return Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim_node",
            **kwargs
        )


class HelloWorldMixin:
    @staticmethod
    def node_talker(**kwargs) -> Node:
        return Node(
            package="demo_nodes_cpp", executable="talker", name="talker", **kwargs
        )

    @staticmethod
    def node_listener(**kwargs) -> Node:
        return Node(
            package="demo_nodes_cpp", executable="listener", name="listener", **kwargs
        )
