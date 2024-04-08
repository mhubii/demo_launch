from launch import LaunchDescription
from launch_mixins.demo_launch import HelloWorldMixin, TurtleSimMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # add turtlesim
    ld.add_action(TurtleSimMixin.arg_node_name())
    ld.add_action(TurtleSimMixin.node_turtlesim())

    # add hello world
    ld.add_action(HelloWorldMixin.node_talker())
    ld.add_action(HelloWorldMixin.node_listener())

    return ld
