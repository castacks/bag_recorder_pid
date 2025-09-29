def pre_logger(node):
    node.get_logger().info("HELLO!!!")
    node.get_logger().info("HELLO!!!")
    node.get_logger().info("HELLO!!!")
    node.get_logger().info("HELLO!!!")
    node.get_logger().info("HELLO!!!")
    node.get_logger().info("HELLO!!!")

def post_logger(node):
    node.get_logger().info(f"DONE 1")
    node.get_logger().info(f"DONE 2")
    node.get_logger().info(f"DONE 3")
    node.get_logger().info(f"DONE 4")
    node.get_logger().info(f"DONE 5")
    node.get_logger().info(f"DONE 6")