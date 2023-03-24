# textual inmports
from node import Node
from database import TopicModel
import display
import config_selection
import rospy
import rospkg
import sys
import threading
import warnings

CONFIG_DIRECTORY = rospkg.RosPack().get_path("topic_analyzer") + "/cfg/"

warnings.filterwarnings(
    action="ignore",
    message="unclosed",
    category=ResourceWarning,
)


def run_node(config, model):
    node = Node(config, model)
    try:
        node.run()
    except rospy.ROSInterruptException:
        sys.exit(0)


if __name__ == "__main__":
    rospy.init_node("topic_analyzer", anonymous=True)
    config = str(config_selection.run(CONFIG_DIRECTORY))
    rospy.logdebug("Creating model")
    model = TopicModel()
    node_thread = threading.Thread(target=run_node, args=(config, model), daemon=True)
    node_thread.start()
    display.run(model)
