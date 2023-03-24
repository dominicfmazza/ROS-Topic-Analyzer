import json
import rospy
import rostopic
import re
import subprocess
import textwrap
from typing import List

# function to calculate the rate of a ros message using a
# ROSTopicHz instance
def calculate_hz(monitor):
    # Check to make sure timestamps aren't empty
    if not monitor.get_times():
        hz = 0.0
    elif monitor.get_msg_tn() == monitor.get_last_printed_tn():
        hz = 0.0
    # Lock execution to prevent callbacks during the calculation
    with monitor.lock:
        # number of samples
        n = len(monitor.get_times())
        # mean times over the amount of samples
        mean = (sum(monitor.get_times()) / n) if n != 0.0 else 0.0
        # rate in hz
        rate = 1.0 / mean if mean > 0.0 else 0.0
        hz = rate
    return hz


# ros node used
class Node:
    # list of subscribers for monitoring topic hz
    hz_subscribers = []

    # list of ROSTopicHZ instances
    monitors: List = []

    # initializes subscribers and database values
    # based on configuration
    def __init__(self, config_file, model):

        # set member variable for database
        self._model = model

        # open config and add to database/list of topics
        with open(config_file) as f:
            config = json.load(f)
            self.length = len(config["topics"])
            for index, topic in enumerate(config["topics"]):
                # get the message type for the topic
                (msg_object, _, _) = rostopic.get_topic_class(topic["handle"])

                # creates a ROSTopicHZ class to monitor topic HZ for
                # the last 100 messages
                monitor = rostopic.ROSTopicHz(100)

                # subscribes to topic to monitor hz
                hz_subscriber = rospy.Subscriber(
                    topic["handle"],
                    msg_object,
                    monitor.callback_hz,
                )

                # create a dictionary to be added to database and adds
                topic = {
                    "title": topic["display_name"],
                    "handle": topic["handle"],
                    "rate": "0.0",
                    "target": topic["hz_target"],
                    "type": msg_object._type,
                }
                self._model.add(topic)

                # add monitor to the list
                self.monitors.append(monitor)

                # add subscriber to the list
                self.hz_subscribers.append(hz_subscriber)
        # timer callback to update rate and information 4 times a second
        self.timer = rospy.Timer(rospy.Duration(0.25), self.timer_callback)

    # running loop for the rosnode
    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

    # callback to update database values 4 times a second
    def timer_callback(self, timer):

        # if there is no currently selected topic,
        # just update the hz for every topic
        if self._model.current_id is None:
            for index in range(self.length):
                hz = self.get_hz(index)
                self._model.update_topic_hz(f"{hz:.2f}", index + 1)

        # if there is a currently selected topic, get its message and its
        # publishers/subscribers as well as only updating the topic's hz
        if self._model.current_id is not None:
            message = ""
            publishers = ""
            subscribers = ""
            try:
                # get hz for topic
                hz = self.get_hz(self._model.current_id - 1)

                # update the hz
                self._model.update_topic_hz(f"{hz:.2f}", self._model.current_id - 1)

                # gets current topic from model
                topic = self._model.get_current_topic()

                # waits for a singular message
                message = rospy.wait_for_message(
                    topic[2],
                    self.hz_subscribers[self._model.current_id - 1].data_class,
                    timeout=5,
                )

                # makes sure the messages wrap correctly
                message = "\n".join(
                    [
                        "\n".join(
                            textwrap.wrap(
                                line,
                                70,
                                break_long_words=False,
                                replace_whitespace=False,
                            )
                        )
                        for line in str(message).splitlines()
                        if line.strip() != ""
                    ]
                )

                # gets pub and subs for topic
                publishers, subscribers = self.get_pubsubs(topic[2])

            # catch errors related to topic not being up
            except Exception:
                message = "TOPIC NOT ONLINE\n"
                publishers = "TOPIC NOT ONLINE\n"
                subscribers = "TOPIC NOT ONLINE\n"

            # update database
            self._model.update_topic_info(
                message,
                publishers,
                subscribers,
            )

    # gets hz for a topic by its index
    def get_hz(
        self,
        index: int,
    ):
        # rospy.loginfo("Calling node:get_hz()")
        return calculate_hz(self.monitors[index])

    # gets publishers and subscribers by stripping the output of
    # rostopic info
    def get_pubsubs(
        self,
        handle,
    ):
        # get output of rostopic info
        output = subprocess.check_output(["rostopic", "info", handle])

        # split string to separate publishers and subscribers
        split_output = output.decode().split(": \n")

        # search a string for nodes in output of rostopic info
        def get_nodes_from_output(output_str):
            nodes = ""

            # find all matches for the node format
            matches = re.finditer(
                r" \* ([/a-zA-Z0-9_]*) \(.*\)",
                split_output[1],
            )
            # creates string
            for match in matches:
                nodes += match.group(1) + "\n"
            return nodes

        # find publishers in string
        publisher_string = get_nodes_from_output(split_output[1])
        subscriber_string = get_nodes_from_output(split_output[2])
        return publisher_string, subscriber_string
