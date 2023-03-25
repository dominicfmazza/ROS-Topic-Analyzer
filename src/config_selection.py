# asciimatics imports
from asciimatics.widgets import (
    Frame,
    Widget,
    FileBrowser,
    TextBox,
    Layout,
    Label,
)
from asciimatics.renderers import FigletText
from asciimatics.exceptions import StopApplication, ResizeScreenError
from asciimatics.event import KeyboardEvent
from asciimatics.screen import Screen
from asciimatics.scene import Scene

# package imports
import os
import rospy
import fnmatch
import json
import re
import sys


# exception to pass out to the main thread for setting config file
class ConfigSet(Exception):
    pass


# exception to catch invalid JSONs
class InvalidJSONConfiguration(Exception):
    pass


# function to list all json files in the config directory
def list_json_files(directory):
    json_files = []

    for root, dirnames, filenames in os.walk(directory):
        for filename in fnmatch.filter(filenames, "*.json"):
            json_files.append(os.path.join(root, filename))

    return json_files


# Function to check if the JSON structure is valid
def is_valid_structure(config, json_data):
    if "topics" in json_data:
        for topic in json_data["topics"]:
            if not all(key in topic for key in ["display_name", "handle", "hz_target"]):
                raise InvalidJSONConfiguration(
                    "Json is not properly formatted,"
                    + "your keys are incorrect."
                    + "\nensure it is formatted like so:"
                    + "\n\t{"
                    + '\n\t\t"topics" : ['
                    + "\n\t\t\t{"
                    + '\n\t\t\t\t"display_name" : "Generic Camera - Images",'
                    + '\n\t\t\t\t"handle" : "/robot/sensors/camera/image",'
                    + '\n\t\t\t\t"hz_target" : "30.0"'
                    + "\n\t\t\t},"
                    + "\n\t\t\t{"
                    + '\n\t\t\t\t"display_name" : "Generic Camera - Camera Info",'
                    + '\n\t\t\t\t"handle" : "/robot/sensors/camera/camera_info",'
                    + '\n\t\t\t\t"hz_target" : "30.0"'
                    + "\n\t\t\t},"
                    + "\n\t\t\t{"
                    + '\n\t\t\t\t"display_name" : "Generic Lidar - Pointcloud",'
                    + '\n\t\t\t\t"handle" : "/robot/sensors/lidar/pointcloud",'
                    + '\n\t\t\t\t"hz_target" : "10.0"'
                    + "\n\t\t\t},"
                    + "\n\t\t]"
                    + "\n\t}"
                    + f"\nIn file: {config}"
                )

    else:
        raise InvalidJSONConfiguration(
            'Json is not properly formatted, "topics" is not found",'
            + "ensure it is formatted like so:"
            + "\n\t{"
            + '\n\t\t"topics" : ['
            + "\n\t\t\t{"
            + '\n\t\t\t\t"display_name" : "Generic Camera - Images",'
            + '\n\t\t\t\t"handle" : "/robot/sensors/camera/image",'
            + '\n\t\t\t\t"hz_target" : "30.0"'
            + "\n\t\t\t},"
            + "\n\t\t\t{"
            + '\n\t\t\t\t"display_name" : "Generic Camera - Camera Info",'
            + '\n\t\t\t\t"handle" : "/robot/sensors/camera/camera_info",'
            + '\n\t\t\t\t"hz_target" : "30.0"'
            + "\n\t\t\t},"
            + "\n\t\t\t{"
            + '\n\t\t\t\t"display_name" : "Generic Lidar - Pointcloud",'
            + '\n\t\t\t\t"handle" : "/robot/sensors/lidar/pointcloud",'
            + '\n\t\t\t\t"hz_target" : "10.0"'
            + "\n\t\t\t},"
            + "\n\t\t]"
            + "\n\t}"
            + f"\nIn file: {config}"
        )

    return True


def is_published_topic(config, handle):
    for topic, _ in rospy.get_published_topics():
        if handle == topic:
            return True

    raise InvalidJSONConfiguration(
        "Handle is not a published_topic. Formatting Example:"
        + "\n/sensors/camera/camera_1"
        + f"\nIn file: {config}"
        + f'\nWith handle option "handle" : "{handle}"'
    )


# Function to check if the handle is in the correct rostopic format
def is_valid_handle(config, handle):
    pattern = re.compile(r"^(/[a-zA-Z0-9_]+)+$")
    if bool(pattern.match(handle)) and is_published_topic(config, handle):
        return True

    raise InvalidJSONConfiguration(
        "Handle formatting is not valid. Formatting Example:"
        + "\n/sensors/camera/camera_1"
        + f"\nIn file: {config}"
        + f'\nWith handle option "handle" : "{handle}"'
    )


# Function to check if hz_target can be converted to a float
def is_valid_hz_target(config, hz_target):
    try:
        float(hz_target)
        return True
    except ValueError:
        raise InvalidJSONConfiguration(
            "HZ formatting is not valid. Make sure it is a float "
            + "surrounded by double quotes:"
            + '\n"0.0", "129.63", etc.'
            + f"\nIn file: {config}"
            + f'\nWith handle option "hz_target" : "{hz_target}"'
        )


# class that provides a selectable list of config files
# in the user's specified config directory
# default directory is topic_analyzer/cfg/
class ConfigView(Frame):
    def __init__(self, screen, directory):
        super(ConfigView, self).__init__(
            screen,
            screen.height // 2,
            screen.width // 2,
            hover_focus=True,
            can_scroll=True,
            title="",
        )
        # variable to save config file for returning to main
        self.config_file = ""

        # set theme to match
        self.set_theme("bright")

        # title in figlet text
        self._title_contents = FigletText("Topic Analyzer", font="speed")

        # create and fill list view
        self._config_list = FileBrowser(
            Widget.FILL_FRAME,
            root=directory,
            file_filter=".*.json$",
        )

        self._error_display = TextBox(height=6, as_string=True)

        # create layout
        layout = Layout([1])

        # add widgets to layout and fix
        self.add_layout(layout)
        layout.add_widget(Label(self._title_contents, height=6))
        layout.add_widget(Label("Config Files", align="^"))
        layout.add_widget(self._config_list)
        layout.add_widget(Label("JSON Configuration Errors", align="^"))
        layout.add_widget(self._error_display)
        layout.add_widget(Label("(q)uit | (s)elect config", align="^"))
        self.fix()

    # Function to process keyboard inputs
    def process_event(self, event):
        if isinstance(event, KeyboardEvent):
            # quit keybind
            if event.key_code in [ord("q"), ord("Q"), Screen.ctrl("c")]:
                raise StopApplication("User quit")
            # select config
            if event.key_code in [ord("s")]:
                self._select_config()
            # Force a refresh for improved responsiveness
            self._last_frame = 0

        # Now pass on to lower levels for normal handling of the event.
        return super(ConfigView, self).process_event(event)

    def _select_config(self):
        config = self._config_list.value
        try:
            if self._validate_config(config):
                # raise config to main
                raise ConfigSet(self._config_list.value)
        except InvalidJSONConfiguration as e:
            self._error_display.value = str(e)
        except IsADirectoryError as e:
            self._error_display.value = str(e)

    # Function to ensure that config files are properly formatted
    # this is done before every topic config is displayed
    def _validate_config(self, config):
        with open(config, "r") as json_file:
            json_data = json.load(json_file)

        # Checks if JSON file is the proper structure
        if is_valid_structure(
            config,
            json_data,
        ):
            for topic in json_data["topics"]:
                handle = topic["handle"]
                hz_target = topic["hz_target"]

                # checks if handle is a valid format for ros
                # and if the hz value is a float as a string
                if is_valid_handle(
                    config,
                    handle,
                ) and is_valid_hz_target(
                    config,
                    hz_target,
                ):
                    continue
            # returns true if there are no issues with the json
            return True


# Function to play the UI
def play(screen, directory):
    screen.play(
        [
            Scene([ConfigView(screen, directory)], -1, name="Config"),
        ],
        stop_on_resize=True,
    )


# Function to wrap the UI and get configs
def run(directory):
    try:
        Screen.wrapper(play, catch_interrupt=True, arguments=(directory,))
        sys.exit(0)
    except ResizeScreenError:
        pass
    # functionality to pass the selected config file to the main thread
    # upon selection
    except ConfigSet as config_file:
        return config_file
