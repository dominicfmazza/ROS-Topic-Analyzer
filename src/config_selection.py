# asciimatics imports
from asciimatics.widgets import (
    Frame,
    ListBox,
    Widget,
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
import fnmatch
import json
import re
import sys

# exception to pass out to the main thread for setting config file
class ConfigSet(Exception):
    pass


# function to list all json files in the config directory
def list_json_files(directory):
    json_files = []

    for root, dirnames, filenames in os.walk(directory):
        for filename in fnmatch.filter(filenames, "*.json"):
            json_files.append(os.path.join(root, filename))

    return json_files


# Function to check if the JSON structure is valid
def is_valid_structure(json_data):
    if "topics" in json_data:
        for topic in json_data["topics"]:
            if not all(key in topic for key in ["display_name", "handle", "hz_target"]):
                return False
    else:
        return False
    return True


# Function to check if the handle is in the correct rostopic format
def is_valid_handle(handle):
    pattern = re.compile(r"^(/[a-zA-Z0-9_]+)+$")
    return bool(pattern.match(handle))


# Function to check if hz_target can be converted to a float
def is_valid_hz_target(hz_target):
    try:
        float(hz_target)
        return True
    except ValueError:
        return False


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

        # set rows of list
        self._rows = [
            (filename, index + 1)
            for index, filename in enumerate(list_json_files(directory))
        ]

        # create and fill list view
        self._config_list = ListBox(
            Widget.FILL_FRAME,
            self._rows,
            validator=self._validate_config,
        )

        # create layout
        layout = Layout([1])

        # add widgets to layout and fix
        self.add_layout(layout)
        layout.add_widget(Label(self._title_contents, height=6))
        layout.add_widget(Label("Config Files", align="^"))
        layout.add_widget(self._config_list)
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
                raise ConfigSet(
                    self._config_list.options[self._config_list.value - 1][0]
                )

            # Force a refresh for improved responsiveness
            self._last_frame = 0

        # Now pass on to lower levels for normal handling of the event.
        return super(ConfigView, self).process_event(event)

    # Function to ensure that config files are properly formatted
    # this is done before every topic config is displayed
    def _validate_config(self, config):
        config = self._rows[config - 1][0]
        with open(config, "r") as json_file:
            json_data = json.load(json_file)

        # Checks if JSON file is the proper structure
        if is_valid_structure(json_data):
            for topic in json_data["topics"]:
                handle = topic["handle"]
                hz_target = topic["hz_target"]

                # checks if handle is a valid format for ros
                # and if the hz value is a float as a string
                if is_valid_handle(handle) and is_valid_hz_target(hz_target):
                    continue
                else:
                    # return false if the handle/target is invalid
                    return False
            # returns true if there are no issues with the json
            return True
        else:
            # returns false if there are incorrect keys in json
            return False


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
