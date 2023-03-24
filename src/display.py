# asciimatics imports
from asciimatics.widgets import (
    Frame,
    MultiColumnListBox,
    Widget,
    Layout,
    TextBox,
    Text,
    Divider,
    Label,
    VerticalDivider,
)
from asciimatics.parsers import AsciimaticsParser
from asciimatics.exceptions import StopApplication, ResizeScreenError, NextScene
from asciimatics.event import KeyboardEvent
from asciimatics.screen import Screen
from asciimatics.scene import Scene

# package imports
import sys

# class for focusing a selected topic and displaying its
# message contents, publishers, and subscribers in addition
# to the information shown in a list view
class FocusView(Frame):
    # header
    _header = TextBox(1, as_string=True, disabled=True)

    # plaintext description of message
    _message_title = Text("Title:", disabled=True)

    # ROS address of message
    _handle = Text("Handle:", disabled=True)

    # current rate in HZ
    _rate = Text("Rate (hz):", disabled=True)

    # rate target in HZ
    _target = Text("Target (hz):", disabled=True)

    # type of message
    _type = Text("Type:", disabled=True)

    # contents of ROS message from topic
    _message_contents = TextBox(
        Widget.FILL_COLUMN,
        label="Contents:",
        as_string=True,
        readonly=True,
        line_wrap=True,
    )

    # create and set layout of display
    def __init__(self, screen, model):
        # initialize the frame itself
        super(FocusView, self).__init__(
            screen,  # screen object to use
            screen.height - 10,  # ui height
            screen.width - 10,  # ui width
            hover_focus=True,  # whether or not focus follows mous
            can_scroll=True,  # allows the user to scroll
            title="Topic Analyzer: Focus View",  # title to be displayed
        )

        # custom palette for Text and TextBox components,
        # as they display in a grey that is hard to read
        # by default
        self.palette["topic_focus"] = (
            Screen.COLOUR_GREEN,
            Screen.COLOUR_WHITE,
            Screen.COLOUR_BLACK,
        )

        # setting custom palette for each Text and TextBox widget
        self._message_title.custom_colour = "topic_focus"
        self._handle.custom_colour = "topic_focus"
        self._rate.custom_colour = "topic_focus"
        self._target.custom_colour = "topic_focus"
        self._type.custom_colour = "topic_focus"
        self._message_contents.custom_colour = "topic_focus"

        # last updated frame
        self._last_frame = 0

        # set theme
        self.set_theme("bright")

        # set the database for the view
        self._model = model
        column_width = ((screen.width - 10) - 1) // 2

        # create layouts
        header_layout = Layout([1])
        body_layout = Layout([column_width, 1, column_width - 1], fill_frame=True)
        footer_layout = Layout([1])

        # add layout for header
        self.add_layout(header_layout)

        # add header
        header_layout.add_widget(self._header)

        # add layout for body
        self.add_layout(body_layout)

        # section title
        body_layout.add_widget(Label("Topic Information", align="^"))

        # add general information in the top left
        body_layout.add_widget(self._message_title, 0)
        body_layout.add_widget(self._handle, 0)
        body_layout.add_widget(self._rate, 0)
        body_layout.add_widget(self._target, 0)

        # add divider between info/pubsub
        body_layout.add_widget(Divider(), 0)
        # calculate height for publishers and subscribers
        pubsub_height = screen.height // 2 - 10

        # section title
        body_layout.add_widget(Label("Publishers and Subscribers", align="^"), 0)

        # add publishers (declared here because of shared height
        # that needs the screen variable)
        self._publishers = TextBox(
            pubsub_height,
            label="Publishers:",
            disabled=True,
            as_string=True,
            readonly=True,
        )
        self._publishers.custom_colour = "topic_focus"
        body_layout.add_widget(
            self._publishers,
            0,
        )

        # add subscribers (declared here because of shared height
        # that needs the screen variable)
        self._subscribers = TextBox(
            pubsub_height,
            label="Subscribers:",
            disabled=True,
            as_string=True,
            readonly=True,
        )
        self._subscribers.custom_colour = "topic_focus"
        body_layout.add_widget(
            self._subscribers,
            0,
        )

        # section title
        body_layout.add_widget(Label("Message", align="^"), 2)

        # type above message
        body_layout.add_widget(self._type, 2)

        # add box for message contents
        self._message_contents.auto_scroll = False
        body_layout.add_widget(
            self._message_contents,
            2,
        )

        # add vertical divider
        body_layout.add_widget(VerticalDivider(), 1)

        # add footer layout
        self.add_layout(footer_layout)

        # add hotkeys
        footer_layout.add_widget(Label("(q)uit | (r)eturn to list", align="^"))
        self.fix()

    # handles keyboard inputs
    def process_event(self, event):
        # Do the key handling for this Frame.
        if isinstance(event, KeyboardEvent):
            if event.key_code in [ord("q"), ord("Q"), Screen.ctrl("c")]:
                raise StopApplication("User quit")
            if event.key_code == ord("r"):
                self._return()
            # Force a refresh for improved responsiveness
            self._last_frame = 0

        # Now pass on to lower levels for normal handling of the event.
        return super(FocusView, self).process_event(event)

    # clears content in view before moving back to list view
    # if this isn't done, focus view will not reset properly and
    # will display an incorrect topic for a few frames
    def _return(self):
        # clearing component contents
        self._message_title.value = ""
        self._handle.value = ""
        self._rate.value = ""
        self._target.value = ""
        self._type.value = ""
        self._message_contents.value = ""
        self._publishers.value = ""
        self._subscribers.value = ""

        # forcing a refresh on next call to focus view
        self._last_frame = 0

        # saves focus view state
        self.save()

        # ensures that the database's current_id is None so that
        # the ros node knows which data to query now
        self._model.current_id = None

        # switch scene to list view
        raise NextScene("List")

    # custom update function for the
    def _update(self, frame_no):
        # rospy.logdebug("Updating display")
        # Refresh the list view if its been too long or if _last_frame
        # has been set to 0 to force refresh
        if (
            frame_no - self._last_frame >= self.frame_update_count
            or self._last_frame == 0
        ):
            # rospy.logdebug("Frame check satisfied")
            # saving values
            self._last_frame = frame_no

            # querying topic info
            topic = self._model.get_current_topic()

            # assign values
            self._message_title.value = topic[1]
            self._handle.value = topic[2]
            self._rate.value = topic[3]
            self._target.value = topic[4]
            self._type.value = topic[5]
            self._message_contents.value = topic[6]
            self._publishers.value = topic[7]
            self._subscribers.value = topic[8]

        # let the Frame class handle the rest of the updating
        super(FocusView, self)._update(frame_no)

    # sets the update rate
    # as of right now, set to 10 frames (20 frames
    # a second is the refresh rate for asciimatics uis)
    @property
    def frame_update_count(self):
        # Refresh once every 2 seconds by default.
        return 10


# defines the ui for the list of all topics in the config
class ListView(Frame):
    def __init__(self, screen, model):
        super(ListView, self).__init__(
            screen,
            screen.height - 10,
            screen.width - 10,
            hover_focus=True,
            can_scroll=True,
            title="Topic Analyzer: List View",
        )

        # divides view by 5 so that the list column width
        # can be uniformly set
        standard_width = (screen.width - 10) / 5

        # set theme of app
        self.set_theme("bright")

        # Internal state required for doing periodic updates
        self._last_frame = 0

        # sql database for topics
        self._model = model

        # Create layout
        layout = Layout([1], fill_frame=True)

        # create header and set attributes
        self._header = TextBox(1, as_string=True)
        self._header.disabled = True
        self._header.custom_colour = "label"

        # Create the form for displaying the list of contacts.
        self._list_view = MultiColumnListBox(
            Widget.FILL_FRAME,
            [
                f"<{standard_width}",  # titles don't need to be that long
                f"<{standard_width*1.5}",  # topics may be lengthy
                f">{standard_width/2}",  # justified right to be close to the target hz
                f"<{standard_width/2}",  # hz is a small value
                f"<{standard_width*1.4}",  # message types given extra room to fill
            ],
            [],  # initialize with empty rows, will query from data base
            titles=[
                "TITLE",
                "HANDLE",
                "RATE (hz)",
                "TARGET (hz)",
                "TYPE",
            ],
            name="topic_list",
            parser=AsciimaticsParser(),
        )
        # add layout
        self.add_layout(layout)

        # add widgets
        layout.add_widget(self._header)
        layout.add_widget(self._list_view)
        layout.add_widget(Label("(q)uit | (s)elect topic", align="^"))
        self.fix()

    # process keyboard input
    def process_event(self, event):
        # Do the key handling for this Frame.
        if isinstance(event, KeyboardEvent):
            # quit application
            if event.key_code in [ord("q"), ord("Q"), Screen.ctrl("c")]:
                raise StopApplication("User quit")
            # select a topic
            if event.key_code in [ord("s")]:
                self._focus_topic()

            # Force a refresh for improved responsiveness
            self._last_frame = 0

        # Now pass on to lower levels for normal handling of the event.
        return super(ListView, self).process_event(event)

    # properly formats rows for the MultiColumnListBox,
    # expects a tuple with ([contents...], index), with
    # index starting from 0
    def process_data(self, data):
        new_data = []
        for x in data:
            new_data.append(([thing for thing in x[0:5]], x[-1]))
        return new_data

    # custom defined update functionality
    def _update(self, frame_no):
        # rospy.logdebug("Updating display")
        # Refresh the list view if needed
        if (
            frame_no - self._last_frame >= self.frame_update_count
            or self._last_frame == 0
        ):
            # rospy.logdebug("Frame check satisfied")
            # saving values
            self._last_frame = frame_no
            last_selection = self._list_view.value
            last_start = self._list_view.start_line

            # query the database for a summary of all topics
            data = self._model.get_summary()

            # if self._list_view.options == self.process_data(data):
            # rospy.logdebug("Options match")

            # assign values
            self._list_view.options = self.process_data(data)
            self._list_view.value = last_selection
            self._list_view.start_line = last_start

        super(ListView, self)._update(frame_no)

    # switch to topic focus view
    def _focus_topic(self):
        # rospy.logdebug("Saving")

        # save the current state of the list
        self.save()

        # rospy.logdebug("Setting value")

        # set the currently selected topic in the database for
        # the ros node
        self._model.current_id = self._list_view.value

        # rospy.logdebug("Setting next scene")

        # switch scenes
        raise NextScene("Focus")

    # set refresh rate, current at 4 times a second
    @property
    def frame_update_count(self):
        return 5

    @staticmethod
    def _quit():
        raise StopApplication("User pressed quit")


# function to set the display up with both list and focus view
def play(screen, model):
    screen.play(
        [
            Scene([ListView(screen, model)], -1, name="List"),
            Scene([FocusView(screen, model)], -1, name="Focus"),
        ],
        stop_on_resize=True,
    )


# run the display, passing in the database
def run(model):
    try:
        # wrapper function for the screen
        Screen.wrapper(play, catch_interrupt=True, arguments=(model,))
        sys.exit(0)
    # allows resizing of the screen
    except ResizeScreenError:
        pass
