# ROS Topic Analyzer

This is a ROS node that allows you to monitor many different aspects of
the topics found on your system:

- The current publishing rate in hz
- The publishers and subscribers to a ROS topic
- The message contents of a ROS topic

## Requirements

- [catkin_virtualenv](https://github.com/locusrobotics/catkin_virtualenv)

## Configuration

The node expects to find JSON configuration files in `cfg/` with the following schema:

```json
{
  "topics": [
    {
      "display_name": "Name", // a plaintext name for the topic
      "handle": "/namespace/namespace/topic", // the actual topic address
      "hz_target": "100.0" // a float for a target hz rate
    }
  ]
}
```

If you do not follow this schema, it will throw an error on validation
of the config list.

## Building

```bash
catkin build topic_analyzer
```

## Running
1. First, run the actual ROS node.
```bash
rosrun topic_analyzer topic_analyzer.py
```
2. Then, select a config in the configuration dialog. Use arrow keys to maneuver, and select with the "s". key.
  - If your configuration file is malformed, or if you attempt to monitor topics that aren't online, you will see an error in the dialog box below all your files:

  <script async id="asciicast-570096" src="https://asciinema.org/a/570096.js"></script>
  <script async id="asciicast-570099" src="https://asciinema.org/a/570099.js"></script>

3. You can then see all your online topics, using the arrow keys to select a topic. This will bring up the topic focus view,
  which allows you to see the topic's publishers, subscribers, and message contents.
  <script async id="asciicast-570097" src="https://asciinema.org/a/570097.js"></script>
