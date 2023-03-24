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
      "display_name": "Name", # a plaintext name for the topic
      "handle": "/can/chassis/brake_torque", # the actual topic address
      "hz_target": "100.0" # a float for a target hz rate
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

```bash
rosrun topic_analyzer topic_analyzer.py
```
