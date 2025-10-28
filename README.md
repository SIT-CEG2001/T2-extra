# ROS2 Tutorial 2 (Communication Basics)

## 1. Node

1. See ros2 node help
```
ros2 node -h
```
2. List current running nodes. Right now there should be nothing.
```
ros2 node list
```

3. Run a node from previous lessons.
```
ros2 run turtlesim turtlesim_node
```

4. Now list the nodes again and you should see `/turtlesim`.
```
ros2 node list
```

5. See node info.
```
ros2 node info <node_name>
ros2 node info /turtlesim
```

## 2. Topic

- A topic is one of the ways by which data moves between nodes. 
- One/more publisher node can connect to one/more subscriber nodes via a topic.

1. Open two new terminals and run on each
```
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

2. See list of topics
```
ros2 topic list
# see details 
ros2 topic list -t
```

3. Start rqt graph. Uncheck hide boxes to see hidden topics.
```
rqt_graph
```

4. See topic output. Move with teleop to see output after running command.
```
ros2 topic echo <topic_name>
ros2 topic echo /turtle1/cmd_vel
```

5. View topic info.
```
ros2 topic info <topic_name>
ros2 topic info /turtle1/cmd_vel
```

6. See interface definition
```
ros2 interface show <type>
ros2 interface show geometry_msgs/msg/Twist
```

7. Publish data to a topic (`--once` meand publish once and exit)
```
ros2 topic pub <topic_name> <msg_type> '<args>'
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}'
```

8. Publish data to a topcc with rate 1Hz.
```
ros2 topic pub --rate 0.5 /turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}'
```

## 3. Service

- Services are used to communicate between nodes using a client-server model.
- The server responds when the client makes a request.

1. Open two new terminals and run on each
```
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

2. List all service names
```
ros2 service list

# see service type
ros2 service list -t
```

3. Find service with specific type
```
ros2 service find <service_type>
ros2 service find std_srvs/srv/Empty
```

4. See interface
```
ros2 interface show <service_type>
ros2 interface show turtlesim/srv/Spawn
```

5. Calling a service
```
ros2 service call <service_name> <service_type> <arguments>

# Clear drawing
ros2 service call /clear std_srvs/srv/Empty

# Spawn turtle
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: 'nemo'}"
```

## 4. Parameters

Parameters are values you can change inside a node.

1. Open two new terminals and run on each
```
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

2. See list of parameters
```
ros2 param list
```

3. Get parameter value
```
ros2 param get <node_name> <parameter_name>
ros2 param get /turtlesim background_g
```

4. Set parameter value
```
ros2 param set <node_name> <parameter_name> <value>
ros2 param set /turtlesim background_g 255
```

5. View all param values for a node
```
ros2 param dump <node_name>
ros2 param dump /turtlesim
```

## 5. Actions

Actions let you communicate between nodes with a goal, feedback, and result in a client-server fashion.

1. Open two new terminals and run on each
```
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

2. In the teleop window, do the following and observe:
    a. Press 'g' and see the status when complete.
    b. Press 'g' and cancel early with 'f'.
    c. Press any of the letters, and before it finishes press another letter.
        - You'll see something like this, i.e., the newest action is considered always.
        ```
        Rotation goal received before a previous goal finished. Aborting previous goal
        ```

3. See the action clients using node info
```
ros2 node info /teleop_turtle
```

4. See all actions
```
ros2 action list -t
```

5. See action info
```
ros2 action info <action_name>
ros2 action info /turtle1/rotate_absolute
```

6. See datatype for action
```
ros2 interface show turtlesim/action/RotateAbsolute
```

7. Send action goal
```
ros2 action send_goal <action_name> <action_type> <goal>
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
```

## 6. Publisher Subscriber

We are following the example given in this documentation: [Simple Publisher-Subscriber](https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

### Note:
1. Rewrite the `main()` in `publisher_member_function.py`:
```
def main(args=None):
    rclpy.init(args=args)
    try:
        minimal_publisher = MinimalPublisher()
        rclpy.spin(minimal_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()
```

2. Rewrite the `main()` in `subscriber_member_function.py`:
```
def main(args=None):
    rclpy.init(args=args)
    try:
        minimal_subscriber = MinimalSubscriber()
        rclpy.spin(minimal_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()
```


