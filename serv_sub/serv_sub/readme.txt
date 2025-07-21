sending data over terminal to a topic /simon
a subscriber  subs to it over the topic /simon
and then calls a service to a server (adding num or something)

ros2 topic pub --once /simon std_msgs/String "data: 'forward right backward left'"

