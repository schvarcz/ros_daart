from rosbag import Bag

newBag = Bag('/home/schvarcz/Dropbox/Doutorado-ENSTA/Daart/DaartTests/2017-07-20/2016-02-11-20-19-14-b.bag', 'w')

allowedTopics = ["/clock", "/cmd_vel",
                    "/daart_hokuyo/parameter_descriptions", "/daart_hokuyo/parameter_updates",
                    "/diagnostics",
                    "/imu", "/imu_node/parameter_descriptions",
                    "/rosout", "/rosout_agg", "/scan"]
for topic, msg, t in Bag('/home/schvarcz/Dropbox/Doutorado-ENSTA/Daart/DaartTests/2017-07-20/2016-02-11-20-19-14.bag'):
    if topic == "/odom":
        msg.header.frame_id = "map"
        msg.child_frame_id = "odom"
        newBag.write("/odom_encoder", msg, t)

    # Y.write('/b_topic' if topic == '/a_topic' else topic, msg, t)
    if topic in allowedTopics:
        newBag.write(topic, msg, t)
newBag.close()
