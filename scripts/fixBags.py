#!/usr/bin/python
from rosbag import Bag
import sys

def cleanBagFile(fileName):
    if not fileName.endswith(".bag"):
        return

    newFileName = fileName[:-4] + "-b.bag"

    print fileName, " -> ", newFileName

    newBag = Bag(newFileName, 'w')

    allowedTopics = ["/clock", "/cmd_vel",
                        "/daart_hokuyo/parameter_descriptions", "/daart_hokuyo/parameter_updates",
                        "/diagnostics",
                        "/imu", "/imu_node/parameter_descriptions",
                        "/rosout", "/rosout_agg", "/scan"]
    for topic, msg, t in Bag(fileName):
        if topic == "/odom":
            msg.header.frame_id = "map"
            msg.child_frame_id = "odom"
            newBag.write("/odom_encoder", msg, t)

        # Y.write('/b_topic' if topic == '/a_topic' else topic, msg, t)
        if topic in allowedTopics:
            newBag.write(topic, msg, t)
    newBag.close()

if len(sys.argv) <= 1:
    print "Inform at least one bagfile."
    sys.exit()

for arg in sys.argv[1:]:
    cleanBagFile(arg)
# cleanBagFile('/home/schvarcz/Dropbox/Doutorado-ENSTA/Daart/DaartTests/2017-07-20/2016-02-11-20-19-14.bag')
