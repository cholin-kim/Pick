
## Use when realsense-ros is unavailable or when not using realsense-ros

import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

ctx = rs.context()
devices = ctx.query_devices()
for dev in devices:
    dev.hardware_reset()
print('Reset : done')
# Camera streams
fps = 30

# Initialize D405
pipe_D405 = rs.pipeline()
config_D405 = rs.config()
config_D405.enable_stream(rs.stream.color, rs.format.bgr8, fps)


# Start pipeline
prof_D405 = pipe_D405.start(config_D405)


if not rospy.get_node_uri():
    rospy.init_node("img_publisher")
image_topic = "/camera/color/image_raw"
pub_img = rospy.Publisher(image_topic, Image, queue_size=1)


def stream(visualize=True, publish=False):
    while True:
        frames_D405 = pipe_D405.wait_for_frames()

        color_frame = frames_D405.get_color_frame()
        color_frame = np.asanyarray(color_frame.get_data())

        if visualize:
            cv2.imshow("RealSense", color_frame)

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                cv2.destroyWindow("RealSense")
                pipe_D405.stop()
                break

            if key == ord('c'):     # capture
                cv2.imwrite("d405_capture_occlude2.jpg", color_frame)
                print("capture done")
                quit()

        if publish:
            frame_send = bridge.cv2_to_imgmsg(color_frame, encoding="bgr8")
            pub_img.publish(frame_send)





if __name__ == "__main__":
    stream(visualize=False, publish=True)