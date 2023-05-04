import rospy
from std_msgs.msg import Float32MultiArray

class SliderFeedback():
    def __init__(self) -> None:
        super(SliderFeedback, self).__init__()
        self.slider_sub=rospy.Subscriber("/slider_node/result", Float32MultiArray, self.slider_result_callback)
        self.screen_yellow_distance=None
        self.screen_green_distance=1e4
        self.screen_orientation=None
        self.exit_execution = False
        self.slider_exit_condition=0

    def slider_result_callback(self, msg):
        self.screen_yellow_distance = msg.data[0]
        self.screen_green_distance = msg.data[1]
        self.screen_angle = msg.data[2]

        if abs(self.screen_green_distance) < 7 and self.slider_exit_condition:
            self.exit_execution = True
            print("exit from slider")
        else:
            self.exit_execution = False
    

        

