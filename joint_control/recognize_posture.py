'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import *
import pickle
from os import listdir
import numpy as np

ROBOT_POSE_CLF = 'robot_pose.pkl'
ROBOT_POSE_DATA_DIR = 'robot_pose_data'

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.class_postures = listdir(ROBOT_POSE_DATA_DIR)
        ## get the classigier
        self.posture_classifier = pickle.load(open(ROBOT_POSE_CLF)) # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        # YOUR CODE HERE
        data = [perception.joint['LHipYawPitch'], perception.joint['LHipRoll'], perception.joint['LHipPitch'], perception.joint['LKneePitch'], perception.joint['RHipYawPitch'], perception.joint['RHipRoll'], perception.joint['RHipPitch'], perception.joint['RKneePitch']]
        data.extend([perception.imu[0], perception.imu[1]])
        data = np.array(data).reshape(1, -1)
        index = self.posture_classifier.predict(data)
        return self.class_postures[index[0]]

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = wipe_forehead('hello')  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
