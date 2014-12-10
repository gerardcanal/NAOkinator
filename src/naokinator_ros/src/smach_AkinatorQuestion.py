__author__ = 'dani'

import rospy
from smach import StateMachine

from naokinator_ros.srv import akinator_srv
from nao_smach_utils.execute_speechgesture_state import SpeechGesture

class AkinatorQuestion(StateMachine):
    def __init__(self, type):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        with self:
            # TODO integrate movements, speech and retrieval from akinator service
            StateMachine.add('QUESTION',
                             Question(),
                             transitions={'succeeded':'ANSWER'},
                             remapping={'text':'text'}
            )

            StateMachine.add('QUESTION_GESTURE',
                             SpeechGesture(text=self.userdata.text, behavior_name='Asking1'),
                             transitions={'succeeded':'succeeded'})

class Question(smach.State):
    def __init__(self):
        self.naokinatorsrv = rospy.ServiceProxy('/akinator_srv', akinator_srv)

    def execute(self, userdata):
        if (userdata.text is None):
            userdata.text = ''

        userdata.text = self.naokinatorsrv(userdata.text)
        return 'succeeded'