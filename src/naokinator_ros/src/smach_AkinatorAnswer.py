__author__ = 'dani'

import rospy
import sys
from smach import StateMachine,State

from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_msgs.msg import WordRecognized

from nao_smach_utils.execute_speechgesture_state import SpeechGesture
from nao_smach_utils.speech_recognition_states import StartRecognitionState,StopRecognitionState,GetRecognizedWord

class AkinatorAnswer(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        with self:
            '''
            StateMachine.add('Answer',
                             Answer(),
                             transitions={'succeeded':'THINKING', 'aborted':'aborted'},
                             remapping={'text':'text'}
            )
            '''
            StateMachine.add('START_LISTEN',
                             StartRecognitionState(),
                             transitions={'succeeded':'ANSWER_DETECTION'}
            )
            StateMachine.add('ANSWER_DETECTION',
                             GetRecognizedWord(timeout=30),
                             transitions={'succeeded':'THINKING',
                                          'timeouted':'STOP_LISTEN_ABORTED'},
                             remapping={'recognized_words':'text'}
            )

            StateMachine.add('THINKING',
                             SpeechGesture(text='Let me see...', behavior_name='Thinking1'),
                             transitions={'succeeded':'STOP_LISTEN_SUCCEEDED'})

            StateMachine.add('STOP_LISTEN_SUCCEEDED',
                             StopRecognitionState(),
                             transitions={'succeeded':'succeeded'}
            )
            StateMachine.add('STOP_LISTEN_ABORTED',
                             StopRecognitionState(),
                             transitions={'succeeded':'aborted'}
            )