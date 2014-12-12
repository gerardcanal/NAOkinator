__author__ = 'dani'

import rospy
from smach import StateMachine, CBState

from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_msgs.msg import WordRecognized

from nao_smach_utils.speech_recognition_states import StartRecognitionState, StopRecognitionState,GetRecognizedWordNoEmptyList

yes_words = ['yes', 'maybe']
no_words = ['no', 'nu', 'mo','mu','nou', 'now', 'nope']


class GetUserAnswer(StateMachine):
    def list_cb(self,ud):
        if ud.text.words == []:
            return 'aborted'

        maxc = -1
        rospy.logwarn("Recognized words: " + str(zip(ud.text.words, ud.text.confidence_values)))
        for w, c in zip(ud.text.words, ud.text.confidence_values):
            if c > maxc:
                word = 'yes' if w in yes_words else 'no'
                maxc = c
        ud.text = word

        return 'succeeded'

    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['text'],output_keys=['text'])
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
                             transitions={'succeeded': 'ANSWER_DETECTION'}
                             )
            StateMachine.add('ANSWER_DETECTION',
                             GetRecognizedWordNoEmptyList(timeout=30),
                             transitions={'succeeded':'STOP_LISTEN',
                                          'timeouted':'STOP_LISTEN_ABORTED'},
                             remapping={'recognized_words':'text'}
                             )

            StateMachine.add('STOP_LISTEN',
                             StopRecognitionState(),
                             transitions={'succeeded':'GET_SINGLE_RESPONSE'}
                             )

            StateMachine.add('GET_SINGLE_RESPONSE',
                             CBState(self.list_cb, input_keys=['text'], output_keys=['text'], outcomes=['succeeded','aborted']),
                             transitions={'succeeded':'succeeded', 'aborted' : 'aborted'}
                            )

            StateMachine.add('STOP_LISTEN_ABORTED',
                             StopRecognitionState(),
                             transitions={'succeeded':'aborted'}
                             )
