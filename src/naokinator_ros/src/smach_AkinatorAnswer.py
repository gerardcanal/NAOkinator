__author__ = 'dani'

import rospy
from smach import StateMachine, CBState

from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_msgs.msg import WordRecognized

from nao_smach_utils.execute_speechgesture_state import SpeechGesture
from nao_smach_utils.speech_recognition_states import GetRecognizedWord


class AkinatorAnswer(StateMachine):
    yes_words = ['yes', 'si', 'se', 'yea', 'yeah', 'see', 'sea', 'yi', 'yees', 'probably', 'maybe']
    no_words = ['no', 'nu', 'mo','mu','nou', 'now', 'nope']

    def list_cb(self,ud):
        maxc = -1
        rospy.logwarn("Recognized words: " + str(zip(ud.text.words, ud.text.confidence_values)))
        for w, c in zip(ud.text.words, ud.text.confidence_values):
            if c > maxc:
                word = 'yes' if w in self.yes_words else 'no'
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
            StateMachine.add('ANSWER_DETECTION',
                             GetRecognizedWord(timeout=30),
                             transitions={'succeeded':'GET_SINGLE_RESPONSE',
                                          'timeouted':'aborted'},
                             remapping={'recognized_words':'text'}
                             )

            StateMachine.add('GET_SINGLE_RESPONSE',
                             CBState(self.list_cb, input_keys=['text'], output_keys=['text'], outcomes=['succeeded']),
                             transitions={'succeeded':'THINKING'},
                            )

            StateMachine.add('THINKING',
                             SpeechGesture(text='Let me see...', behavior_name='CIR_Thinking1'),
                             transitions={'succeeded':'succeeded'})
