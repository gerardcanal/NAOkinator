__author__ = 'dani'

import rospy
from smach import StateMachine, CBState

from nao_smach_utils.speech_recognition_states import StartRecognitionState, StopRecognitionState,GetRecognizedWordNoEmptyList

# yes_words = ['yes', 'maybe']
# no_words = ['no', 'nu', 'mo', 'mu', 'nou', 'now', 'nope']
yes_words = ['yes']
no_words = ['no']
maybe_words = ['maybe']
probably_not_words = ['probably not']
dunno_words = ['i dunno']


class GetUserAnswer(StateMachine):
    def list_cb(self, ud):
        if ud.text.words == []:
            return 'aborted'

        maxc = -1
        rospy.logwarn("Recognized words: " + str(zip(ud.text.words, ud.text.confidence_values)))
        for w, c in zip(ud.text.words, ud.text.confidence_values):
            if c > maxc:
                if w in yes_words:
                    word = 'yes'
                elif w in no_words:
                    word = 'no'
                elif w in maybe_words:
                    word = 'maybe'
                elif w in probably_not_words:
                    word = 'probably not'
                else:
                    word = 'dunno'
                maxc = c
        ud.text = word

        return 'succeeded'

    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                              input_keys=['text'], output_keys=['text'])
        with self:
            StateMachine.add('START_LISTEN',
                             StartRecognitionState(),
                             transitions={'succeeded': 'ANSWER_DETECTION'}
                             )
            StateMachine.add('ANSWER_DETECTION',
                             GetRecognizedWordNoEmptyList(timeout=30),
                             transitions={'succeeded': 'STOP_LISTEN',
                                          'timeouted': 'STOP_LISTEN_ABORTED'},
                             remapping={'recognized_words': 'text'}
                             )

            StateMachine.add('STOP_LISTEN',
                             StopRecognitionState(),
                             transitions={'succeeded': 'GET_SINGLE_RESPONSE'}
                             )

            StateMachine.add('GET_SINGLE_RESPONSE',
                             CBState(self.list_cb, input_keys=['text'], output_keys=['text'], outcomes=['succeeded', 'aborted']),
                             transitions={'succeeded': 'succeeded', 'aborted': 'aborted'}
                             )

            StateMachine.add('STOP_LISTEN_ABORTED',
                             StopRecognitionState(),
                             transitions={'succeeded': 'aborted'}
                             )
