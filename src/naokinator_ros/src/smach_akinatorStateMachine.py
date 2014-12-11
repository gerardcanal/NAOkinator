__author__ = 'dani'
#!/usr/bin/env python
import rospy
import sys
from smach import StateMachine, State, SimpleActionState, CBState

from naokinator_ros.srv import akinator_srv
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_msgs.msg import WordRecognized

from nao_smach_utils.execute_speechgesture_state import SpeechGesture
from smach_AkinatorQuestion import AkinatorQuestion
from smach_AkinatorAnswer import AkinatorAnswer

class AkinatorGame(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.userdata.is_guess = False
        self.userdata.text = None
        self.userdata.max_questions = 25
        self.userdata.n_questions = 0
        self.userdata.max_repeats = 3
        self.userdata.n_repeats = 0
        with self:
            # TODO integrate movements, timeout, unsuscribing from /word_recognized and maximum questions to WIN/LOSE
            StateMachine.add('QUESTION',
                             AkinatorQuestion(),
                             transitions={'succeeded':'GET_USER_ANSWER'},
                             remapping={'text':'text'}
            )
            StateMachine.add('GET_USER_ANSWER',
                             AkinatorAnswer(),
                             transitions={'succeeded':'ISGUESS',
                                          'aborted':'REPEAT'},
                             remapping={'text':'text', 'is_guess_out':'is_guess'}
            )

            StateMachine.add('ISGUESS',
                             CBState(self.guess_cb, input_keys=['is_guess'], outcomes=['finished', 'continue']),
                             transitions={'continue':'QUESTIONCOUNT',
                                          'finished':'succeeded'},
            )

            StateMachine.add('QUESTIONSCOUNT',
                             #CountdownController(self.userdata.max_questions),
                             CBState(self.countdown_cb, input_keys=['n_questions','max_questions'], outcomes=['succeeded', 'aborted']),
                             transitions={'succeeded':'QUESTION','aborted':'succeeded'},
                             remapping={'count':'n_questions','max':'max_questions'}
            )

            StateMachine.add('REPEAT',
                             SpeechGesture(text='I did not listen you. Can your repeat?', behavior_name='AskingAgain1'),
                             transitions={'succeeded':'GET_USER_ANSWER'})

            StateMachine.add('REPEATCOUNT',
                             #CountdownController(2),
                             CBState(self.countdown_cb, input_keys=['n_repeats','max_repeats'], outcomes=['succeeded', 'aborted']),
                             transitions={'succeeded':'REPEATRESET','aborted':'succeeded'},
                             remapping={'count':'n_repeats','max':'max_repeats'})

            StateMachine.add('REPEATRESET',
                             CBState(self.reset_cb, input_keys=['n_repeats'], outcomes=['succeeded']),
                             transitions={'succeeded':'QUESTION'},
                             remapping={'var':'n_repeats'})


    def guess_cb(ud):
        if ud.is_guess: return 'finished'
        return 'continue'

    def countdown_cb(ud):
        ud.count = ud.count+1
        if ud.counter < ud.max: return 'succeeded'
        return 'aborted'

    def reset_cb(ud):
        ud.var = 0
        return 'succeeded'

'''
class CountdownController(State):
    def __init__(self, max):
        self.max = max

    def execute(self, userdata):
        self.userdata.count = self.userdata.count+1
        if self.userdata.count < self.max:
            return 'succeeded'
        else:
            return 'aborted'
'''