__author__ = 'dani'
#!/usr/bin/env python
import rospy
import sys
from smach import StateMachine, State, SimpleActionState

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
        self.userdata.n_questions = 0
        self.userdata.n_repeats = 0
        with self:
            # TODO integrate movements, timeout, unsuscribing from /word_recognized and maximum questions to WIN/LOSE
            StateMachine.add('QUESTION',
                             AkinatorQuestion(),
                             transitions={'succeeded':'ANSWER'},
                             remapping={'text':'text'}
            )
            StateMachine.add('ANSWER',
                             AkinatorAnswer(),
                             transitions={'succeeded':'succeeded' if self.userdata.is_guess else 'QUESTIONSCOUNT',
                                          'aborted':'REPEAT'},
                             remapping={'text':'text', 'is_guess_out':'is_guess'}
            )
            StateMachine.add('QUESTIONSCOUNT',
                             CountdownController(25),
                             transitions={'succeeded':'QUESTION','aborted':'succeeded'},
                             remapping={'count':'n_questions'}
            )
            StateMachine.add('REPEAT',
                             SpeechGesture(text='I did not listen you. Can your repeat?', behavior_name='AskingAgain1'),
                             transitions={'succeeded':'ANSWER'})
            StateMachine.add('REPEATCOUNT',
                             CountdownController(2),
                             transitions={'succeeded':'QUESTION','aborted':'succeeded'},
                             remapping={'count':'n_questions'})
            StateMachine.add('REPEATRESET',
                             Repeat_reset(2),
                             transitions={'succeeded':'QUESTION','aborted':'succeeded'},
                             remapping={'count':'n_questions'})

class CountdownController(smach.State):
    def __init__(self, max):
        self.max = max

    def execute(self, userdata):
        self.userdata.count = self.userdata.count+1
        if self.userdata.count < self.max:
            return 'succeeded'
        else:
            return 'aborted'