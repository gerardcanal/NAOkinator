__author__ = 'dani'
#!/usr/bin/env python

from smach import StateMachine, CBState

from nao_smach_utils.execute_speechgesture_state import SpeechGesture
from smach_AkinatorQuestion import AkinatorRequestQuestion
from smach_AkinatorAnswer import AkinatorAnswer


class AkinatorGame(StateMachine):

    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], output_keys=['win'])
        self.userdata.text = ''
        self.userdata.max_questions = 25
        self.userdata.n_questions = 0
        self.userdata.max_repeats = 3
        self.userdata.n_repeats = 0
        with self:
            # TODO integrate movements, timeout, unsuscribing from /word_recognized and maximum questions to WIN/LOSE

            StateMachine.add('QUESTION',
                             AkinatorRequestQuestion(),
                             transitions={'succeeded':'GET_USER_ANSWER', 'aborted':'succeeded'},
                             remapping={'user_answer':'text','text':'text'}
                             )
            StateMachine.add('GET_USER_ANSWER',
                             AkinatorAnswer(),
                             transitions={'succeeded':'QUESTIONSCOUNT',
                                          'aborted':'REPEAT'},
                             remapping={'text':'text'}
                             )
            '''
            StateMachine.add('ISGUESS',
                             CBState(self.guess_cb, input_keys=['is_guess'], outcomes=['finished', 'continue']),
                             transitions={'continue':'QUESTIONSCOUNT',
                                          'finished':'aborted'},
                             )
            '''
            StateMachine.add('QUESTIONSCOUNT',
                             #CountdownController(self.userdata.max_questions),
                             CBState(self.countdown_cb,
                                     input_keys=['count', 'max'],
                                     output_keys=['count'],
                                     outcomes=['succeeded', 'aborted']),
                             transitions={'succeeded':'QUESTION','aborted':'aborted'},
                             remapping={'count':'n_questions','max':'max_questions'}
                             )

            StateMachine.add('REPEAT',
                             SpeechGesture(text='I did not listen you. Can your repeat?', behavior_name='CIR_AskingAgain1'),
                             transitions={'succeeded':'GET_USER_ANSWER'})

            StateMachine.add('REPEATCOUNT',
                             CBState(self.countdown_cb,
                                     input_keys=['count','max'],
                                     output_keys=['count'],
                                     outcomes=['succeeded', 'aborted']),
                             transitions={'succeeded':'REPEATRESET','aborted':'aborted'},
                             remapping={'count':'n_repeats','max':'max_repeats'})

            StateMachine.add('REPEATRESET',
                             CBState(self.reset_cb, output_keys=['n_repeats'], outcomes=['succeeded']),
                             transitions={'succeeded':'QUESTION'},
                             remapping={'var':'n_repeats'})



    def guess_cb(self,ud):
        if ud.is_guess: return 'finished'
        return 'continue'

    def countdown_cb(self,ud):
        ud.count = ud.count+1
        if ud.counter < ud.max: return 'succeeded'
        return 'aborted'

    def reset_cb(self,ud):
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
