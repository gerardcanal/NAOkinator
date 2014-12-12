__author__ = 'dani'
#!/usr/bin/env python

from smach import StateMachine, CBState
from smach_ros import ServiceState

from nao_smach_utils.execute_speechgesture_state import SpeechGesture
from smach_AkinatorQuestion import AkinatorRequestQuestion
from smach_AkinatorAnswer import GetUserAnswer
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehaviorFromPoolSM


class AkinatorGame(StateMachine):

    def __init__(self):
        StateMachine.__init__(self, output_keys=['text'], outcomes=['succeeded', 'aborted', 'preempted'])
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
                             GetUserAnswer(),
                             transitions={'succeeded':'THINKING',
                                          'aborted':'REPEATCOUNT'},
                             remapping={'text':'text'}
                             )
            
            StateMachine.add('THINKING',
                             #SpeechGesture(text='Let me see', behavior_name='CIR_Thinking1'),
                             ExecuteBehaviorFromPoolSM(behavior_pool=['CIR_Thinking1','CIR_Thinking2','CIR_Thinking3','CIR_Thinking4','CIR_Thinking5','CIR_Thinking6']),
                             transitions={'succeeded':'REPEATRESET'})
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
                             transitions={'succeeded':'REPEAT','aborted':'aborted'},
                             remapping={'count':'n_repeats','max':'max_repeats'})

            StateMachine.add('REPEATRESET',
                             CBState(self.reset_cb, output_keys=['var'], outcomes=['succeeded']),
                             transitions={'succeeded':'QUESTIONSCOUNT'},
                             remapping={'var':'n_repeats'})



    def guess_cb(self,ud):
        if ud.is_guess: return 'finished'
        return 'continue'

    def countdown_cb(self,ud):
        ud.count = ud.count+1
        if ud.count < ud.max: return 'succeeded'
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
