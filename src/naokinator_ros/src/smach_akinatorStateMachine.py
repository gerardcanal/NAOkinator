__author__ = 'dani'
#!/usr/bin/env python

from smach import StateMachine, CBState

from nao_smach_utils.execute_speechgesture_state import SpeechGesture
from smach_AkinatorQuestion import AkinatorRequestQuestion
from smach_AkinatorAnswer import GetUserAnswer
from concurrent_userspeech_jointtrajectory import SpeechRecognitionAndGesture
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehaviorFromPoolSM


class AkinatorGame(StateMachine):

    def __init__(self):
        StateMachine.__init__(self, output_keys=['text'], outcomes=['succeeded', 'aborted', 'preempted'])
        self.userdata.text = ''
        self.userdata.max_questions = 25
        self.userdata.n_questions = 0
        self.userdata.max_repeats = 3
        self.userdata.n_repeats = 0
        list_thinking = ['Let me think', 'Let me see', 'So', 'I think I am getting near', 'Uff, that is difficult', 'well',
                         'I do not know what to ask you now', 'I think I am getting near.', 'Wait a second', 'Wait a moment',
                         'Well. Hum', 'Um. Well', 'No clue about it', 'I think I may know it', 'Well, maybe is this.']
        list_repeating = ['I did not hear you well, can you repeat please?', 'Can you repeat your answer please?', 'What?',
                          'Pardon?', 'I beg your pardon, could you repeat?', 'Again? What did you say?', 'Tell me again, please',
                          'Which is your answer?', 'Repeat the asnwer please?', 'Did you say something?', 'And your answer is?']
        with self:
            # TODO integrate movements, timeout, unsuscribing from /word_recognized and maximum questions to WIN/LOSE

            StateMachine.add('QUESTION',
                             AkinatorRequestQuestion(),
                             transitions={'succeeded': 'GET_USER_ANSWER', 'aborted': 'succeeded'},
                             remapping={'user_answer': 'text', 'text': 'text'})
            StateMachine.add('GET_USER_ANSWER',
                             SpeechRecognitionAndGesture(),
                             transitions={'succeeded': 'THINKING',
                                          'aborted': 'REPEATCOUNT'},
                             remapping={'text': 'text'}
                             )

            ThinkingPool = ['CIR_Thinking1', 'CIR_Thinking2', 'CIR_Thinking3',
                            'CIR_Thinking4', 'CIR_Thinking5', 'CIR_Thinking6',
                            'CIR_Thinking7', 'CIR_Thinking8', 'CIR_Thinking9',
                            'CIR_Thinking10', 'CIR_Thinking11']
            StateMachine.add('THINKING',
                             SpeechGesture(textpool=list_thinking,
                                           behavior_pool=ThinkingPool, wait_before_speak=2.5),
                             #ExecuteBehaviorFromPoolSM(behavior_pool=['CIR_Thinking1','CIR_Thinking2','CIR_Thinking3','CIR_Thinking4','CIR_Thinking5','CIR_Thinking6']),
                             transitions={'succeeded': 'REPEATRESET'})

            StateMachine.add('QUESTIONSCOUNT',
                             #CountdownController(self.userdata.max_questions),
                             CBState(self.countdown_cb,
                                     input_keys=['count', 'max'],
                                     output_keys=['count'],
                                     outcomes=['succeeded', 'aborted']),
                             transitions={'succeeded': 'QUESTION', 'aborted': 'aborted'},
                             remapping={'count': 'n_questions', 'max': 'max_questions'}
                             )

            StateMachine.add('REPEAT',
                             SpeechGesture(textpool=list_repeating, behavior_pool=['CIR_AskingAgain1', 'CIR_AskingAgain2', 'CIR_AskingAgain3']),
                             #SpeechGesture(text='I did not listen you. Can your repeat?', behavior_name='CIR_AskingAgain1'),
                             transitions={'succeeded': 'GET_USER_ANSWER'})

            StateMachine.add('REPEATCOUNT',
                             CBState(self.countdown_cb,
                                     input_keys=['count', 'max'],
                                     output_keys=['count'],
                                     outcomes=['succeeded', 'aborted']),
                             transitions={'succeeded': 'REPEAT', 'aborted': 'aborted'},
                             remapping={'count': 'n_repeats', 'max': 'max_repeats'})

            StateMachine.add('REPEATRESET',
                             CBState(self.reset_cb, output_keys=['var'], outcomes=['succeeded']),
                             transitions={'succeeded': 'QUESTIONSCOUNT'},
                             remapping={'var': 'n_repeats'})



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
