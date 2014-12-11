#!/usr/bin/env python
import rospy

from smach import StateMachine

from smach_akinatorStateMachine import AkinatorGame
from nao_smach_utils.execute_speechgesture_state import SpeechGesture
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehavior
from nao_smach_utils.home_onoff import HomeOn_SM
from nao_smach_utils.stiffness_states import DisableStiffnessState
from nao_smach_utils.tts_state import SpeechState
from nao_smach_utils.go_to_posture_state import GoToPostureState
from nao_smach_utils.speech_recognition_states import StartRecognitionState,StopRecognitionState


class StartNaokinator(StateMachine):

    def __init__(self, driver=None):
        StateMachine.__init__(self, outcomes=['succeeded',
                                              'preempted', 'aborted'])

        self.userdata.text = ''
        self.userdata.win = False

        with self:

            #PREPARE NAO

            StateMachine.add('CLEAN_LISTEN_STATE',
                             StopRecognitionState(),
                             transitions={'succeeded':'START_LISTEN'}
                             )
            StateMachine.add('START_LISTEN',
                             StartRecognitionState(),
                             transitions={'succeeded': 'PREPARE'}
                             )
            StateMachine.add('PREPARE',
                             HomeOn_SM('Sit'),
                             #transitions={'succeeded': 'START_GAME_INTRO'})
                             transitions={'succeeded': 'GAME'})     #DEBUGGING LINE

            '''
            StateMachine.add('SIT_INIT',
                             GoToPostureState('Sit', 0.6),
                             transitions={'succeeded': 'START_GAME_INTRO'})
            '''

            ## INTRODUCTION OF THE GAME

            StateMachine.add('START_GAME_INTRO',
                             ExecuteBehavior(behavior_name='CIR_Presentation'),
                             transitions={'succeeded': 'GAME'},
                             )

            ''''
            StateMachine.add('START_GAME_INTRO',
                             SpeechGesture(text='I am Naomi',behavior_name=
                                           'Presentation-IamNAOMI'),
                             {'succeeded': 'START_GAME_INSTRUCTIONS'})

            speechText = 'What's your name?'
            behaviorName = 'Presentation-WhatsYourName'
            StateMachine.add('START_GAME_YEAR',
                             SpeechGesture(speechText,behaviorName),
                             {'succeeded':'GAME_INTRO_YEAR'})

            speechText = 'How old are you?'
            behaviorName = 'Presentation-WhatsYourName'
            StateMachine.add('START_GAME_NAME',
                             SpeechGesture(speechText, behaviorName),
                             {'succeeded': 'GAME_INTRO_INSTRUCTIONS'})

            instructions = 'I am gonna explain you a game. Think about a any character and ' \
                           'I am going to try to find out who are you thinking about. ' \
                           'I will ask you some questions and you can answer me with ' \
                           'yes or no. '
                           #'yes, no, probably, probably not and i don\'t know. '
            StateMachine.add('GAME_INTRO_INSTRUCTIONS',
                             SpeechGesture(text=instructions, behavior_name='Presentation1-1'),
                             transitions={'succeeded':'GAME_INTRO_START'})

            StateMachine.add('GAME_INTRO_START',
                             SpeechGesture(text='Let\'s start the game', behavior_name='Presentation2-1'),
                             transitions={'succeeded':'GAME'})
            '''

            #GAME LOOP
            StateMachine.add('GAME',
                             AkinatorGame(),
                             transitions={'succeeded': 'WIN', 'aborted': 'LOSE'},
                             remapping={'is_guess': 'win'}
                             )

            #END OF GAME

            StateMachine.add('WIN',
                             ExecuteBehavior(behavior_name='CIR_Winning1'),
                             transitions={'succeeded': 'DISABLE_STIFF'})
            StateMachine.add('LOSE',
                             ExecuteBehavior(behavior_name='CIR_Losing1'),
                             transitions={'succeeded': 'DISABLE_STIFF'})

            StateMachine.add('DISABLE_STIFF',
                             DisableStiffnessState(),
                             transitions={'succeeded': 'STOP_LISTEN'})
            StateMachine.add('STOP_LISTEN',
                             StopRecognitionState(),
                             transitions={'succeeded':'succeeded'}
                             )


if __name__ == "__main__":
    rospy.init_node('start_naoakinator')

    sm = StartNaokinator()

    sm.execute()
