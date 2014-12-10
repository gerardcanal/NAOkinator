#!/usr/bin/env python
import rospy

from smach import StateMachine

from nao_smach_utils.execute_speechgesture_state import SpeechGesture
from nao_smach_utils.tts_state import SpeechState
from nao_smach_utils.go_to_posture_state import GoToPostureState

class StartNaokinator(StateMachine):

    def __init__(self, driver=None):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])

        self.userdata.win = False

        with self:
            StateMachine.add('SIT_INIT',
                             GoToPostureState('Sit', 0.6),
                             transitions={'succeeded': 'START_GAME_INTRO'})

            ## INTRODUCTION OF THE GAME
            StateMachine.add('START_GAME_INTRO',
                             SpeechGesture(text='I am Naomi', behavior_name='Presentation-IamNAOMI'),
                             transitions={'succeeded':'START_GAME_INSTRUCTIONS'})
            '''
            StateMachine.add('START_GAME_YEAR',
                             SpeechGesture(text='What's your name?', behavior_name='Presentation-WhatsYourName'),
                             transitions={'succeeded':'GAME_INTRO_YEAR'})

            StateMachine.add('START_GAME_NAME',
                             SpeechGesture(text='How old are you?', behavior_name='Presentation-WhatsYourName'),
                             transitions={'succeeded':'GAME_INTRO_INSTRUCTIONS'})
            '''

            instructions = 'I am gonna explain you a game. Think about a any character and ' \
                           'I am going to try to find out who are you thinking about. ' \
                           'I will ask you some questions and you can answer me with ' \
                           'yes or no. '
                           #'yes, no, probably, probably not and i don’t know. '
            StateMachine.add('GAME_INTRO_INSTRUCTIONS',
                             SpeechGesture(text=instructions, behavior_name='Presentation1-1'),
                             transitions={'succeeded':'GAME_INTRO_START'})

            StateMachine.add('GAME_INTRO_START',
                             SpeechGesture(text='Let\'s start the game', behavior_name='Presentation2-1'),
                             transitions={'succeeded':'GAME'})

            #GAME LOOP
            StateMachine.add('GAME',
                             SpeechGesture(text='Let\'s start the game', behavior_name='Presentation2-1'),
                             transitions={'succeeded':'WIN' if self.userdata.win else 'LOSE'},
                             remapping={'is_guess':'win'}
            )

            #END OF GAME

            StateMachine.add('WIN',
                             SpeechGesture(text='Yeah! I knew it!', behavior_name='Win1'),
                             transitions={'succeeded':'suceeded'})
            StateMachine.add('GAME_INTRO_START',
                             SpeechGesture(text='Oh, i missed it completely.', behavior_name='Lose1'),
                             transitions={'succeeded':'suceeded'})


if __name__ == "__main__":
    rospy.init_node('start_naoakinator')

	sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    smach.StateMachine.add('Start_NAO_position',
                            GoToPostureState(),
                            transitions={'succeeded': 'succeeded','aborted':'Start_NAO_position'})