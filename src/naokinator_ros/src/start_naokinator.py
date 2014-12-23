#!/usr/bin/env python
import rospy

from smach import StateMachine, CBState, Concurrence
from smach_ros import ServiceState, IntrospectionServer

from naokinator_ros.srv import ResetAkinator, ResetAkinatorRequest
from smach_AkinatorAnswer import GetUserAnswer, yes_words, no_words, maybe_words, probably_not_words, dunno_words
from smach_akinatorStateMachine import AkinatorGame
from nao_smach_utils.execute_speechgesture_state import SpeechGesture
from concurrent_userspeech_jointtrajectory import SpeechRecognitionAndGesture
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehavior
from nao_smach_utils.home_onoff import HomeOn_SM
from nao_smach_utils.stiffness_states import DisableStiffnessState
from nao_smach_utils.speech_recognition_states import SetSpeechVocabularyState, StopRecognitionState
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehaviorFromPoolSM


class StartNaokinator(StateMachine):

    def __init__(self, driver=None):
        StateMachine.__init__(self, outcomes=['succeeded',
                                              'preempted', 'aborted'])

        self.userdata.text = ''

        with self:

            StateMachine.add('PREPARE',
                             HomeOn_SM('Sit'),
                             transitions={'succeeded': 'INIT'})
                             #transitions={'succeeded': 'GAME'}) #DEBUGGING LINE
            #PREPARE NAO
            setup = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
            with setup:
                StateMachine.add('CLEAN_LISTEN_STATE',
                                 StopRecognitionState(),
                                 transitions={'succeeded': 'SET_VOCABULARY'}
                                 )
                StateMachine.add('SET_VOCABULARY',
                                 SetSpeechVocabularyState(no_words + yes_words + probably_not_words + dunno_words + maybe_words),
                                 transitions={'succeeded': 'RESET_AKINATOR'}
                                 )

                #RESET AKINATOR WEB
                StateMachine.add('RESET_AKINATOR',
                                 ServiceState('/reset_akinator_params',
                                              ResetAkinator,
                                              request=ResetAkinatorRequest('Name', 22)),
                                 transitions={'succeeded': 'succeeded'}
                                 )
            cc = Concurrence(outcomes=['succeeded', 'aborted', 'preempted'], default_outcome='aborted',
                             outcome_map={'succeeded': {'START_GAME_INTRO': 'succeeded', 'SETUP': 'succeeded'}})
            with cc:
                # INTRODUCTION OF THE GAME
                Concurrence.add('START_GAME_INTRO',
                                ExecuteBehaviorFromPoolSM(behavior_pool=['CIR_Presentation1', 'CIR_Presentation2']))
                # NAO SETUP
                Concurrence.add('SETUP', setup)

            StateMachine.add('INIT', cc,
                             transitions={'succeeded': 'GAME', 'aborted': 'LOSE'})

            #GAME LOOP
            StateMachine.add('GAME',
                             AkinatorGame(),
                             transitions={'succeeded': 'CHAR_QUESTION_MAKE', 'aborted': 'LOSE'}
                             )

            #END OF GAME
            StateMachine.add('CHAR_QUESTION_MAKE',
                             CBState(self.compoundQuestion,
                                     input_keys=['text'],
                                     output_keys=['text'],
                                     outcomes=['succeeded', 'aborted']),
                             transitions={'succeeded': 'CHAR_CORRECT'}
                             )

            StateMachine.add('CHAR_CORRECT',
                             SpeechGesture(behavior_pool=['CIR_Asking1', 'CIR_Asking2', 'CIR_Asking3', 'CIR_Asking4', 'CIR_Asking5', 'CIR_Asking6']),
                             transitions={'succeeded': 'FINAL_ANSWER'}
                             )
            StateMachine.add('FINAL_ANSWER',
                             SpeechRecognitionAndGesture(),
                             transitions={'succeeded': 'CHECK_WIN_LOSE', 'aborted': 'LOSE'}
                             )

            def check_w_l(userdata):
                if (userdata.text == 'yes'):
                    return 'win'
                return 'lose'
            StateMachine.add('CHECK_WIN_LOSE',
                             CBState(check_w_l, input_keys=['text'], outcomes=['win', 'lose']),
                             transitions={'win': 'WIN', 'lose': 'LOSE'})

            StateMachine.add('WIN',
                             ExecuteBehaviorFromPoolSM(behavior_pool=['CIR_Winning1', 'CIR_Winning2', 'CIR_Winning3', 'CIR_Winning4', 'CIR_Winning5']),
                             transitions={'succeeded': 'DISABLE_STIFF'})
            StateMachine.add('LOSE',
                             #ExecuteBehavior(behavior_name='CIR_Losing1'),
                             ExecuteBehaviorFromPoolSM(behavior_pool=['CIR_Loosing1', 'CIR_Loosing2', 'CIR_Loosing3', 'CIR_Loosing4']),
                             transitions={'succeeded': 'DISABLE_STIFF'})

            StateMachine.add('DISABLE_STIFF',
                             DisableStiffnessState(),
                             transitions={'succeeded': 'succeeded'})

    def compoundQuestion(self, ud):
        rospy.logwarn(ud.text)
        ud.text = 'Is your character %s?' % ud.text
        return 'succeeded'


if __name__ == "__main__":
    rospy.init_node('start_naoakinator')

    sm = StartNaokinator()

    sis = IntrospectionServer("naokinator_introspection_srv", sm, '/NAOKINATOR_ROOT')
    sis.start()

    sm.execute()
    rospy.spin()
    sis.stop()
