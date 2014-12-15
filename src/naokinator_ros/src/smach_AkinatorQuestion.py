__author__ = 'dani'

import rospy
from smach import StateMachine, State
from smach_ros import ServiceState

from naokinator_ros.srv import AkinatorQA
from nao_smach_utils.execute_speechgesture_state import SpeechGesture
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehaviorFromPoolSM


class AkinatorServiceState(ServiceState):
    def __init__(self):
        def resp_cb(ud, response):
            ud.resp_text = response.question_guess_response
            if response.is_guess:
                return 'game_finished'
            return 'continue_game'
        ServiceState.__init__(self, '/akinator_srv', AkinatorQA, output_keys=['resp_text'], request_slots=['question_response'], outcomes=['game_finished', 'continue_game'], response_cb=resp_cb)


class AkinatorRequestQuestion(StateMachine):

    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['user_answer'], output_keys=['text'])
        with self:
            # DONE integrate movements, speech and retrieval from akinator service
            StateMachine.add('QUESTION',
                             AkinatorServiceState(),
                             transitions={'game_finished': 'aborted', 'continue_game': 'QUESTION_GESTURE'},
                             remapping={'resp_text': 'text', 'question_response': 'user_answer'}
                             )


            StateMachine.add('QUESTION_GESTURE',
                             SpeechGesture(behavior_pool=['CIR_Asking1','CIR_Asking2','CIR_Asking3','CIR_Asking4','CIR_Asking5','CIR_Asking6']),
                             transitions={'succeeded': 'succeeded'})
