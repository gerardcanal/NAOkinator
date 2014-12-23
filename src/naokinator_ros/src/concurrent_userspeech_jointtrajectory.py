
from smach import StateMachine, Concurrence

from smach_AkinatorAnswer import GetUserAnswer
from nao_smach_utils.joint_trajectory_state import JointAngleState
from nao_smach_utils.random_selection_state import RandomSelectionFromPoolState
from nao_smach_utils.timeout_state import TimeOutState


class SpeechRecognitionAndGesture(Concurrence):

    _movementList = [
                     [-0.2, -0.2, 0.7, -0.7],
                     [-0.2, -0.2, 0.78, -0.7],
                     [-0.2, -0.2, 0.87, -0.7],
                     [-0.2, -0.2, 0.7, -0.78],
                     [-0.2, -0.2, 0.78, -0.78],
                     [-0.2, -0.2, 0.87, -0.78],
                     [-0.2, -0.2, 0.7, -0.87],
                     [-0.2, -0.2, 0.78, -0.87],
                     [-0.2, -0.2, 0.87, -0.87],
                     [-0.2, 0, 0.7, -0.7],
                     [-0.2, 0, 0.78, -0.7],
                     [-0.2, 0, 0.87, -0.7],
                     [-0.2, 0, 0.7, -0.78],
                     [-0.2, 0, 0.78, -0.78],
                     [-0.2, 0, 0.87, -0.78],
                     [-0.2, 0, 0.7, -0.87],
                     [-0.2, 0, 0.78, -0.87],
                     [-0.2, 0, 0.87, -0.87],
                     [-0.2, 0.2, 0.7, -0.7],
                     [-0.2, 0.2, 0.78, -0.7],
                     [-0.2, 0.2, 0.87, -0.7],
                     [-0.2, 0.2, 0.7, -0.78],
                     [-0.2, 0.2, 0.78, -0.78],
                     [-0.2, 0.2, 0.87, -0.78],
                     [-0.2, 0.2, 0.7, -0.87],
                     [-0.2, 0.2, 0.78, -0.87],
                     [-0.2, 0.2, 0.87, -0.87],
                     [0, -0.2, 0.7, -0.7],
                     [0, -0.2, 0.78, -0.7],
                     [0, -0.2, 0.87, -0.7],
                     [0, -0.2, 0.7, -0.78],
                     [0, -0.2, 0.78, -0.78],
                     [0, -0.2, 0.87, -0.78],
                     [0, -0.2, 0.7, -0.87],
                     [0, -0.2, 0.78, -0.87],
                     [0, -0.2, 0.87, -0.87],
                     [0, 0, 0.7, -0.7],
                     [0, 0, 0.78, -0.7],
                     [0, 0, 0.87, -0.7],
                     [0, 0, 0.7, -0.78],
                     [0, 0, 0.78, -0.78],
                     [0, 0, 0.87, -0.78],
                     [0, 0, 0.7, -0.87],
                     [0, 0, 0.78, -0.87],
                     [0, 0, 0.87, -0.87],
                     [0, 0.2, 0.7, -0.7],
                     [0, 0.2, 0.78, -0.7],
                     [0, 0.2, 0.87, -0.7],
                     [0, 0.2, 0.7, -0.78],
                     [0, 0.2, 0.78, -0.78],
                     [0, 0.2, 0.87, -0.78],
                     [0, 0.2, 0.7, -0.87],
                     [0, 0.2, 0.78, -0.87],
                     [0, 0.2, 0.87, -0.87],
                     [0.2, -0.2, 0.7, -0.7],
                     [0.2, -0.2, 0.78, -0.7],
                     [0.2, -0.2, 0.87, -0.7],
                     [0.2, -0.2, 0.7, -0.78],
                     [0.2, -0.2, 0.78, -0.78],
                     [0.2, -0.2, 0.87, -0.78],
                     [0.2, -0.2, 0.7, -0.87],
                     [0.2, -0.2, 0.78, -0.87],
                     [0.2, -0.2, 0.87, -0.87],
                     [0.2, 0, 0.7, -0.7],
                     [0.2, 0, 0.78, -0.7],
                     [0.2, 0, 0.87, -0.7],
                     [0.2, 0, 0.7, -0.78],
                     [0.2, 0, 0.78, -0.78],
                     [0.2, 0, 0.87, -0.78],
                     [0.2, 0, 0.7, -0.87],
                     [0.2, 0, 0.78, -0.87],
                     [0.2, 0, 0.87, -0.87],
                     [0.2, 0.2, 0.7, -0.7],
                     [0.2, 0.2, 0.78, -0.7],
                     [0.2, 0.2, 0.87, -0.7],
                     [0.2, 0.2, 0.7, -0.78],
                     [0.2, 0.2, 0.78, -0.78],
                     [0.2, 0.2, 0.87, -0.78],
                     [0.2, 0.2, 0.7, -0.87],
                     [0.2, 0.2, 0.78, -0.87],
                     [0.2, 0.2, 0.87, -0.87]]

    _delayList = [0.2, 0.8, 0.9, 1, 1.2, 1.5]

    def getfinish_Cb(self, outcome_map):
        if outcome_map['GET_USER_ANSWER'] == 'succeeded' or outcome_map['GET_USER_ANSWER'] == 'aborted':
            return True
        return False

    def outcome_Cb(self, outcome_map):
        if outcome_map['GET_USER_ANSWER'] == 'succeeded':
            return 'succeeded'
        return 'aborted'

    def __init__(self):
        Concurrence.__init__(self, default_outcome='aborted',
                             input_keys=['text'],
                             output_keys=['text'],
                             outcomes=['succeeded', 'preempted', 'aborted'],
                             child_termination_cb=self.getfinish_Cb,
                             outcome_cb=self.outcome_Cb)

        jointLoop = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with jointLoop:
            StateMachine.add('NEXT_MOVE',
                             RandomSelectionFromPoolState(self._movementList),
                             remapping={'selected_item': 'joint_angles'},
                             transitions={'succeeded': 'MOVEMENT'}
                             )
            StateMachine.add('MOVEMENT',
                             JointAngleState(['HeadPitch', 'HeadYaw', 'RElbowYaw', 'LElbowYaw']),
                             transitions={'succeeded': 'NEXT_DELAY'}
                             )
            StateMachine.add('NEXT_DELAY',
                             RandomSelectionFromPoolState(self._delayList),
                             remapping={'selected_item': 'timeout'},
                             transitions={'succeeded': 'DELAY'}
                             )
            StateMachine.add('DELAY',
                             TimeOutState(),
                             transitions={'succeeded': 'NEXT_MOVE'}
                             )

        with self:
            Concurrence.add('MOVING',
                            jointLoop)

            Concurrence.add('GET_USER_ANSWER',
                            GetUserAnswer(),
                            remapping={'text': 'text'})
