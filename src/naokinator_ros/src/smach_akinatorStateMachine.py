__author__ = 'dani'
#!/usr/bin/env python
import rospy
import sys
from smach import StateMachine, State, SimpleActionState

from naokinator_ros.srv import akinator_srv
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_msgs.msg import WordRecognized

from nao_smach_utils.tts_state import SpeechState
from nao_smach_utils.go_to_posture_state import ExecuteBehavior

class AkinatorGame(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.userdata.is_guess = False
        self.userdata.text = None
        with self:
            # TODO integrate movements, timeout, unsuscribing from /word_recognized and maximum questions to WIN/LOSE
            StateMachine.add('QUESTION',
                             Question(),
                             transitions={'succeeded':'ANSWER'},
                             remapping={'text':'text'}
            )
            StateMachine.add('ANSWER',
                             Answer(),
                             transitions={'succeeded':'succeeded' if self.userdata.is_guess else 'QUESTION'},
                             remapping={'text':'text', 'is_guess_out':'is_guess'}
            )

class Question(smach.State):
    def __init__(self):
        self.naokinatorsrv = rospy.ServiceProxy('/akinator_srv', akinator_srv)

    def execute(self, userdata):
        if (userdata.text is None):
            userdata.text = ''

        userdata.text = self.naokinatorsrv(userdata.text)
        return 'succeeded'

word = None

class Answer(smach.State):
    def __init__(self):
        self.startrec = rospy.ServiceProxy('/start_recognition', Empty)
        self.stoprec = rospy.ServiceProxy('/stop_recognition', Empty)
        self.pub = rospy.Publisher('/speech', String)
        rospy.Subscriber("/word_recognized", WordRecognized, self.callback)

    def callback(data):
        ## FIXME words must be in the robot to be able to recognize something
        yes_words = ['yes', 'si', 'se', 'yea', 'yeah', 'see', 'sea', 'yi', 'yees', 'probably', 'maybe']
        no_words = ['no', 'nu', 'mo','mu','nou', 'now', 'nope']

        maxc = -1
        print "Recognized words:", zip(data.words, data.confidence_values)
        for w, c in zip(data.words, data.confidence_values):
            if c > maxc:
                word = 'yes' if w in yes_words else 'no'
                maxc = c

    def execute(self, userdata):
        global word
        self.startrec()
        self.pub.publish(userdata.text)
        sys.stdout.write(userdata.text+' ')
        while word is None:
            if rospy.is_shutdown():
                sys.exit(-1)
        self.stoprec()
        print word
        word = None