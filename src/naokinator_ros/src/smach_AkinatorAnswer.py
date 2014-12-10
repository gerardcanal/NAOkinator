__author__ = 'dani'

import rospy
from smach import StateMachine

from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_msgs.msg import WordRecognized
from nao_smach_utils.execute_speechgesture_state import SpeechGesture

class AkinatorAnswer(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        with self:
            # TODO integrate movements, speech and retrieval from akinator service
            StateMachine.add('Answer',
                             Answer(),
                             transitions={'succeeded':'THINKING', 'aborted':'aborted'},
                             remapping={'text':'text'}
            )

            StateMachine.add('THINKING',
                             SpeechGesture(text='Let me see...', behavior_name='Thinking1'),
                             transitions={'succeeded':'succeeded'})


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
        self.pub.unregister()