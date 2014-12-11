#!/usr/bin/env python
import rospy
import sys
from naokinator_ros.srv import akinator_srv
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_msgs.msg import WordRecognized

## FIXME words must be in the robot to be able to recognize something

word = None
yes_words = ['yes', 'si', 'se', 'yea', 'yeah', 'see', 'sea', 'yi', 'yees', 'probably', 'maybe']
no_words = ['no', 'nu', 'mo','mu','nou', 'now', 'nope']

def callback(data):
    global word
    maxc = -1
    print "Recognized words:", zip(data.words, data.confidence_values)
    for w, c in zip(data.words, data.confidence_values):
        if c > maxc:
            word = 'yes' if w in yes_words else 'no'
            maxc = c

if __name__ == "__main__":
    rospy.init_node('naoakinator_test')

    naokinatorsrv = rospy.ServiceProxy('/akinator_srv', akinator_srv)
    startrec = rospy.ServiceProxy('/start_recognition', Empty)
    stoprec = rospy.ServiceProxy('/stop_recognition', Empty)
    pub = rospy.Publisher('/speech', String)

    rospy.Subscriber("/word_recognized", WordRecognized, callback)

    resp = naokinatorsrv('')
    while not resp.is_guess:
        startrec()
        pub.publish(resp.question_guess_response)
        sys.stdout.write(resp.question_guess_response+' ')
        while word is None:
            if rospy.is_shutdown():
                sys.exit(-1)
        stoprec()
        print word
        resp = naokinatorsrv(word)
        word = None
    pub.publish('I am done! You were thinking about '+resp.question_guess_response)
    print 'I am done! You were thinking about '+resp.question_guess_response
