#!/usr/bin/env python
import rospy
import sys
from AkinatorHandler import AHandler
from naokinator_ros.srv import AkinatorQA, ResetAkinator, AkinatorQAResponse, ResetAkinatorResponse

akinatorHandler = None
name = None
age = 8


def handle_akinator_request(req):
    global akinatorHandler
    if akinatorHandler is None:
        akinatorHandler = AHandler(name, age)

    if (req.question_response == req.init_conversation or req.question_response == ""):
        return AkinatorQAResponse(akinatorHandler.getQuestion(), False)
    akinatorHandler.setAnswer(req.question_response)
    Q = akinatorHandler.getQuestion()
    guess = akinatorHandler.guess
    rospy.loginfo('Akinator service response is :"' + (guess + '". IS' if guess else Q + '". IS NOT') + ' a guess.')
    if guess:
        akinatorHandler.close()
        akinatorHandler = None
        return AkinatorQAResponse(guess, True)
    return AkinatorQAResponse(Q, False)


def handle_reset(req):
    global akinatorHandler, name, age
    if akinatorHandler is not None:
        akinatorHandler.close()
    name = req.name
    age = req.age
    akinatorHandler = AHandler(name, age)
    return ResetAkinatorResponse()


def akinator_service():
    global akinatorHandler
    rospy.loginfo("Created akinator handler...")
    akinatorHandler = AHandler(name, age)
    rospy.Service('akinator_srv', AkinatorQA, handle_akinator_request)
    rospy.Service('reset_akinator_params', ResetAkinator, handle_reset)
    rospy.loginfo('Akinator services are ready!')
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("AkinatorHandler_SRV")
    if len(sys.argv) > 1:
        name = sys.argv[1]
        age = int(sys.argv[2])
    else:
        name = 'default'
        age = 22
    # name = raw_input("Input user name: ")
    # age = raw_input("Input user age: ")
    akinator_service()
