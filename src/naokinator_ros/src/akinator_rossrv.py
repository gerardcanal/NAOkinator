#!/usr/bin/env python
import rospy
import sys
from AkinatorHandler import AHandler
from naokinator_ros.srv import akinator_srv, reset_akinator, akinator_srvResponse, reset_akinatorResponse

akinatorHandler = None
name = None
age = 8

def handle_akinator_request(req):
    global akinatorHandler
    if akinatorHandler is None:
        akinatorHandler = AHandler(name, age)

    if (req.question_response == req.init_conversation or req.question_response == ""):
        return akinator_srvResponse(akinatorHandler.getQuestion(), False)
    akinatorHandler.setAnswer(req.question_response)
    Q = akinatorHandler.getQuestion()
    guess = akinatorHandler.guess
    if guess:
        akinatorHandler.close()
        akinatorHandler = None
        return akinator_srvResponse(guess, True)
    return akinator_srvResponse(Q, False)

def handle_reset(req):
    global akinatorHandler, name, age
    if akinatorHandler is not None:
        akinatorHandler.close()
    name = req.name
    age = req.age
    akinatorHandler = AHandler(name, age)
    return reset_akinatorResponse()

def akinator_service():
    global akinatorHandler
    rospy.loginfo("Created akinator handler...")
    akinatorHandler = AHandler(name, age)
    rospy.Service('akinator_srv', akinator_srv, handle_akinator_request)
    rospy.Service('reset_akinator_params', reset_akinator, handle_reset)
    rospy.loginfo('Akinator services are ready!')
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("AkinatorHandler_SRV")
    if len(sys.argv) > 1:
        name = sys.argv[1]
        age = int(sys.argv[2])
    else:
        name = 'default'
        age = 14
    # name = raw_input("Input user name: ")
    # age = raw_input("Input user age: ")
    akinator_service()
