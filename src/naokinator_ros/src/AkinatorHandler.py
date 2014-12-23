__author__ = 'dani'

from selenium import webdriver
from selenium.common.exceptions import NoSuchElementException


class AHandler():
    def __init__(self, name, age, driver=None):
        if driver is None:
            driver = webdriver.Firefox()
        self.name = name
        self.age = age
        self.driver = driver
        self.nQuestion = 0
        self.questions = []
        self.answers = []
        self.guess = None
        self.GetPage()

    def GetPage(self):
        self.driver.get("http://en.akinator.com/personnages")
        txtage = self.driver.find_element_by_xpath("//input[@id='elokence_sitebundle_identification_age']")
        txtage.send_keys(str(self.age))
        txtage.submit()
        try:
            self.getQuestion()
        except Exception:
            self.GetPage()

    def getQuestion(self):

        Q = ''
        try:
            self.nQuestion = int(self.driver.find_element_by_xpath("//span[@id='nqp']").text)
            Q = self.driver.find_element_by_xpath("//div[@id='bulle-inner']").text

            if(len(self.questions) >= self.nQuestion):
                self.questions[self.nQuestion-1] = Q
            else:
                self.questions.insert(self.nQuestion-1, Q)
        except NoSuchElementException:
            self.guess = self.driver.find_element_by_xpath("//h2[@id='perso']").text

        return Q

    def setAnswer(self, a):
        if a == "yes" or a == "yep":
            r = 1
        elif a == "no" or a == "nope":
            r = 2
        elif a == "dunno" or a == "i don't know" or a == "don't know":
            r = 3
        elif a == "maybe":
            r = 4
        elif a == "probably not":
            r = 5
        else:
            return
        rospy.loginfo(a)
        self.driver.find_element_by_xpath("//a[@id='reponse" + str(r) + "']").click()

    def close(self):
        self.driver.close()

if __name__ == "__main__":

    akinator = AHandler('Dani', 27, webdriver.Firefox())

    Q = akinator.getQuestion()
    import random
    import rospy
    while not akinator.guess:
        a = "yes" if random.random() > 0.5 else "no"
        akinator.setAnswer(a)
        rospy.sleep(2)
        Q = akinator.getQuestion()

    print akinator.guess

    akinator.close()
