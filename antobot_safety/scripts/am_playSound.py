#!/usr/bin/env python3

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

#   Description:    This code is in charge of playing all audio for safety purposes. At the moment, this is limited to a
#                   single service for playing 2 beeps, but may be extended in the future.
#             	  
#   Contacts:       daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import rospy

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Bool
from am_msgs.srv import safetySound, safetySoundResponse

class playSound:
    def __init__(self):
        # # # Initialises the "playSound" class

        argv = rospy.myargv()
        self.force_stop = False
        self.force_stop_last = False
        self.soundhandle = SoundClient()        # Defines sound client
        self.srvSafetySound = rospy.Service("/antobot_safety/sound", safetySound, self._serviceCallbackSafetySound)  # Defines service callback function
        self.sound1 = "/root/catkin_ws/src/AntoMove/antobot_safety/sounds/mixkit-small-car-horn-717.wav"
        self.word1 = "force stop triggered"
        self.word2 = "hello"
        self.sub_force_stop = rospy.Subscriber("/antobridge/bump_front",Bool,self.force_stop_callback)
        #return

    def _serviceCallbackSafetySound(self, request):
        # # #   A service callback function for /am/safety/sound. At the moment, it will play two beeps from a
        # # #   predefined file in the audio_common/sound_play/sounds folder. This file is also located in antobot_safety/sounds
        # # #   for easier file transfer
        #       Input:    request <int> - can define different integers for different sound requests

        return_msg = safetySoundResponse()
        print("service calback entered!")
        self.play_sound(self.sound1)        # First play is ignored in most situations
        # self.play_sound("mixkit-small-car-horn-717.wav")        # Beep #1
        # self.play_sound("mixkit-small-car-horn-717.wav")        # Beep #2

        return_msg.responseBool = True
        return_msg.responseString = "Success!"

        return return_msg

    def force_stop_callback(self,data):
        #print("sending force stop signal from callback")
        self.force_stop = data.data #True as triggered, false as not triggered
        #print(force_stop)
        #if (force_stop == True):
        #    self.soundhandle.say(self.word1,'voice_kal_diphone',1.0)
        return

    def sound_selection(self):
        #rospy.sleep(1)
        print("into sound_selction")
        print("force_stop=:",self.force_stop)
        #self.soundhandle.say(self.word1,'voice_kal_diphone',1.0)
        if (self.force_stop == True):
        #    self.soundhandle.playWave(self.sound1, 1.0)
            if (self.force_stop != self.force_stop_last):  
                self.soundhandle.playWave(self.sound1,1.0)
                print("say force stop triggered")
        self.force_stop_last = self.force_stop
        print("111")
        #return  
    
    def play_sound(self, sound_in, volume=1.0):
        # # #   Base function for playing a sound via the sound_play package
        #       Input:  sound_in <string> - name of the sound file (.wav or .ogg) to play; must be located in the audio_common/sound_play/sounds folder
        #               volume <float> - value between 0.0 and 1.0 which determines how loud the sound will play 
        print("play_sound entereed!")
        # rospy.sleep(1)
        self.soundhandle.playWave(sound_in, volume)
     

if __name__ == '__main__':
    rospy.init_node('am_playSound', anonymous=True)
    rate = rospy.Rate(1)
    pS = playSound()
   # pS.play_sound("/antobot_safety/sound/mixkit-small-car-horn-717.wav") 
    #pS.sound_selection()
    #rospy.spin()
  
    try:
        while not rospy.is_shutdown():
            # pS.sound_selection()
            rate.sleep()
    except:
        print("Exception occured!")

