#!/usr/bin/env python

import numpy as np
import sys, os
import cv2


root = os.path.dirname(os.path.abspath(__file__)).split('/mo_jenga')[0]
sys.path.insert(0, root+'/mo_jenga/catkin_ws/src/postporcessing')
form dataManager import DataManagerRefined

sys.path.insert(0, '/home/robot2/anaconda3/envs/python-3.5/lib/python3.5/site-packages')
from Jenga4D.predict_4dpos import Jenga4Dpos



class ImageDataParser():

    def __inti__(self):
        self.predictor = Jenga4Dpos()
        self.dm = DataManagerRefined()


    def parseFilcodeIndex(self, filecode, index):
        # Get image
        color_img = self.dm.getColorImage(filecode, index)

        # call Jenga4D

        return bx, bd, phi


    def parseFilecode(self, filecode):
        num_indxs = self.dm.getNumberPushes(filecode)

        # Result container:
        results = {'B_x':[], 'B_d':[], 'phi':[]}

        for indx in range(num_indxs):
            bx, bd, phi = self.parseFilcodeIndex(filecode, indx)







