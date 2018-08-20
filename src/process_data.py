#!/usr/bin/env python

import numpy as np
import sys, os
import cv2
import logging


root = os.path.dirname(os.path.abspath(__file__)).split('/mo_jenga')[0]
sys.path.insert(0, root+'/mo_jenga/catkin_ws/src/postporcessing')
from dataManager import DataManagerRefined

sys.path.insert(0, '/home/robot2/anaconda3/envs/python-3.5/lib/python3.5/site-packages')
from Jenga4D.predict_4dpos import Jenga4Dpos



class ImageDataParser():

    def __inti__(self):
        self.predictor = Jenga4Dpos()
        self.dm = DataManagerRefined()
        logging.basicConfig(filename=os.path.join(root,'visionDEBUG.log'), level=logging.DEBUG)


    def parseFilcodeIndex(self, filecode, index):
        # Get parameters:
        parameters = self.dm.getParamsFileCode(filecode)
        layer = parameters['layer']
        row = parameters['row']

        # Get image
        color_img = self.dm.getColorImage(filecode, index)

        # Call Jenga4D
        bnp = self.predictor(color_img,'filecode_%d'%self.filecode, (layer,row)) # This return a np array (4,)

        # Process Jenga4D output
        blocks_pose_list = []
        zaxis = (0, 0, 1)
        theta = bnp[3] + np.pi / 2.
        # the following 3 lines are for getting the orientation as jenga_tf
        epsilon = np.pi / 8
        if np.pi - epsilon < theta < np.pi + epsilon:
            theta -= np.pi
        x = bnp[0] / 100.
        y = bnp[1] / 100.
        z = bnp[2] / 100.


        # Write the code in a debug file in case it has failed
        # logging.debug('FC %d | Indx  %d||- Block (%d,%d) NOT found'%(filecode, index, layer, row))

        # Assumption: LOWEST LAYER AS Y!!!!
        is_y = layer%2 == 1

        # Compute bx,bd, phi
        if is_y:
            bw = y
            bd = x
            phi = theta
        else:
            bw = x
            bd = y
            phi = theta - np.pi/2

        return (bw, bd, phi)


    def parseFilecode(self, filecode):
        num_indxs = self.dm.getNumberPushes(filecode)

        # Result container:
        results = {'B_w':[], 'B_d':[], 'phi':[]}

        for indx in range(num_indxs):
            bw, bd, phi = self.parseFilcodeIndex(filecode, indx)
            results['B_w'] = bw
            results['B_d'] = bd
            results['phi'] = phi

        # Pack the values to the file
        self.dm.fillBlockValues(filecode, results)


    def parseResult(self, result):
        assert(result in ['0','1','2','3'])
        filecode_names = self.getFileCodeNames_Result(result, return_df=False)
        for fc in filecode_names:
            self.parseFilecode(fc)


    def parseAll(self):
        results = ['0','1','2','3']
        for result in results:
            self.parseResult(result)






## TEST CODE::

if __name__ == '__main__':
    filecodes = [0]

    idp = ImageDataParser()
    for fc in filecodes:
        idp.parseFilecode(fc)




