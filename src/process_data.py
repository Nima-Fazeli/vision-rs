#!/usr/bin/env python

import numpy as np
import sys, os
import time
import math


root = os.path.dirname(os.path.abspath(__file__)).split('/mo_jenga')[0]
sys.path.insert(0, root+'/mo_jenga/catkin_ws/src/postporcessing')
from dataManager import DataManagerRefined

sys.path.insert(0, '/home/robot2/anaconda3/envs/python-3.5/lib/python3.5/site-packages')
import cv2
import logging

import matplotlib.pyplot as plt

from Jenga4D.predict_4dpos import Jenga4Dpos



class ImageDataParser():

    def __init__(self):
        self.predictor = Jenga4Dpos()
        self.dm = DataManagerRefined()
        logging.basicConfig(filename=os.path.join(root,'visionDEBUG.log'), level=logging.DEBUG)
        self.counter = None



    def parseFilcodeIndex(self, filecode, index):
        # Get parameters:
        parameters = self.dm.getParamsFileCode(filecode)
        layer = parameters['layer']
        row = parameters['row']

        # Get image
        color_img = self.dm.getColorImage(filecode, index)

        # Call Jenga4D
        bnp, top50_gt_with_scores = self.predictor.predict_4dpos(color_img,'filecode_%d'%filecode, (layer,row)) # This return a np array (4,)

        #bnp = block_pred[np.newaxis, :]
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

        return (bw, bd, phi), top50_gt_with_scores[-10:,:5]



    def plotPred(self, filecode):
        num_indxs = self.dm.getNumberPushes(filecode)

        # Result container:
        predictions = []
        start_t = time.time()
        for indx in range(num_indxs):
            print('______________________________')
            (bw, bd, phi), top10 = self.parseFilcodeIndex(filecode, indx)
            print('%d/%d Completed  ~~~ %s'%(indx+1, num_indxs, timeSince(start_t, float(indx+1)/num_indxs)))
            predictions.append(top10)

        print('plotting')
        predictions = np.array(predictions)
        axis_list = ['y','x','theta']
        axis_index = [1,0,3]
        axis_dim = [10., 10., 180./np.pi]
        for a, ax_name in enumerate(axis_list):
            print('plotting %s'%(a))
            plt.figure(ax_name)
            data_to_plot = predictions[:,:,axis_index[a]]*axis_dim[a]
            for i in range(9):
                plt.plot(data_to_plot[:,i], 'go', label='ranked %d'%i)

            plt.plot(data_to_plot[:,9], 'ro', label='best one')
            mean_v =np.mean(data_to_plot, axis=1)
            std_v = np.std(data_to_plot, axis=1)
            plt.errorbar( np.arange(predictions.shape[0]), mean_v, std_v)
            plt.title('Evolution %s'%ax_name)
            # plt.legend()
            figure_name = 'fc_%d_%s_evolution.png'%(filecode, ax_name)
            path = os.path.join('/home','robot2' ,'mo_jenga','data','vision_plots','best_evolution')
            plt.savefig(os.path.join(path, figure_name))
            print('saved')




    def parseFilecode(self, filecode):
        num_indxs = self.dm.getNumberPushes(filecode)

        # Result container:
        results = {'B_w':[], 'B_d':[], 'phi':[]}

        for indx in range(num_indxs):
            (bw, bd, phi), _ = self.parseFilcodeIndex(filecode, indx)
            results['B_w'].append(bw)
            results['B_d'].append(bd)
            results['phi'].append(phi)

        # Pack the values to the file
        self.dm.fillBlockValues(filecode, results)




    def parseFilecodeAndPlot(self, filecode):
        logging.info('PROCESSING FILECODE -- %d'%filecode)
        num_indxs = self.dm.getNumberPushes(filecode)
        # Get parameters:
        parameters = self.dm.getParamsFileCode(filecode)
        layer = parameters['layer']

        # Result container:
        results = {'B_w':[], 'B_d':[], 'phi':[]}

        # Result container:
        predictions = []
        start_t = time.time()

        for indx in range(num_indxs):
            print('____________________________________________________')
            _, top_10 = self.parseFilcodeIndex(filecode, indx)
            print('top10 shape: ',top_10.shape)
            print(top_10)
            print('%d/%d Completed  ~~~ %s'%(indx+1, num_indxs, timeSince(start_t, float(indx+1)/num_indxs)))
            # Get the mean:
            mean_v = np.mean(top_10, axis=0)
            print('Shape mean_v: ',mean_v.shape)
            # Transform:
            # Assumption: LOWEST LAYER AS Y!!!!
            is_y = layer%2 == 1
            x = mean_v[0]
            y = mean_v[1]
            theta = mean_v[3]
            # Compute bx,bd, phi
            if is_y:
                bw = y/ 100.
                bd = x/ 100.
                phi = theta- np.pi/2
            else:
                bw = x/ 100.
                bd = y/ 100.
                phi = theta #- np.pi/2

            print(bw, bd, phi)
            if indx>0 and abs(bd-results['B_d'][-1])>0.009:
                print('Keeping the same values as the last one')
                results['B_w'].append(results['B_w'][-1])
                results['B_d'].append(results['B_d'][-1])
                results['phi'].append(results['phi'][-1])
            else:
                results['B_w'].append(bw)
                results['B_d'].append(bd)
                results['phi'].append(phi)

            predictions.append(top_10)

        # Pack the values to the file
        self.dm.fillBlockValues(filecode, results)

        predictions = np.array(predictions)
        axis_list = ['y','x','theta']
        axis_index = [1,0,3]
        axis_dim = [10., 10., 180./np.pi]
        for a, ax_name in enumerate(axis_list):
            print('plotting %s'%(a))
            plt.figure(ax_name)
            data_to_plot = predictions[:,:,axis_index[a]]*axis_dim[a]
            for i in range(9):
                plt.plot(data_to_plot[:,i], 'go', label='ranked %d'%i)

            plt.plot(data_to_plot[:,9], 'ro', label='best one')
            mean_v =np.mean(data_to_plot, axis=1)
            std_v = np.std(data_to_plot, axis=1)
            plt.errorbar( np.arange(predictions.shape[0]), mean_v, std_v)
            plt.title('Evolution %s'%ax_name)
            # plt.legend()
            figure_name = 'fc_%d_%s_evolution.png'%(filecode, ax_name)
            path = os.path.join('/home','robot2' ,'mo_jenga','data','vision_plots','best_evolution')
            plt.savefig(os.path.join(path, figure_name))
            print('saved')


    def parseResult(self, result):
        assert(result in ['0','1','2','3'])
        filecode_names = self.dm.getFileCodeNames_Result(result, return_df=False)
        num_filecodes = len(filecode_names)
        time_s = time.time()
        if self.counter is None:
            self.counter = 0
            self.total_num_files = num_filecodes
            self.time_start = time_s
        for i, fc in enumerate(filecode_names):
            if fc<484:
                print('\n==========================================\n')
                print('PROCESSING FILECODE -- %d  -- (%d/%d)--- %s\n'%(fc, self.counter, self.total_num_files, timeSince(self.time_start, float(self.counter+1)/self.total_num_files)))
                print('Result %s -- Done %d/%d'%(result, i, num_filecodes))
                self.parseFilecodeAndPlot(fc)
                self.counter += 1


    def parseAll(self):
        self.time_start=time.time()
        self.counter = 0
        self.total_num_files = 0
        results = ['0','1','2','3']
        for r in results:
            self.total_num_files += len(self.dm.getFileCodeNames_Result(r, return_df=False))

        for result in results:
            self.parseResult(result)



def asMinutes(s):
    m = math.floor(s / 60)
    s -= m * 60
    return '%dm %ds' % (m, s)


def timeSince(since, percent):
    now = time.time()
    s = now - since
    es = s / percent
    rs = es - s
    return '%s (- %s)' % (asMinutes(s), asMinutes(rs))


## TEST CODE::

if __name__ == '__main__':
    filecodes = [2]

    idp = ImageDataParser()
    # for fc in filecodes:
    #     #idp.plotPred(fc)
    #     idp.parseFilecodeAndPlot(fc)
    # idp.parseAll()
    idp.parseResult('0')
