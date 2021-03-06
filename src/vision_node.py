#!/usr/bin/env python
"""
For Launch:
# <node name="vision_node" pkg="vision_rs" type="vision_node.py" />

catkin package:
catkin_create_pkg vision_rs std_msgs std_srvs rospy roscpp

then run catkin_make
"""

import sys, time, os
import matplotlib.pyplot as plt
from PIL import Image as imm

sys.path.insert(0, '/home/robot2/anaconda3/envs/python-3.5/lib/python3.5/site-packages')
root =  os.path.dirname(os.path.abspath(__file__)).split('/mo_jenga')[0]
from Jenga4D.predict_4dpos import Jenga4Dpos

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'Jenga4D'))

# numpy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib, rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage, Image
import std_srvs.srv
from geometry_msgs.msg import TransformStamped, Pose
from vision_rs.msg import BlocksPose
from vision_rs.srv import BlockPoseService


root =  os.path.dirname(os.path.abspath(__file__)).split('/mo_jenga')[0]



class Vision:
    def __init__(self):
        # Initialize the ros node
        rospy.init_node("vision_node_server")
        rospy.loginfo("[VISION] - Loading model ...")
        # Initialize the vision class Jenga4D
        self.predictor = Jenga4Dpos()
        rospy.loginfo("[VISION] - Finished loading ...")
        # vision service
        self.visSrv = rospy.Service('vision_rs/blocks_poses', BlockPoseService, self.handle_vision_service)

        # Memory for storing images
        self.filecode = -1  # Last filecode recieved
        self.image_count = 0

        self.predictions = []

    def restart_pred(self):
        self.predictions = []

    def plot_pred(self):
        if len(self.predictions) != 0:
            print('plotting')
            self.predictions = np.array(self.predictions)
            axis_list = ['y','x','theta']
            axis_index = [1,0,3]
            axis_dim = [10., 10., 180./np.pi]
            for a, ax_name in enumerate(axis_list):
                print('plotting %s'%(a))
                plt.figure(ax_name)
                data_to_plot = self.predictions[:,:,axis_index[a]]*axis_dim[a]
                for i in range(9):
                    plt.plot(data_to_plot[:,i], 'go', label='ranked %d'%i)

                plt.plot(data_to_plot[:,9], 'ro', label='best one')
                mean_v =np.mean(data_to_plot, axis=1)
                std_v = np.std(data_to_plot, axis=1)
                plt.errorbar( np.arange(self.predictions.shape[0]), mean_v, std_v)
                plt.title('Evolution %s'%ax_name)
                # plt.legend()
                figure_name = 'fc_%d_%s_evolution.png'%(self.filecode, ax_name)
                path = os.path.join('/home','robot2' ,'mo_jenga','data','vision_plots','best_evolution')
                plt.savefig(os.path.join(path, figure_name))
                print('saved')
        else:
            pass


    def pack_blocks(self, blocks_list):
        # Initialize
        bp = BlocksPose()
        bp.header.stamp = rospy.Time.now()
        bp.header.frame_id = '/jenga_tower'
        poses = []
        for i, block in enumerate(blocks_list):
            # TODO: fill the data with the one form blocks_list
            # Pack the values
            pose = Pose()
            pose.position.x = block['x']
            pose.position.y = block['y']
            pose.position.z = block['z']
            pose.orientation.x = block['qx']
            pose.orientation.y = block['qy']
            pose.orientation.z = block['qz']
            pose.orientation.w = block['qw']
            poses.append(pose)

        bp.blocks = poses
        return bp

    def get_image(self, color=True):
        # get image from camera and convert to cv format
        if color:
            camera_message = '/camera/color/image_raw/compressed'
            while True:
                try:
                    ros_data = rospy.wait_for_message(camera_message, CompressedImage)
                    # print('message received')
                    rospy.loginfo('message received')
                    break
                except:
                    print('waiting for message')

            np_arr = np.fromstring(ros_data.data, np.uint8)
            # print(np_arr.shape)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        else:
            camera_message = '/camera/aligned_depth_to_color/image_raw' #'/camera/depth/image_rect_raw/compressed'

            #~ camera_message = '/camera/aligned_depth_to_color/image_raw/compressed' #'/camera/depth/image_rect_raw/compressed'
            # print('going for depth')
            while True:
                try:
                    ros_data = rospy.wait_for_message(camera_message, Image)
                    #~ ros_data = rospy.wait_for_message(camera_message, CompressedImage)
                    # print('message received')
                    rospy.loginfo('message received')
                    break
                except:
                    print('waiting for depth message')
            #~ mg = self.compress_to_cv2(ros_data, '16UC1')
            #~ return mg
            if True:
                h = ros_data.height
                w = ros_data.width
                step = ros_data.step
                # print('size (h,w,s): (%d, %d, %d)' % (h,w, step))
                encoding = ros_data.encoding
                # print('encoding: %s'%encoding)

                np_arr = ros_data.data#np.fromstring(ros_data.data, np.uint8)
                arrr = np.full((h,w), 0.0)
                top_crop = 80
                for i in range(top_crop,h):
                    for j in range(120,w):
                        val = (np_arr[i*w*2+j*2]+256.0*np_arr[i*w*2+j*2+1])#/(2**8-1)
                        if val > 2**8*5:
                            val = 2**8*5
                            #~ val = 0
                        arrr[i,j] = val
                        #~ arrr[i,j] = (np_arr[i*w*2+j*2]+256.0*np_arr[i*w*2+j*2+1])#/(2**8-1)
                        #~ arrr[i,j] = (np_arr[i*w*2+j]+256*np_arr[i*w*2+j])#/(2**8-1)
                #~ pdb.set_trace()
                # print('max: %.4f'%arrr.max())
                # print('max: %.4f'%np.amax(arrr))

                cv2.normalize(arrr,arrr, 0, 1, cv2.NORM_MINMAX)

                arrr_norm = 255*arrr #*(2**8-1)/arrr.max()#(2**16-1.0)

                #print(np_arr.shape)
                # print(arrr.shape)

                image_np = arrr_norm#.astype(np.int8)#cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                # print(image_np)

            else:
                dtype = np.dtype(np.int16)
                dtype = dtype.newbyteorder('>' if ros_data.is_bigendian else '<')
                shape = (ros_data.height, ros_data.width, 1)
                data = np.fromstring(ros_data.data, dtype = dtype).reshape(shape)
                cv2.normalize(data, data, 0, 1, cv2.NORM_MINMAX)
                data.strides = (ros_data.step, dtype.itemsize*1, dtype.itemsize)
                image_np = data[...,0]*255


        return image_np


    def get_dimg_raw(self):
        camera_message = '/camera/aligned_depth_to_color/image_raw'
        while True:
            try:
                ros_data = rospy.wait_for_message(camera_message, Image)
                print('message received')
                rospy.loginfo('message received')
                break
            except:
                print('waiting for message')
            #~ mg = self.compress_to_cv2(ros_data, '16UC1')
            #~ return mg
        h = ros_data.height
        w = ros_data.width
        step = ros_data.step
        # print('size (h,w,s): (%d, %d, %d)' % (h,w, step))
        encoding = ros_data.encoding
        # print('encoding: %s'%encoding)

        np_arr = ros_data.data#np.fromstring(ros_data.data, np.uint8)

        araw = np.empty((h,w), dtype=np.uint16)
        for i in range(h):
            for j in range(w):
                val = (np_arr[i*w*2+j*2]+256*np_arr[i*w*2+j*2+1])#/(2**8-1)
                araw[i,j] = val
        return araw

    def compress_to_cv2(self, cmprs_img_msg, desired_encoding = "passthrough"):
        str_msg = cmprs_img_msg.data
        buf = np.ndarray(shape=(1, len(str_msg)), dtype=np.uint8, buffer=cmprs_img_msg.data)
        im = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)
        if desired_encoding == "passthrough":
            return im
        try:
            res = cv2.cvtColor(im, "bgr8", desired_encoding)
        except RuntimeError as e:
            raise CvBridgeError(e)

        return res


    def save_image(self,img, depth=False):
        # print('Saving image')
        cd = {True:'depth',False:'color'}
        c = cd[depth]
        filename = '%s_fc_%d_%d.png'%(c, self.filecode, self.image_count)
        path = os.path.join(root,'mo_jenga', 'data', 'imgs', c, 'filecode_%d'%self.filecode)
        # print('path creating')
        # print(path)
        if not os.path.exists(path):
            print ('Creating directory: %s'%path)
            os.makedirs(path)
            print('path created')
        file_path = os.path.join(path, filename)
        rospy.loginfo('ready to save')
        cv2.imwrite(file_path, img)

    def save_dimg_raw(self, img_array):
        ia = img_array.astype(np.uint16)
        imaged = imm.fromarray(ia)
        # Save
        filename = '%s_fc_%d_%d.tif'%('raw_depth', self.filecode, self.image_count)
        path = os.path.join(root,'mo_jenga', 'data', 'imgs', 'raw_depth', 'filecode_%d'%self.filecode)
        # print('path creating')
        # print(path)
        if not os.path.exists(path):
            print ('Creating directory: %s'%path)
            os.makedirs(path)
            print('path created')
        file_path = os.path.join(path, filename)
        imaged.save(file_path)







    # SERVICE CALLBACK:****************************************************************************************


    def handle_vision_service(self, args):
        # service handle for the vision

        # work with filecode
        filecode = args.filecode
        layer = args.layer
        row = args.row

        rospy.loginfo('Filecode received: %d'%filecode)
        rospy.loginfo('Previous filecode: %d'%self.filecode)
        if filecode != self.filecode:
            self.plot_pred()

            self.filecode = filecode
            self.image_count = 0

            self.restart_pred()
        else:
            self.image_count += 1


        blocks_pose_list = []
        cv_image = self.get_image()
        # print('we have image')

        # Save the image to process later
        self.save_image(cv_image) # Color
        image_np = self.get_image(color=False)
        araw = self.get_dimg_raw()
        self.save_image(image_np,depth=True)
        self.save_dimg_raw(araw)

        # pass cv_image to Jenga4D Predictor

        # TODO : Uncommment
        num_times = 1
        bnp_raw = []
        # Average Loop
        for i in range(num_times):
            cv_image = self.get_image()
            block_pred, top50_gt_with_scores, mask_fail = self.predictor.predict_4dpos(cv_image,'filecode_%d'%self.filecode, (layer, row), self.image_count)
            median_v = np.median(top50_gt_with_scores[-5:], axis=0)
            mean_v = np.mean(top50_gt_with_scores[-5:], axis=0)
            print('\nMEDIANS: ', median_v[:2], np.degrees(median_v[3]))
            print('\nMEANS: ', mean_v[:2], np.degrees(mean_v[3]))
            print('')
            if mask_fail:
                print('\n\n @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \n')
                print('\t MASK FAILED =====================')
                print('\n @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \n\n')
            self.predictions.append(top50_gt_with_scores[-10:,:5])
            bnp_raw.append(block_pred[np.newaxis, :])
        elements, counts = np.unique(np.array(bnp_raw), axis=0, return_counts=True)
        bnp = elements[np.argmax(counts)]
        # print('done')

        # Fill the block_pose_list
        blocks_pose_list = []
        if False: # DEBUGG:
            # odd layers:
            for i in range(17):
                if i%2 != 0:
                    # layers 2,4,6,8...
                    blocks_pose_list.append({'x': 0.026, 'y': 0.0, 'z': 0.0143*i, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
                    blocks_pose_list.append({'x': 0.0, 'y': 0.0, 'z': 0.0143*i, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
                    blocks_pose_list.append({'x': -0.026, 'y': 0.0, 'z': 0.0143*i, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
                else:
                    # layers 1,3,5,7...
                    blocks_pose_list.append({'x': 0.0, 'y': 0.0, 'z':0.0143*i, 'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0})
                    blocks_pose_list.append({'x': 0.0, 'y': 0.026, 'z':0.0143*i, 'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0})
                    blocks_pose_list.append({'x': 0.0, 'y': -0.026, 'z':0.0143*i, 'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0})

        else:
            MEAN_RES = True
            zaxis = (0, 0, 1)
            #TODO: Remove print
            # print('bnp.shape', bnp.shape)
            if MEAN_RES and row != 2:
                theta = mean_v[3]+np.pi/2.
                # the following 3 lines are for geting the orientation as jenga_tf
                epsilon = np.pi/8
                if layer%2 == 1:
                    theta -= np.pi
                print('x, y: ', (mean_v[0]/100., mean_v[1]/100.))
                print('Theta: ',np.degrees(theta))

                q = [np.cos(theta/2.), 0., 0., 1.0*np.sin(theta/2.)]
                pose_dict = {'x': mean_v[0]/100.,  'y': mean_v[1]/100., 'z':mean_v[2]/100., 'qw': q[0], 'qx': q[1], 'qy': q[2], 'qz': q[3]}
                blocks_pose_list.append(pose_dict)

            else:
                for ind in range(bnp.shape[0]):

                    theta = bnp[ind,3]+np.pi/2.
                    # the following 3 lines are for geting the orientation as jenga_tf
                    epsilon = np.pi/8
                    if layer%2 == 1:
                        theta -= np.pi
                    print('x, y: ', (bnp[ind, 0]/100., bnp[ind, 1]/100.))
                    print('Theta: ',np.degrees(theta))

                    q = [np.cos(theta/2.), 0., 0., 1.0*np.sin(theta/2.)]
                    pose_dict = {'x': bnp[ind, 0]/100.,  'y': bnp[ind, 1]/100., 'z':bnp[ind, 2]/100., 'qw': q[0], 'qx': q[1], 'qy': q[2], 'qz': q[3]}
                    blocks_pose_list.append(pose_dict)
        #TODO: Remove print
        # print(len(blocks_pose_list))

        # write the output to a file
        str_test =  "[VISION] -  ... {}".format(blocks_pose_list[0]['z'])
        rospy.loginfo(str_test)
        os.system('echo {} > test.txt'.format(str_test))

        # Pack the values observed into a BlocksPose msg
        bp = self.pack_blocks(blocks_pose_list)

        #print(bp)

        # Return the service result
        return bp


if __name__ == "__main__":
    try:
        v = Vision()
        rospy.loginfo("[VISION] - Vision service running ...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[VISION] - Vision service stopped.")
        pass
