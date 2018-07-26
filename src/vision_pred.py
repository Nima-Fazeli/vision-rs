# Script to take an image as input and output the 3D position and rotation

import os, sys
import argparse
import time

sys.path.insert(0, '/home/robot2/anaconda3/envs/python-3.5/lib/python3.5/site-packages')

from Jenga4D.predict_4dpos import Jenga4Dpos

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'Jenga4D'))
# print(sys.path)

def predict_single_image(image_cnt, predictor):
	image_dir = os.path.join('/home/robot2/data-color', 'color_' + str(image_cnt) + '.jpg')
	pos4ds = predictor.predict_4dpos(image_dir)
	return pos4ds
	
def main(args):
	tic = time.time()
	os.system('rosrun rs2image dc_capt_inf ' + args.image_cnt)
	print('total time cost: {}s'.format(time.time()-tic))
	
	predictor = Jenga4Dpos()
	
	pos4ds = predict_single_image(int(args.image_cnt) , predictor)
	# print(pos4ds)
	
def parse_args():
	argv = sys.argv
	parser = argparse.ArgumentParser(description='')
	parser.add_argument('--image_cnt', help='the image count pass to the main function', 
			required=True, type=str)
		
	args = parser.parse_args()
	return args
	
if __name__ == '__main__':
	args = parse_args()
	main(args)
