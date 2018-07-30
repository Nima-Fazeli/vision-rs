#!/usr/bin/env python

from ros_vision_pred import Vision


def main():
    
    vp = Vision()
    
    test = vp.getImage()

if __name__ == '__main__':
    main()
