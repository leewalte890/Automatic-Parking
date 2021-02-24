#!/usr/bin/env python
'''
Simple Image Labeler

Enables user to specify a target with a closed polygon, and converts this to a mask image

Usage:  python labeler image_name

Note: output mask is save as <image_name>_mask.png, and is a 3-channel image.  To use it
you will likely need to select just one channel.

Daniel Morris, Jan 2020
'''
from __future__ import print_function
import numpy as np
import os
import cv2
import argparse

class PolygonMaker(object):
    def __init__(self, window_name):
        self.window_name = window_name  # Name for our window
        self.done = False  # Flag signalling we're done
        self.current = (0, 0)  # Current position, so we can draw the line-in-progress
        self.points = []  # List of points defining our polygon

    def on_mouse(self, event, x, y, buttons, user_param):
        # Mouse callback that gets called for every mouse event (i.e. moving, clicking, etc.)
        if self.done:  # Nothing more to do
            return
        if event == cv2.EVENT_MOUSEMOVE:
            # We want to be able to draw the line-in-progress, so update current mouse position
            self.current = (x, y)
        elif event == cv2.EVENT_LBUTTONDOWN:
            # Left click means adding a point at current position to the list of points
            self.points.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN:
            # Right click means we're done
            self.done = True

    def run(self, img, outName):
        # Let's create our working window and set a mouse callback to handle events
        cv2.namedWindow(self.window_name)
        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)
        cv2.setMouseCallback(self.window_name, self.on_mouse)

        while not self.done:
            # This is our drawing loop, we just continuously draw new images
            # and show them in the named window
            canvas = img.copy()
            if len(self.points) > 0:
                # Draw all the current polygon segments
                cv2.polylines(canvas, np.array([self.points]), False, (255,255,255), 1)
                # And  also show what the current segment would look like
                cv2.line(canvas, self.points[-1], self.current, (127,127,127))
            # Update the window
            cv2.imshow(self.window_name, canvas)
            # And wait 50ms before next iteration (this will pump window messages meanwhile)
            if cv2.waitKey(50) == 27:  # ESC hit
                self.done = True

        #Create mask from polygon
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, np.array([self.points]), (255,255,255))

        #Show mask overlayed on image for user to validate:
        canvas = (img.copy() / 2.0 + mask / 2.0).astype(np.uint8)        
        cv2.imshow(self.window_name, canvas)

        print("Press a key to save mask, or 'q' to quit without saving")
        if (cv2.waitKey() & 0xff) == ord('q'):
            print('Quitting without saving')
        else: 
            print('Saving mask to:', outName)
            cv2.imwrite( outName, mask )

        cv2.destroyWindow(self.window_name)

# ============================================================================

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Polygon Arguments')
    parser.add_argument('imgName', type=str, metavar='PATH', help='Full path of image name')
    parser.add_argument('maskName', type=str, metavar='PATH', help='Full path of mask name')

    args = parser.parse_args()

    if os.path.isfile(args.imgName):
        print("Reading in image:", args.imgName)
        img = cv2.imread(args.imgName)
        root, _ = os.path.splitext( args.imgName)    
        outName = root + '_mask.png'
        pd = PolygonMaker("Labeler")  # Initialize labeler
        pd.run(img, outName)          # Run
    else:
        print("Error: unable to find:", args.imgName)

