#!/usr/bin/env python
import roslib
roslib.load_manifest('marker_print')
import rospy
import cv
import sys
import commands
import os.path
import math

DEBUGGING = False

class MarkerImageGrid(object):
    
    def __init__(self):
        self.load_bch_markers()
        self.IMG_SPACING = 4
        self.IMG_ROWS = 64
        self.IMG_COLS = 64
        self.MARKER_SIZE = 8
        self.BORDER_SIZE = 2
        self.MAX_ID = self.IMG_COLS*self.IMG_ROWS-1

    def load_bch_markers(self):
        self.img = cv.LoadImageM(os.path.join(\
                commands.getoutput('rospack find marker_print'),\
                'images/bch_thin_markers.png'), cv.CV_LOAD_IMAGE_GRAYSCALE)

    def get_marker(self, id):
        if not isinstance(id, int) or id > self.MAX_ID or id < 0:
            sys.stderr.write('Invalid id: %d.\n' % id)
            sys.exit(1)
        xp = id%self.IMG_COLS*(self.IMG_SPACING+self.MARKER_SIZE)+\
                self.IMG_SPACING
        yp = id/self.IMG_COLS*(self.IMG_SPACING+self.MARKER_SIZE)+\
                self.IMG_SPACING
        return cv.GetSubRect(self.img, (xp, yp, self.MARKER_SIZE,\
                self.MARKER_SIZE))

    def scale_marker(self, img, scale_factor):
        scaled = cv.CreateMat(img.rows*scale_factor, img.cols*scale_factor,\
                cv.CV_8UC1)
        cv.Resize(img, scaled, cv.CV_INTER_AREA)
        return scaled

    def get_markers_scaled(self, ids, scale_factor):
        markers = list()
        for id in ids:
            markers.append(self.scale_marker(self.get_marker(id), scale_factor))
        return markers

class MarkerPageLayout():

    def __init__(self, layout, width, height, margin):
        self.L2R = 0
        self.T2B = 1
        self.layout = layout
        self.px_width = int(width*28.57)
        self.px_height = int(height*28.57)
        self.px_margin = int(margin*28.57)
        self.grid = MarkerImageGrid()

    def layout_markers(self, imgs):
        size = imgs[0].rows
        scale = size/self.grid.MARKER_SIZE
        spacing = self.grid.IMG_SPACING*scale + 1

        count = len(imgs)
        max_rows = int((self.px_height-self.px_margin*2)/(size+spacing))
        max_cols = int((self.px_width-self.px_margin*2)/(size+spacing))
        if max_cols == 0 or max_rows == 0:
            sys.stderr.write('Number of markers with given scale cannot fit on page.\n')
            sys.exit(1)

        if self.layout == self.L2R:
            rows = int(math.ceil(float(count)/max_cols))
            cols = min(count, max_cols)
        else:
            cols = int(math.ceil(float(count)/max_rows))
            rows = min(count, max_rows)

        if DEBUGGING:
            print '\n'.join((('size: %d, scale: %d, spacing: %d, count: %d, '+\
                    'max_rows: %d, max_cols: %d, rows: %d, cols: %d') % (size,\
                    scale, spacing, count, max_rows, max_cols, rows, cols)).\
                    split(', '))

        if cols > max_cols or rows > max_rows or rows*cols < count:
            sys.stderr.write('Number of markers with given scale cannot fit on page.\n')
            sys.exit(1)

        self.laid_out_img = cv.CreateMat(rows*(spacing+size)+spacing,\
                cols*(spacing+size)+spacing, cv.CV_8UC1)

        cv.Set(self.laid_out_img, 255)

        for i in range(0, len(imgs)):
            if self.layout == self.L2R:
                xp = i%cols*(spacing+size)+spacing
                yp = i/cols*(spacing+size)+spacing
            else:
                xp = i/rows*(spacing+size)+spacing
                yp = i%rows*(spacing+size)+spacing
            roi = cv.GetSubRect(self.laid_out_img, (xp, yp, size, size))
            cv.Copy(imgs[i], roi)

    def draw_dotted_lines(self, marker_size, spacing):
        for i in range(spacing/2+1, self.laid_out_img.rows,\
                marker_size+spacing):
            for j in range(spacing/2+1, self.laid_out_img.cols-spacing/2, 2):
                cv.Set2D(self.laid_out_img, i, j, 200)
        for i in range(spacing/2+1, self.laid_out_img.cols,\
                marker_size+spacing):
            for j in range(spacing/2+1, self.laid_out_img.rows-spacing/2, 2):
                cv.Set2D(self.laid_out_img, j, i, 200)

# Maybe later write wrapper for lpr
class Printer(object):
    pass

def parse():
    import argparse

    parser = argparse.ArgumentParser(description='Utility to generate an\
            image of scaled markers for printing')
    parser.add_argument('-b', '--begin', dest='begin', type=int,\
            default=0, help='ID of beginning marker (starting at 0)')
    parser.add_argument('-e', '--end', dest='end', type=int,\
            required=True, help='ID of ending marker (max 4095)')
    parser.add_argument('-s', '--scale', dest='scale', type=int,\
            default=8, help='Factor by which to scale 8x8 pixel markers\
            (preferably power of 2)')
    parser.add_argument('-l', '--layout', dest='layout', type=int,\
            default=0, help='Print layout: 0 - Left to right, then top to\
            bottom, 1 - Top to bottom, then left to right')
    parser.add_argument('-w', '--page_width', dest='page_width', type=float,\
            default=21.59, help='Page width in cm')
    parser.add_argument('-x', '--page_height', dest='page_height', type=float,\
            default=27.94, help='Page height in cm')
    parser.add_argument('-m', '--page_margin', dest='page_margin', type=float,\
            default=2.0, help='Minimum page margin in cm')
    parser.add_argument('-o', '--out_file', dest='out_file', type=str,
            required=True, help='File to save image to be printed')
    parser.add_argument('-q', '--quiet', dest='quiet', action='store_true',
            required=False, help='Prevents printing or displaying for use in scripts')
    return parser.parse_args()

def main(args):
    mg = MarkerImageGrid()
    imgs = mg.get_markers_scaled(range(args.begin, args.end+1), args.scale)
    pl = MarkerPageLayout(args.layout, args.page_width, args.page_height,\
            args.page_margin)
    pl.layout_markers(imgs)
    pl.draw_dotted_lines(imgs[0].rows, imgs[0].rows/mg.MARKER_SIZE*mg.IMG_SPACING+1)
    if not args.quiet:
        cv.ShowImage('Marker Print Image', pl.laid_out_img)
        print "Hit 's' to save image, or any other key to quit."
        k = cv.WaitKey()
        if k == 1048691 or k == 115:
            cv.SaveImage(args.out_file, pl.laid_out_img)
            print 'Saved.'
        else:
            print 'Exiting.'
    else:
        cv.SaveImage(args.out_file, pl.laid_out_img)
if __name__ == '__main__':
    args = parse()
    try:
        main(args)
    except rospy.ROSInterruptException: pass

