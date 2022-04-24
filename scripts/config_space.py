import os
from tqdm.autonotebook import tqdm
import cv2 
from math import sqrt
import numpy as np


class Arena():
    def __init__(self, clearance, rob_dia, find_obstacle, grid_size) -> None:
        self.clearance = clearance
        self.rob_rad = rob_dia/2
        self.find_obstacle = find_obstacle # find only obstacle (without the clearance/robot radius) around it
        self.grid_size = grid_size 
        # self.grid = np.zeros((self.grid_size, self.grid_size, 3)) # will hold grayscale image showing navigable & unnavigable space
        # ith row of self.grid    represents y=1e4-1-i in cartesian coords #i varies from [0,9999]
        # jth column of self.grid represents x=j
        self.obstacle_points = [] # contains (row, column) for all points on all obstacles (without the clearance/robot radius around it)

    def square_outer(self, x, y): #return true if the point is inside the map while considering safe clearance & non-zero rob radius
        is_inside = False
        if (x > 0+self.clearance+self.rob_rad) and (x < 9999-self.clearance-self.rob_rad):
            if (y > 0+self.clearance+self.rob_rad) and (y < 9999-self.clearance-self.rob_rad):
                is_inside = True
        if self.find_obstacle:
            if x==0 or x==9999 or y==0 or y==9999:
                self.obstacle_points.append([self.grid_size-1-y, x])
        return is_inside
        

    def circle1(self, x, y): #return true if the point is outside the circle while considering safe clearance & non-zero rob radius
        is_outside = False
        if (((x-2000)**2 + (y-2000)**2) - ((1000+self.clearance+self.rob_rad)**2)) > 0:
            is_outside = True
        if self.find_obstacle:
            if (((x-2000)**2 + (y-2000)**2) - ((1000)**2)) <= 0:
                self.obstacle_points.append([self.grid_size-1-y, x])
        return is_outside

    def circle2(self, x, y): #return true if the point is outside the circle while considering safe clearance & non-zero rob radius
        is_outside = False
        if (((x-2000)**2 + (y-8000)**2) - ((1000+self.clearance+self.rob_rad)**2)) > 0:
            is_outside = True
        if self.find_obstacle:
            if (((x-2000)**2 + (y-8000)**2) - ((1000)**2)) <= 0:
                self.obstacle_points.append([self.grid_size-1-y, x])
        return is_outside
    
    def square_inner(self, x, y): #return true if the point is outside the square while considering safe clearance & non-zero rob radius
        is_outside = False
        if (x < 250-self.clearance-self.rob_rad) or (x > 1750+self.clearance+self.rob_rad) or\
            (y < 4250-self.clearance-self.rob_rad) or (y > 5750+self.clearance+self.rob_rad):
                is_outside = True
        if self.find_obstacle:
            if x >= 250 and x <= 1750:
                if y >= 4250 and y <= 5750:
                    self.obstacle_points.append([self.grid_size-1-y, x])
        return is_outside

    def rect_horiz(self, x, y): #return true if the point is outside the rect while considering safe clearance & non-zero rob radius
        is_outside = False
        if (x < 3750-self.clearance-self.rob_rad) or (x > 6250+self.clearance+self.rob_rad) or \
            (y < 4250-self.clearance-self.rob_rad) or (y > 5750+self.clearance+self.rob_rad):
                is_outside = True
        if self.find_obstacle:
            if x >= 3750 and x <= 6250:
                if y >= 4250 and y <= 5750:
                    self.obstacle_points.append([self.grid_size-1-y, x])
        return is_outside
    
    def rect_vert(self, x, y): #return true if the point is outside the rect while considering safe clearance & non-zero rob radius
        is_outside = False
        if (x < 7250-self.clearance-self.rob_rad) or (x > 8750+self.clearance+self.rob_rad) or \
            (y < 2000-self.clearance-self.rob_rad) or (y > 4000+self.clearance+self.rob_rad):
                is_outside = True
        if self.find_obstacle:
            if x >= 7250 and x <= 8750:
                if y >= 2000 and y <= 4000:
                    self.obstacle_points.append([self.grid_size-1-y, x])
        return is_outside

    def is_navigable(self, x,y=-1): #Returns True if the point of interest lies within navigable space
        try:
            if isinstance(x, list):
                assert y == -1
                y = x[1]
                x = x[0]
            else:
                assert isinstance(x,int)
                assert isinstance(y,int) and y != -1
        except Exception as e:
            print(e)
            return False
        if self.square_outer(x,y) \
            and self.square_inner(x,y) and self.circle1(x,y) and self.circle2(x,y) and self.rect_horiz(x,y) and self.rect_vert(x,y):
            return True
        return False

def main(clearance, display):
        diameter = 210 #mm Robot diameter
        L = 160 #mm. Distance b/w wheels
        grid_size = int(1e4) #in mm.
        arena = Arena(clearance, diameter, True, grid_size)
        grid = np.zeros((grid_size, grid_size, 3)) #The obstacles are colored black & the navigable space is colored white.

        if os.path.exists('grid_{}.npy'.format(clearance)):
            grid = np.load('grid_{}.npy'.format(clearance))
        else:
            print('Pre-Existing grid file not found. ')
            with tqdm(total = grid_size) as pbar:
                for row in range(grid_size):
                    pbar.update(1)
                    for col in range(grid_size):
                        if arena.is_navigable(col, grid_size-1-row):
                            grid[row, col] = [255, 255, 255]
                        else:
                            grid[row, col] = [0, 255, 0]
                    
            for obstacle_point in arena.obstacle_points:
                row = obstacle_point[0]
                col = obstacle_point[1]
                grid[row, col] = [0, 0, 0]
            np.save(open('grid_{}.npy'.format(clearance), 'wb'), grid)
        
        if os.path.exists('nav_space_{}.npy'.format(clearance)):
            nav_space = np.load('nav_space_{}.npy'.format(clearance))
        else:
            nav_space = np.ones((grid_size, grid_size))
            for row in range(grid_size):
                for col in range(grid_size):
                    if not np.array_equal(grid[row, col], [255, 255, 255]):
                        nav_space[row, col] = 0
            nav_space = nav_space.astype(np.uint8)
            np.save(open('nav_space_{}.npy'.format(clearance), 'wb'), nav_space)

        if display == True:
            nav_space_disp = (nav_space*255).astype(np.uint8)
            nav_space_disp = cv2.resize(nav_space_disp, (int(grid_size/10), int(grid_size/10)))

            cv2.imshow('nav_space_disp', nav_space_disp)

            grid_disp = grid.astype(np.uint8)
            grid_disp = cv2.resize(grid_disp, (int(grid_size/10), int(grid_size/10)))
            cv2.imshow('grid_disp', grid_disp)
            cv2.imwrite('empty_grid.jpg', grid_disp)
            cv2.waitKey(0)

        return nav_space, grid