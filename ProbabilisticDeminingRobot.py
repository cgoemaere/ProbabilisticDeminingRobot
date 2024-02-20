# -*- coding: utf-8 -*-
"""
Created on Sun Dec  6 16:31:11 2020

@author: Cédric Goemaere
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import matplotlib.animation

np.random.seed(42) #for reproducability

class ProbRobot:
    def __init__(self, reset=False): #reset so that we can do self.run() multiple times without calculating all_patterns every time
        #Turtlebot3 characteristics (maximum speeds)
        self.speed = 0.22 #m/s = 1 gridblock/s (max. speed is 0.22m/s)
        #Note: if self.speed <=0.17, then the required distance to hit a mine will go from 0 to 1,
        #meaning that driving over a cell just next to the mine will be good enough to hit it.
        self.radspeed = 2.84 #rad/s (max. rad speed is 2.84rad/s)
        
        #Adjusting distances to gridblock sizes
        self.mine_radius = 0.03/self.speed
        self.radius = 0.14/self.speed
        
        #Environment characteristics
        self.mapheight = int(np.round(6/self.speed)) #6m = 27 gridblocks
        self.mapwidth = int(np.round(6/self.speed)) #6m = 27 gridblocks
        #Generate all coords in a grid
        self.coordgrid = np.mgrid[0:self.mapheight, 0:self.mapwidth]
        
        #Starting coordinates
        self.x = 0
        self.y = 0
        self.angle = 0.0 #direction robot is facing
        
        #Initializing variables for mines
        self.n = 3 #Create a nxn mine pattern
        self.nr_mines = self.n**2
        self.mines = None #Chosen mine pattern
        self.minesfound = [] #list of mines already found
        self.just_hit = False
        self.hits = 0
        self.hitstime = [] #list of times at which the mines were found
        self.time = 0.0 #real runtime
                
        self.probmap = None #probability map of environment concerning whether there is a mine
        
        #Generate data needed for prob update (only on initialization)
        if not reset:
            self.all_patterns = [] #list of all possible patterns (to be expanded)
            self.create_possible_mine_patterns()
        
        else: #Don't do this on initialization to save some time
            #For now, assume all patterns are possible
            self.pos_patterns = np.arange(len(self.all_patterns)).tolist() #list of indices of possible patterns (will be updated)
            
            #Choose one pattern as real minefield
            self.create_minefield()
            
            #Create initial environment
            self.reset_environment(begin=True)
            
            #Make an initial map based on all possible patterns
            self.update_probmap()
    
    def create_possible_mine_patterns(self):
        #Generate all possible patterns (with self-imposed boundaries on the ranges)
        print("Generating all possible mine patterns...")
        self.all_patterns = []
        
        #Generates this amount of angles between -pi/4..pi/4
        angle_nr = 8 
        all_angles = np.linspace(-np.pi/4, np.pi/4, angle_nr)
        #To speed things up
        all_cos = np.cos(all_angles)
        all_sin = np.sin(all_angles)
        
        #Use local variables instead of self pointers to save time
        mapheight = self.mapheight
        mapwidth = self.mapwidth
        
        for interrowspace in range(1, 5): #including 4, but not 5
            for intercolspace in range(1, 5):
                print("Generated", 100*(interrowspace*4+intercolspace-5)/16, "% of the patterns...")
                for begin_x in range(1, int(mapwidth-self.n*interrowspace)):
                    for begin_y in range(1, int(mapheight-self.n*intercolspace)):
                        begin_point_patterns = [] #patterns belonging to this begin point
                        
                        for ang_indx in range(angle_nr):
                            #angle = all_angles[ang_indx] #not needed
                            valid_angle = True

                            cos = all_cos[ang_indx]
                            sin = all_sin[ang_indx]
                            
                            pattern = np.zeros((mapheight, mapwidth), dtype=np.uint8)
                            for i in range(self.n):
                                x = interrowspace*i
                                for j in range(self.n):
                                    y = intercolspace*j
                                    y_rot = int(np.round(cos*y + sin*x + begin_y))
                                    x_rot = int(np.round(-sin*y + cos*x + begin_x))
                                    #Check if there is already a mine there
                                    if y_rot < 0 or y_rot >= mapheight or x_rot < 0 or x_rot >= mapwidth:
                                        valid_angle = False #search something else
                                        break
                                    if pattern[y_rot, x_rot]: #Already a mine here
                                        valid_angle = False
                                        break
                                    else:
                                        pattern[y_rot, x_rot] = 1
                                
                                #break i range
                                if not valid_angle:
                                    break
                                
                            else: #i range did not hit break-statement
                                for check_pattern in begin_point_patterns:
                                    #we dont want duplicates
                                    if np.all(pattern == check_pattern):
                                        break
                                else: #not a duplicate
                                    begin_point_patterns.append(pattern)
                        
                        #Add all patterns belonging to this begin point
                        self.all_patterns.extend(begin_point_patterns)
        
        print("Generated", len(self.all_patterns), "patterns!")
    
    def create_minefield(self):
        index = np.random.randint(len(self.all_patterns))
        pattern = self.all_patterns[index]
        self.mines = np.argwhere(pattern == 1).tolist()

    def reset_environment(self, begin=False):
        #Initialization of environment
        if not begin:
            index = (self.probmap != 0) #Don't reset where we have already been
            self.probmap[index] = np.ones((self.mapheight, self.mapwidth))[index]
        else: #In the begin, we have no probmap yet!
            self.probmap = np.ones((self.mapheight, self.mapwidth))
    
    def look_close_to(self, x, y, multiplier=1):
        #Function to precise where probabilities should be increased
        
        rowcoords, colcoords = self.coordgrid
        #Calculate the distance from this grid to the wanted (x,y)
        distance_from_point = (rowcoords-y)**2 + (colcoords-x)**2

        #To not multiply everything with near-zeros, we add 1
        #To create sharp tops, we multiply with 16
        #However, with more mines hit, we want to be better at finding it anywhere
        #The /(1+self.hits)**2 makes sure the cones get wider if we find more mines
        self.probmap *= 1+multiplier*np.exp(-self.nr_mines/(1+self.hits)**2*distance_from_point)
        
        # Normalize the probmap
        self.probmap /= np.sum(np.sum(self.probmap))

    def find_best_angle_and_move(self):
        anglerange_turn = np.array([-np.pi/2, -np.pi/4, 0, np.pi/4, np.pi/2])
        turningcost = anglerange_turn**2
        #Make sure the turning cost is of decent scale
        turningcost *= np.max(self.probmap)/(1.2*np.max(turningcost))
        
        #Add the current angle to the range
        anglerange = self.angle + anglerange_turn
        
        chance = []
        for a in anglerange:
            prob = 0
            i = 1
            #To speed thing up
            sin = np.sin(a)
            cos = np.cos(a)
            while prob < 1e-200: #already having been there is not that bad
                indx_y = int(np.round(self.y+i*sin))
                indx_x = int(np.round(self.x+i*cos))
                if indx_x < 0 or indx_x >= self.mapwidth or indx_y < 0 or indx_y >= self.mapheight:
                    prob = -10 #Lower than anything else
                    break
                else:
                    prob = self.probmap[indx_y, indx_x]
                    i += 1
                    
            chance.append(prob/np.sqrt(i)) #too long distance is not good (unless all scores are negative, then it is good)
        
        chance = np.array(chance)
        
        if max(chance) == -10: #every option was terrible and lead to certain death (only if robot comes in corner at 45°)
            self.angle += np.pi
            turned_angle = np.pi
            print("Terrible options! Turned around!")
        
        else: #this will happen almost always
            chance /= (turningcost+1)
            
            #Keep turned angle for time reasons
            turned_angle = 0.0
            
            #Prefer to go straight, but change if necessary (argmax takes first occurence)
            if chance[2] != max(chance):
                max_index = np.argmax(chance)
                self.angle = anglerange[max_index]
                turned_angle = anglerange_turn[max_index]
        
        #Move
        self.probmap[self.y, self.x] = 0 #there cannot be a (new) mine if we have been there
        self.x += int(np.round(np.cos(self.angle)))
        self.y += int(np.round(np.sin(self.angle)))
        
        #Add time needed for move
        self.time += (1+turned_angle/self.radspeed)
    
    def detect_hit(self):
        for y_mine, x_mine in self.mines:
            if (self.x-x_mine)**2 + (self.y-y_mine)**2 < (self.radius + self.mine_radius)**2:
                self.hits += 1
                self.hitstime.append(self.time)
                self.mines.remove([y_mine, x_mine])
                self.minesfound.append((y_mine, x_mine))
                print('Mine found at', (y_mine, x_mine))
                
                self.just_hit = True
                break
        else: #we did not break the for loop
            self.just_hit = False
    
    def update_probmap(self):
        multiplier_map = np.zeros((self.mapheight, self.mapwidth))
        new_pos_patterns = []
        
        self.reset_environment()
        
        #If he just hit a mine, check for '1'
        #If he passed a place where there is a known mine, check for '1'
        #Else, check for '0'
        value_to_check = int(self.just_hit or (self.y, self.x) in self.minesfound)
        
        for indx in self.pos_patterns:
            pattern = self.all_patterns[indx]
            #If pattern is what we know to be true, add it to the possibilities
            if pattern[self.y, self.x] == value_to_check:
                new_pos_patterns.append(indx)
                multiplier_map += pattern
        
        #Debug code:
        if np.max(multiplier_map) == 0 and len(self.pos_patterns) != 0: #just to be sure
            print('No patterns left!')
            print('We are at y =', self.y, 'and x =', self.x)
            print('These were the last possible patterns:')
            for i in self.pos_patterns:
                print(np.argwhere(self.all_patterns[i] == 1).tolist())
        #Else: everything works fine! (max != 0)
        else:
            multiplier_map /= np.max(multiplier_map)
        
        for (i, j) in self.minesfound: #we don't want to promote already found mines
            multiplier_map[i, j] = 0
        
        #Apply changes to probmap
        for i, row in enumerate(multiplier_map):
            for j, mult in enumerate(row):
                if mult > 2**-10: #to speed things up
                    self.look_close_to(j, i, multiplier=mult)
        
        #Only save possible patterns
        self.pos_patterns = new_pos_patterns
    
    def plot(self, ims):
        artist = []
        artist.append(plt.imshow(self.probmap, animated=True, vmin = 0, vmax = np.max(self.probmap)))
        artist.append(plt.scatter(self.x, self.y, color='r'))
        for mine in self.minesfound:
            artist.append(plt.scatter(mine[1], mine[0], color='b'))
        for mine in self.mines:
            artist.append(plt.scatter(mine[1], mine[0], color='w'))
        ims.append(artist)
    
    def run(self):
        #Reset all values
        print('\n\nStarting new simulation...')
        self.__init__(reset=True)
        
        print('Mines at:')
        print(self.mines)
        fig = plt.figure()
        ims = []
        counter = 0
        ani = None
        
        while self.hits != self.nr_mines:   
            #Check if we hit a mine
            self.detect_hit()
            
            #Update probmap
            update_start_time = time.time()
            self.update_probmap()
            time_needed_for_update = time.time() - update_start_time
            
            #Plotting
            self.plot(ims)
            
            #Decision making for next move
            self.find_best_angle_and_move()
            
            counter += 1
                
            if counter%10 == 0 or self.just_hit:
                print('Run', counter, ',', self.hits, 'mines hit')
                print('Still', len(self.pos_patterns), 'patterns left...')
                print('Update took', time_needed_for_update)
            if counter == self.mapwidth*self.mapheight: #worse than sweeping
                print('Algorithm failed')
                break
        print('Found all mines!')
        plt.colorbar(ims[0][0]) #not super representative, but cant animate colorbar :(
        ani = matplotlib.animation.ArtistAnimation(fig, ims, blit=True, repeat=True,interval=100)
        return ani

#Initializing to generate all possible mine patterns (only do this once to save time)
robot = ProbRobot()
ani_list = []

first_mine = []
all_mines = []
for i in range(1): #run multiple times (max 6, because animations take a lot of memory)
    ani = robot.run()
    ani_list.append(ani) #so that it is also shown (just to be sure)
    # ani.save('animation_%i.gif'%(i), fps=10)
    
    # first_mine.append(robot.hitstime[0])
    # all_mines.append(robot.hitstime[-1])
    
    # try:
    #     fig = plt.figure()
    #     ax = plt.gca()
    #     ax.set_title("Time it took to hit the x-th mine")
    #     ax.set_ylabel("Time (s)")
    #     ax.set_xlabel("X-th mine hit")
    #     plt.plot(np.arange(1, robot.nr_mines+1), np.diff(np.array([0.0] + robot.hitstime)))
    # except ValueError: #if the algorithm fails, hittime will not be long enough
    #     print('Did algorithm fail?')


# print('First mine found in', np.percentile(first_mine, [0, 25, 50, 75, 100]))
# print('Mean is', np.mean(first_mine))
# print('All mines found in', np.percentile(all_mines, [0, 25, 50, 75, 100]))
# print('Mean is', np.mean(all_mines))
# print('All mines found in', np.percentile(np.array(all_mines)-np.array(first_mine), [0, 25, 50, 75, 100]))
# print('Mean is', np.mean(np.array(all_mines)-np.array(first_mine)))
plt.show()
# plt.close('all')