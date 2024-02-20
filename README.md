# ProbabilisticDeminingRobot
*Found this little gem in my private repo on UGent's Enterprise GitHub, and thought I'd share it with the rest of the world.*
![Animation of demining robot performing a probabilistic search in the mine field](https://github.com/cgoemaere/ProbabilisticDeminingRobot/blob/main/probabilistic_demining_robot_animation.gif)

**Context:** For the Robotics course in the first year of my Master's studies, the task was to make a demining robot. Whereas most people focussed on the robot's operating system (ROS) or the 3D simulation in Gazebo, I decided to work on the algorithmic aspect of the problem.

## Introduction
What is the optimal route for a demining robot to take? A full sweep of the area is expensive, and does not take prior information into account. If we have prior knowledge on the mine laying patterns ([which is](https://www.globalsecurity.org/military/library/policy/army/fm/20-32/chap7.html) [typically the case](https://www.reddit.com/r/WarCollege/comments/fe5emr/are_there_patterns_used_in_laying_minefields/)), we may find a faster way to demine the area, by determining the locations that have the highest probability of containing a mine.

## Algorithm
This Python file simulates such a probabilistic demining robot. Broadly speaking, this is how it works:
1) Generate all possible mine patterns. In this file, we assume mines only occur in an $n{\times}n$-grid, with variations in tilting, and interrow and intercolumn distance.
2) Select one pattern at random as ground truth.
3) Initialize robot somewhere on the map in a specific direction. We chose the upper left corner of the area, with the robot facing right.
4) Repeat procedure below until all mines are found.
	1) Check ground truth to see if robot is currently standing on a mine (or near enough to make it explode, depending on robot and mine dimensions).
	2) Eliminate all possible patterns that do not conform to the previous observation of there being a/no mine at current position.
	3) From the remaining possible patterns, build a probability density map of mines in the area. To avoid sharp peaks in our optimization landscape, we model each mine with a Gaussian instead of a Dirac delta. The variance of the Gaussian depends on the number of mines we have already hit (more hits = sharper peak).
	4) Iterate over all possible directions the robot can move along. Use gradient ascent on the probability density map to determine the optimal angle in which the robot should position itself.
		* A line of zero probability (i.e., somewhere the robot has already passed, but found no mines) would act as an impermeable boundary for vanilla gradient ascent. To avoid this, we work with the nearest non-zero probability in the given direction, adjusted by a distance penalty (having to cross many zeros is undesirable).
		* Turning the robot incurs an additional cost, which in hindsight seems similar to adding momentum.
		* We limit turning to the range [-90°, 90°].
	5) Turn the robot according to the optimal angle, and move a single block forward (i.e., along the optimal direction).

## Simulation details
In this discrete-space simulation, we choose the sizing of the coordinate grid such that the maximum speed of the robot equals 1 block/s. As the physical robot and mines have a fixed diameter (in meters), this means that their width in the coordinate grid may exceed a single block if the maximum speed is low enough. Note that the simulation was not tested extensively for this case.

We also keep track of the time the robot requires to perform the demining sweep, taking into account its maximum forward and rotational speed. This allows us to objectively assess the benefits of our approach compared to other methods, like full grid search. (This part of the code is commented out at the bottom of the file)
