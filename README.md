# zumo
Custom code for my Zumo 32u4 robot

Arduino project folders:

<h2>around_the_box</h2>

A simple program that will make right turns to go around a box.  The goals are to go around an arbitrary box without touching it and then return as close as possible to the starting location.  As an additional goal, I decided to make it fast.  This wasn't a goal, but I thought it made it more fun.

I use the internal gyro to execute precise 90 degree angles.  I start by calibrating the gyroscope while the robot is at rest, then I use a pd loop to quickly make the turns. I also use the gyro to maintain straight lines when going along the edges of the box.

I use the side proximity detectors to detect when the robot has gone past the edge of the box.  This part took a little fiddling to understand what the different readings represented, but the end result worked OK.

I use odometry to track the position of the robot and then at the final stage, I have it return to its original position.

<h2>follow_the_line</h2>
A traditional line follower program that will follow a dark line on a light surface.

[![Alt text](https://img.youtube.com/vi/TDYIutR5_Wo/0.jpg)](https://www.youtube.com/watch?v=TDYIutR5_Wo)<br/>
https://www.youtube.com/watch?v=TDYIutR5_Wo

<h2>zumo_clock</h2>
This program makes the zumo robot rotate in place like the second hand of a clock.  It moves once second and uses the gyro to maintain position, so it will return to the correct position even if disturbed.

<h2>zumo_compas</h2>
Playing with the on-board compass.
