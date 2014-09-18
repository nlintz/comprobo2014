Which behaviors did you implement?
====================
For this project, I implemented wall following and obstacle avoidance using PID controllers.

For each behavior, what strategy did you use to implement the behavior?
====================
For the wall follower, I implemented a complementary PID control system. One PID control pushed the robot to be more parallel to the wall. The other PID controller pushed the robot to be the target distance from the wall. The weighted combination of these two control systems yielded a good result where the robot could handle tight corners and follow a wall accurately.

For the finite state controller, which behaviors did you combine and how did you detect when to transition between behaviors?
====================
For the FSM I combined the wall follower and the obstacle avoider. I have the robot follow the wall unless an object gets too close to the robot. In this case, the robot's obstacle avoidance procedure is run. My detection algorithm is unfortunately naiive as I ran out of time to make something more sophisticated. I just check to see if there are any objects which are too close to the robot and transition to obstacle avoidance if this is the case.

How did you structure your code?
====================
My code is structured into two parts. There is an abstract PID controller class which implements general PID behavior. I tested this class using numpy by plotting the controllers response to different set targets.

The second part of my code is the behaviors. These files include WallFollowing, ObstacleAvoidance, and the finite state machine. By separating my code into two classes, a controller class, and behavior classes I was able to create testable, isolated components which could easily implement the desired behaviors.

What if any challenges did you face along the way?
====================
There were three main challenges I faced with this project. First was the time constraint. I wish we had more time to work on the project so I could have implemented all three behaviors and I could have more time to test my code. Second, I had some trouble implementing the wall avoidance algorithm. In the handout, a preferred direction model is described. However, there are no absolute directions relative to the robot so it doesn't really make sense to use this model to the best of my understanding. To clarify, I wanted the robot to move in an absolute heading without using the odometer so it could dodge an object and keep moving. Howeover, that didn't work out particularly well. Finally, I had some challenges picking appropriate coefficients for my robot. Namely, the PID constants were difficult to pick because it was hard for me to tell if my algorithm was wrong of if I just picked strange numbers.

What would you do to improve your project if you had more time?
====================
If I had more time, I would run optimization algorithms to pick better coefficients for all of my beahviors. I would have also implemented the person following algorithm as well as the wall following algorithm. I would have liked to spend more time on my FSM and on my obstacle avoider as well. Finally, I would have liked to use better algorithm for wall detection e.g. the Hough transform.

Did you learn any interesting lessons for future robotic programming projects?
====================
From this project, I learned how to write python code which can construct ROS nodes that can publish and subscribe to the neatos robots. These lessons are applicable to any ROS project so I'm excited to tackle future programming challenges. In addition, this project taught me more about coordinate systems in ros because I needed to subscribe to the robots odometry readings and apply appropriate transforms for my obstacle avoider.