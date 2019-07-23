# 4549A
VEX Robotics Code For the Team: 4549A

#Code Explanation

Position Drive:

1. https://github.com/Sajantoor/4549A/blob/master/src/drive.cpp#L1025

This line of code, works by calculating the angle of the line that we are following relative to the y-axis (the return value is in radians)

2. https://github.com/Sajantoor/4549A/blob/master/src/drive.cpp#L1026

This line of code calculates the direction between the start and end points of the path (the return value is in radians)

3. https://github.com/Sajantoor/4549A/blob/master/src/drive.cpp#L1035

This line of code calcultes the difference between our current position and our ending poisition or the target in the x-axis (The return value is in inches)

4. https://github.com/Sajantoor/4549A/blob/master/src/drive.cpp#L1045

This line of code calculates the difference between our current orientation and our "target orientation" (the return value is in radians)

5. https://github.com/Sajantoor/4549A/blob/master/src/drive.cpp#L1046

This line of code calculates how far off we will be if we continue on our current trajectory (The return value is in inches)

6. https://github.com/Sajantoor/4549A/blob/master/src/drive.cpp#L1047

Calculation of the direction to the target from our current position

7. https://github.com/Sajantoor/4549A/blob/master/src/drive.cpp#L1048

If the max speed is set to a negative number (less then 0), add 180 degrees or pi (radians) to correctA

8. https://github.com/Sajantoor/4549A/blob/master/src/drive.cpp#L1050

Calculation of the correction value but the correction value is only calculated if the bot is going to end up more off than we're allowed to be if our current trajectory continues

9. https://github.com/Sajantoor/4549A/blob/master/src/drive.cpp#L1064

The calculation of the final power (-127/40 is basically kp (127 is the most amount of power and it means that until we are 40 inches away from the target give full power))

10. https://github.com/Sajantoor/4549A/blob/master/src/drive.cpp#L1077

The cases start here, they are basically to change final power depended on the sgn of correction, the way sgn works is if the value of correction is positive it will return a value of 1, if the value of correction is negative it will return a value of -1 and if it is 0 then it will return a value of 0

11. https://github.com/Sajantoor/4549A/blob/master/src/drive.cpp#L1079

If the sgn returns a value of 0 then that means we are going straight and theres no need to correct

12. https://github.com/Sajantoor/4549A/blob/master/src/drive.cpp#L1083

If the sgn returns a value of 1 then that means we are curving to the left and we have to slow down the right side

13. https://github.com/Sajantoor/4549A/blob/master/src/drive.cpp#L1087

If the sgn returns a value of -1 then that means we are curving to the right and we have to slow down the left side
