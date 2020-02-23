# 4549A's Programming And Algorithm Design Proccess
### Table of Contents
* [Problem Solving](#Problem-Solving)
* [Code Style](#Code-Style)
* [Algorithms](#Algorithms)
* [Variables](#Variables)

## Problem Solving
> Problem solving is what programming is all about. The goal of programming can be summed up as solving problems using the simplest and best solution possible. In VEX the problem could be as simple as moving the motor with a button press or as complex as tracking and following cubes using the vision sensor in the autonomous period. 

There are several steps in solving problems: 
* Understand the Problem
* Deriving a Solution
* Pseudo Code
* Writing Code and Testing
* Clean Code and Optimize

### Understanding the Problem 
> It's impossible to tackle a problem without understanding it fully first. The programmer must understand the problem fully and ask important questions at this stage. If the problem is too big, it must be broken down to several smaller problems which can be tackled independently, drive functions are an example of this (seperate into turning, driving, etc.). It's important to write down the problem and all known and unknown information at this stage as it will help the proccess of creating a solution. 

### Deriving a Solution
> The solution of the problem doesn't have to be code at this stage. It could be plain English or math, which would be translated into code later in the proccess. For example for an autonomous stack, a viable solution would be using a potentiometer and a PID loop. The programmer doesn't need to know how to write a PID loop or write a PID loop at this stage. It's important to write down all possible solution you come up with. Solutions can come in many forms from diagrams to flow charts or text. 

### Pseudo Code 
> Once you come up with a viable solution you are happy with it's time to write pseudo code, this allows for a transition between the english solution and the final coded solution. Pseudo code is an algorithm designed using words. By looking at your solution generate some pseudo code. For example for an autonomous stack: `while the potentiometer value is smaller than the target, run the motors. If potentiometer value is bigger than the target, stop the motors`. 

### Writing Code And Testing
> Using the pseudo code you've created it's time to finally write the code. The pseudo code should allow you to know exactly what to write and or what you need to write. If you don't know how to do something check documentation at this stage, for example if you don't know how to get the potentiometer value check the PROS documentation. Once you are done, test the code. If you aren't getting expected results it's time for debugging. Debugging strategies with VEX would be printing values or reading over your code to make sure everything makes sense. 

### Clean Code and Optimize
> At this stage it's important to clean up your code and remove anything that's unnecessary, such as prints, unused codes and commented out code. It's also time to optimize your algorithm for maxmimum performance, removing bugs, working on edge cases, and writing a cleaner solution. After that you should add comments so it's easy to understand the code and read it later.  

![An Example](https://github.com/Sajantoor/4549A/blob/documentation/docs/Vision%20Sensor%20Problem%20Solving.jpg)
> Example: solving the problem of cube tracking using the vision sensor using this problem solving process.

## Code Style
> The style of your code is extremely important as it allows for improved readability which can help in the programming process. Code should be properly indented and commented. General rule of thumb is there should be and indent every open brace. There should also be spacing between `if`s and between brackets and commas. Here's an example of good code style: 

```cpp
 // intake on triggers
 if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !intakeUsed) {
     loader_left.move_voltage(12000);
     loader_right.move_voltage(12000);
 } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !intakeUsed) {
     loader_left.move_velocity(-12000);
     loader_right.move_velocity(-12000);
 } else if (!intakeUsed) {
     loader_left.move(0);
     loader_right.move(0);
 }

```
> Here's what your code **SHOULDN'T** look like.
```cpp
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !intakeUsed)     {
	loader_left.move_voltage(12000);
			loader_right.move_voltage(12000);
		         } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !intakeUsed) {
		loader_left.move_velocity(-12000);
	     		loader_right.move_velocity(-12000);
	   	}else if(!intakeUsed) 
{
loader_left.move(0);
loader_right.move(0);
}
```

## Algorithms
> Algorithms are supposed to be as simple as possible and designed to be easily changed and manipulated if needed. The programmers need to be aware of the hardware and memory constraints  of the V5 Brain when designing an algorithm. An example of a bad algorithm would be multiple for loops within themselves, this would have a large [time complexity](https://en.wikipedia.org/wiki/Time_complexity) pushing the hardware closer to its limits. Another example is an algorithm using an infinite array when only 4 values need to be stored at a given time, this would be memory intensive. 

## Variables
> Variables are an important part of programming. Variables are used when some data has to be stored and manipulated or when something is a constant, such an unchanging speed and is used in multiple places. 

### Variable Naming
> Variables are named using camel case (camelCase). Booleans are to be named as verbs to make it clear what it is. Constants are to be named using all capital letters seperated by underscores.
