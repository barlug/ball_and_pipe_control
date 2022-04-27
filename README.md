# Ball and Pipe Control System
MATLAB codes to open serial communication with a ball and pipe system. The system is made of a vertical cylinder with a ping pong ball controlled by a fan on the bottom and height measured by a time of flight sensor on top. The objective is to balance the ball at a target altitude. 

This Project was created by Kyle Naddeo & Dr. Rasool For Rowan University ECE09321 Systems & Controls class. 

Team Dr. Hanafi:
- Long "First Try" H. Chau
- Robert "PID-kid" Kerwin
- Jacob "Darwin Award Recipient" King
- Andrew "Ninja" Heyer

========================================
![Ball and Pipe Control System Image](https://github.com/barlug/ball_and_pipe_control/blob/main/ReadMe_Images/Ball_Pipe_system.jpg)

========================================

# PID Controller with Genetic Algorithm

In control theory, physical systems are represented by differential equations. In order to not have to solve differential equations, engineers take the representation of these physical systems from the time domain (where you solve differential equations) to the s-domain. In the s-domain, differentiation becomes a simple multiplication operation, and integration becomes a simple division operation. Going from one domain to another is called a transformation, and the Laplace transformation is used to go from the time domain to the s-domain. 

In the s-domain, rather than working with the "t" variable, which is a real number, we work with the "s" variable, which is a complex number. In the s-domain, the input-output relation of the system gives a complete description of the system. This input-output relationship is called the transfer function. In this project, we will work with a closed-loop system which utilizes a negative feedback from the output to the intput. This negative feedback allows the system to inherently adjust to error between the set point (i.e., the input) and the output. Depending on the system type, this negative feedback can either reduce the error to zero over time, or reduce it to a small constant value over time.

A system's behavior depend entirely on its poles and zeros. We completely determine a system's steady-state and transient behavior simply by knowing its poles and zeros. Thus, the poles and zeros of the transfer function, which give a complete desciption of the system, descibe completely the behavior of the system. In order to manipulate the behavior of systems, we must manipulate the poles and zeros of the system. There are three ways to move the poles and zeros of a system. 

The first way is to adjust gain. Adjusting gain moves a system's poles on set root locus, or a set path that the poles take when the gain is adjusted. Adjusting gain has no effect on the zeros of the system, however.

The second way is to redesign the system entirely to have the differential equations which, when put throught the Laplace transformation, give the necessary poles and zeros required for the desired system behavior. This process is necessarily tedious and expensive. The often more prefereable way is to design a controller, which is in itself a transfer function whose set point is the error betweeen the system's input and output, and whose output is the input to the system's transfer function. The controller is the third and most common method to manipulate a systems poles and zeros. Whereas in the first method, where adjusting gain affects the system's poles and zeros but they can only ever exist on a set root locus, controller allows the systems engineer to take the system's poles off the root locus entirely. Controllers do this by adding more zeros and poles to the system.

There are many different types of controller. In this project, we will be focusing on the PID (proportional-integral-derivative) controller, which is in itself made up of two smaller controllers: the PD (proportional-derivative), which addresses the system's transient behavior, and the PI (proportional-integral) controller, which addresses the system's steady-state error (i.e., the error between the set point and the system output as time goes to infinity). Just as like the system's transfer function, which has a gain that controls the location of the poles of the system on the root locus, the PD and PI each have their own gain. The combined gain (known as Kp, Kd, and Ki) determines where the additional poles and zeros and zeros will be added to the system. These additional poles and zeros added by the PID controller in turn affect the poles of the entire system, which in turn affect the steady state and transient response of the system. Finding the perfect values of the Kp, Kd, and Ki to have the desired system response is known as "tuning" the controller. There are many PID controller tuning methods, and the tuning method this project utilizes is called the Genetic Algorithm tuning method, which uses machine Deep Learning to to find the optimal values of Kp, Ki, and Kd.

# 1) Project Description
## What your application does
Our group's Ball & Pipe Controller utilizes a PID controller with a genetic Algorithm tuner to get the ball to reach any specified steady state location in the tube in the fastest time possible. 
## How do you send actions and recieve data in the correct format?
### Serial Interface:
  The SCFBA uses two serial links to communicate with MATLAB. Both utilize a TTL to USB serial translation interface based on the FTDI FT232RL USB to serial IC. Windows recognizes this USB device and will install the necessary drivers automatically.
  
 **The links are configured as follows:**
  - Connector: USB-A male
  - COM port settings:
    - Baud: 19200
    - Data bits: 8
    - Parity: None
    - Stop bits: 1
    - Flow control: None
  - ACK/NAK: None
  - All bytes are printable ASCII characters, case insensitive
  - There is no end of packet terminator

### PID Packets
![PID Packet Output](https://github.com/barlug/ball_and_pipe_control/blob/main/ReadMe_Images/PID_Packets_Screenshot.jpg)

# 2) The Theory Behind the Controller
## PID Controller 
Proportional Integral Derivative Control, or PID control, is a type of control system that uses three different types of responses: proportional, integral, and derivative. The responses are controlled with an input and a process variable. The input is the basic input of the system, like a step response. The process variable is the thing that is being changed by the system. A sensor measures a real-world thing, like pressure, temperature, or force, and then sends a signal to the system. The PID is a closed loop system. This means that it takes the error in the system, measured by a sensor or transducer, and uses it to correct itself after the first loop of the system. The system will cycle until it sees that it has reached the desired output. 

### Theory Background
The design of a PID controller consists of two active compensators: an active PD controller followed by an active PI controller. A block diagram of the general PID controller is shown below:

![PID Controller Image](https://github.com/barlug/ball_and_pipe_control/blob/main/ReadMe_Images/PID_Controller.jpg)

The purpose of the proportional is to have a large, immediate reaction on the output to bring the current value closer to the desired value (set point). As the error lessens, the influence of the proportional value lessens. Every time the controller performs the PID calculation, the new integral value is added to the integral total. The integral value will not have an immediate influence, but the longer it takes for the process value to reach the desired value, the more effect the integral will have. The purpose of the derivative is to predict where the process value is going, and bias the output in the opposite direction of the proportional and integral values. This is done to prevent the controller from overshooting the set point. 


### Challenges Faced & Future Edits 

## Genetic Algorithm 
A genetic algorithm models itself after the theory of evolution. It takes a program, makes slight changes to a number of copies, and sees which copy performs the best overall. Once the algorithm finds the best copy, it does the whole process all over again. The first program is called the initialization. Then, after the changes, or mutations, happen, the new ones are called individuals in a generation. The generation is the number of cycles that the process has gone through.

A genetic algorithm can find many solutions to one problem. Each time the program is ran, the mutations will rarely be the same as it was during the previous one. This means that it can find many solutions to one problem. Furthermore, as long as a genetic algorithm is running, it can always improve a solution. Although, as time goes one, improvements will make smaller changes as the algorithm gets more and more accurate. Downsides to the genetic algorithm are that it is not suitable for problems with a wide assortment of solutions. If there are too many viable solutions, then the fitness function may have trouble determining which is better. Then, when it comes to crossover, the combination of those two different solutions may be a terrible solution with very poor fitness. Another time when the genetic algorithm is not good is when the mutation probability is too large.

### Theory Background
In order to implement a genetic algorithm, there are certain functions that are needed. These functions include, the initialization, measure fitness, selection, crossover, and mutation. To begin, the initialization function is the function that creates the initial population of the first generation. Next, the fitness of each individual’s fitness is measured against the given solution. Sometimes the fitness function may take too long to evaluate each of the individuals, so it will look at how close it is. Basically, it will look at the individual, and determine if it is good or not. Next, the selection function takes the fitness values given to each of the individuals and compares them. The individuals that are selected become the parents of the next generation. The first step of creating the next generation is crossover. In one point crossover, a point in the array is chosen, and the values of each parent are swapped with another. The other type of crossover is called multi-point crossover. Multi-point crossover is when more than one crossover point is selected. Then, the values in between or outside of those points are swapped with one another. Finally, the mutation function applies random changes to the offspring of the new parents. These mutations are based off of a probability factor, where the larger the factor, the more mutations occur. Finally, the “offspring” are initialized again with the characteristics of the best individuals from the prior generation. When the fitness function determines that the fitness of an individual is close enough to the desired outcome, the cycle will stop and that individual will be the final solution. 

![GA_Diagram](https://github.com/barlug/ball_and_pipe_control/blob/main/ReadMe_Images/Genetic_Algorithm_Diagram.jpg)

### Challenges Faced & Future Edits
Creating the Genetic Algorithm was the hardest part of this process.  Luckily, two professors from Cambridge, Steven L. Brunton, and J. Nathan Kutz, had already pioneered the use of Genetic Algorithms in MATLAB.  Using their example code as a guide, the Genetic Algorithm implementation was created.  The other challenge of the Genetic Algorithm was developing a proper fitness function for the individuals of each generation to be graded.  We got over this challenge by implementing a Linear Quadratic Regulator (LQR) cost function that prioritized a fast rise time and a low steady-state error.

The implementation into the PID controller was simple as the Genetic Algorithm only provided the gain values. When implementing the Genetic Algorithm with the PID controller the main challenge that was faced was properly identifying an accurate enough transfer function that modeled the solution to a high enough accuracy that the initial Genetic Algorithm gain values would be accurate.  With the system being a complex set of ordinary differential equations with many factors affecting the ball like gravity, rotation, wind speed, friction with the sides of the pipe, etc. it is hard to simulate it with 100% accuracy.  This made it so multiple iterations of the algorithm would have to be run as more and more of these factors are implemented into the original transfer function.  For future work on the Genetic Algorithm, not much would change.  The Genetic Algorithm has been implemented in full to achieve the best Kp, Kd, and Ki, values based on the simulation’s transfer function and the fitness function.  Future tweaking of the systems transfer function and the fitness function may be done to get more refined and accurate gain values for the PID controller.


# 3) WALK THROUGH HOW TO USE CODE
## How to Install & Run the Project: What are the required dependancies? (MATLAB, Toolkits, Ball & pipe system, connection testing setup using Putty)
The Ball & Pipe system is coded using [MATLAB](https://matlab.mathworks.com), which can be downloaded off their website.  
he system uses the following MATLAB Toolkits: 
- Symbolic Math Toolbox
- Control System Toolbox
- Optimization Toolbox
- Global Optimization Toolbox

All four of these toolboxes can be added onto MATLAB for free using the Add-On Explorer found on the HOME tab of MATLAB.


## How to Use the Project 
Provide instructions & Examples (finding from our testing). Include Screenshots 
Discuss structure & design principles used in the project   

### PID Controller Files
#### real_world.m (Top Level)
The “real_world” function is the top level function, as well as the function that implements the PID controller into the system. The first thing it does is connect to the serial port. Then, it gives the system a kick in order to get the ball into the air, so the pwm is set to 3000. Next, it determines if it needs to go up or down by comparing the current height to the target set by the user. Using this error, it calculates each part of the PID. The gain for each part was calculated using the genetic algorithm. The proportional response is calculated by multiplying the most recent error value by the gain. The integral response is calculated by multiplying the gain by the sum of the error time the sampling rate. Finally, the derivative response if found by multiplying the derivative gain by the difference in the most recent and second most recent error values, divided by the sampling rate. To find the pwm, the responses are summed together. 

The target variable is the desired location where the ball will settle. The sample rate is the time in between the execution of each control signal, and it is used with the pause() function to wait to send the next signal. Kp, Ki, and Kd are the gains of each of their respective responses. These values can be changed to change the response of the system. 

#### set_pwm.m
The ”set_pwm” function is responsible for sending the calculated pwm to the ball and pipe system. The first thing it does is make sure the calculated pwm value is within the boundaries of the project. It does this by using an if statement. If the value is greater than 4095, then it sets it to 4095, and if the value is less than 0, then it sets it to zero. After this, the function converts the double variable back to a string. Finally, it sends the value to the device using the serial port.

#### ir2y.m
The “ir2y” function of the program is responsible for converting the sensor reading to a y value that the program can use. In order to get the ball’s position in the tube, the program uses the equation: pipe_percentage = 1 - (ir-ir_top)/(ir_bottom-ir_top). In this equation, ir is the reading from the sensor, ir_top is the value of the sensor when the ball is at the top of the tube, and ir_bottom is the value of the sensor when the ball is at the bottom of the tube. Since the sensor is at the top of the tube, if the pipe_percentage was 25%, that means that the ball was 25% away from the top of the tube. In order to get the value with reference from the bottom, the percentage was subtracted from one. Using the previous example, if the reading was 25%, subtracting it from one would yield a value of 75%.  Finally, to find a value for y, the pipe_percentage is multiplied by the actual height of the tube, 0.9144 meters. 

#### read_data.m
The ”read_data” function is used to read the data coming from the ball_and_pipe itself. The data packet from the assembly contains the ir value from the sensor, used in the “ir2y” function, and the settings put forth by the knobs. These include, target, deadpan, and pwm. The function uses the str2double() and extractBetween() MATLAB functions. Due to the data packet structure, each of these values need to be extracted from the packet. This is done using the extractBetween() function. In this function, the first element is the place the data is coming from, and the next two are the start position and end position. Then, str2double() converts the string extracted from the data packet to a double variable that can be used to find the pipe percentage. 

### Genetic Algorithm Files
#### geneticAlgorithm.m (Get Tuned Variables)
The genetic algorithm used is based off of the one developed by Steven L. and Brunton and J. Nathan Kutz of Cambridge. It is used to find the tuned gain of the responses for the PID controller. In order to run this code and use the genetic algorithm function, the ‘Global Optimization’ and ‘Optimization Toolbox’ must be added onto MATLAB.  These are the toolboxes that are responsible for computing the genetic algorithm using the ‘ga’ function. First, the known variables are declared, like the mass and volume of the ping pong ball, gravitational acceleration,and the density of air. Next, the coefficients of the transfer function are declared. C2 is the characteristic equation of the ball and pipe system. C3 is the coefficient that relates air speed to the pwm signal. These variables are then plugged into the transfer function: G=c3*c2/(s*(s+c2)). 
Next, the parameters for the genetic algorithm are set by the user. PopSize is the number of individuals that are produced during a generation, and MaxGenerations is the number of generations that will be produced. Options sets the parameters for the genetic algorithms using the previous declared variables. Finally, the genetic algorithm is executed using the command: [x,fval]=ga(@(K)pidtest(G,dt,K),3,-eye(3),zeros(3,1),[],[],[],[],[],options);.
X is the values for the gain of the responses of the PID.
fval is the fitness value for each set of values for x. 
Ga() is the command for the genetic algorithm.
Pidtest() is the function call for the cost function to evaluate the fitness of each individual. 
3 is the number of variables that are being found through the genetic algorithm. 

#### pidtest.m
The “pidtest” function is the cost or fitness function for the genetic algorithm. It first puts the pid controller in series with the control function: K=parms(1)+parms(2)/s+parms(3)*s/(1+.001*s).parsm(1) is the proportional response, parms(2) is the integral response, and parms(3) is the derivative response. The derivative has an additional part in order to drive it to zero at very high frequencies and prevent any instability. Then, it creates a feedback loop around the plants using the feedback() MATLAB function. In order to test each individual, a LQR, or a linear quadratic regulator, to minimize the error in the system while finding the fastest rise time that the system can possibly have. “drawnow” is used to update the figure for every iteration of the genetic algorithm. 

# 4) Credits & References
Kyle Naddeo for structure of project

Steve Brunton for Genetic Algorithm Code: [Machine Learning Control: Tuning a PID Controller with Genetic Algorithms](https://www.youtube.com/watch?v=S5C_z1nVaSg)

Mario Leone, Karl Dyer, Michelle Frolio for Systems & Control Floating Ball Apparatus (SCFBA)

Works Cited
Control Station. (2021, November 15). How does the derivative term affect PID controller performance? How Does the Derivative Affect PID Performance? Retrieved February 23, 2022, from https://controlstation.com/blog/derivative-affect-pid-controller-performance/#:~:text=Derivative%20is%20the%20third%20term%20within%20the%20PID.&text=Seen%20in%20the%20context%20of,seeks%20to%20correct%20for%20error. 

Jayachitra, A., & Vinodha, R. (2014, December 23). Genetic algorithm based PID controller tuning approach for continuous stirred tank reactor. Advances in Artificial Intelligence. Retrieved February 23, 2022, from https://www.hindawi.com/journals/aai/2014/791230/ 

Mallawaarachchi, V. (2017, November 10). How to define a fitness function in a genetic algorithm? Medium. Retrieved February 23, 2022, from https://towardsdatascience.com/how-to-define-a-fitness-function-in-a-genetic-algorithm-be572b9ea3b4 

Neelarghya. (2021, July 26). Reinforcement learning vs genetic algorithm - AI for simulations. Medium. Retrieved February 23, 2022, from https://medium.com/xrpractices/reinforcement-learning-vs-genetic-algorithm-ai-for-simulations-f1f484969c56 

Ni. (2020, March 7). PID theory explained. PID Theory Explained. Retrieved February 23, 2022, from https://www.ni.com/en-us/innovations/white-papers/06/pid-theory-explained.html 

TutorialsPoint. (n.d.). Genetic Algorithms. Genetic algorithms tutorial. Retrieved February 23, 2022, from https://www.tutorialspoint.com/genetic_algorithms/index.htm 

# 5) Contributions
This project was created for Systems & Controls, ECE09321, Rowan University, Spring 2022 Semester. 
This project will not be updated further, and so Forks are welcomed to further refine the code. 
