# Ball and Pipe Control System
MATLAB codes to open serial communication with a ball and pipe system. The system is made of a vertical cylinder with a ping pong ball controlled by a fan on the bottom and height measured by a time of flight sensor on top. The objective is to balance the ball at a target altitude. 

This Project was created by Kyle Naddeo & Dr. Rasool For Rowan University ECE09321 Systems & Controls class. 

Team Dr. Hanafi:
- Long "First Try" H. Chau
- Robert "PID-kid" Kerwin
- Jacob "Darwin Award Recipient" King
- Andrew "Ninja" Heyer

========================================

# PID Controller with Genetic Algorithm

In control theory, physical systems are represented by differential equations. In order to not have to solve differential equations, engineers take the representation of these physical systems from the time domain (where you solve differential equations) to the s-domain. In the s-domain, differentiation becomes a simple multiplication operation, and integration becomes a simple division operation. Going from one domain to another is called a transformation, and the Laplace transformation is used to go from the time domain to the s-domain. 

In the s-domain, rather than working with the "t" variable, which is a real number, we work with the "s" variable, which is a complex number. In the s-domain, the input-output relation of the system gives a complete description of the system. This input-output relationship is called the transfer function. In this project, we will work with a closed-loop system which utilizes a negative feedback from the output to the intput. This negative feedback allows the system to inherently adjust to error between the set point (i.e., the input) and the output. Depending on the system type, this negative feedback can either reduce the error to zero over time, or reduce it to a small constant value over time.

A system's behavior depend entirely on its poles and zeros. We completely determine a system's steady-state and transient behavior simply by knowing its poles and zeros. Thus, the poles and zeros of the transfer function, which give a complete desciption of the system, descibe completely the behavior of the system. In order to manipulate the behavior of systems, we must manipulate the poles and zeros of the system. There are three ways to move the poles and zeros of a system. 

The first way is to adjust gain. Adjusting gain moves a system's poles on set root locus, or a set path that the poles take when the gain is adjusted. Adjusting gain has no effect on the zeros of the system, however.

The second way is to redesign the system entirely to have the differential equations which, when put throught the Laplace transformation, give the necessary poles and zeros required for the desired system behavior. This process is necessarily tedious and expensive. The often more prefereable way is to design a controller, which is in itself a transfer function whose set point is the error betweeen the system's input and output, and whose output is the input to the system's transfer function. The controller is the third and most common method to manipulate a systems poles and zeros. Whereas in the first method, where adjusting gain affects the system's poles and zeros but they can only ever exist on a set root locus, controller allows the systems engineer to take the system's poles off the root locus entirely. Controllers do this by adding more zeros and poles to the system.

There are many different types of controller. In this project, we will be focusing on the PID (proportional-integral-derivative) controller, which is in itself made up of two smaller controllers: the PD (proportional-derivative), which addresses the system's transient behavior, and the PI (proportional-integral) controller, which addresses the system's steady-state error (i.e., the error between the set point and the system output as time goes to infinity). Just as like the system's transfer function, which has a gain that controls the location of the poles of the system on the root locus, the PD and PI each have their own gain. The combined gain (known as Kp, Kd, and Ki) determines where the additional poles and zeros and zeros will be added to the system. These additional poles and zeros added by the PID controller in turn affect the poles of the entire system, which in turn affect the steady state and transient response of the system. Finding the perfect values of the Kp, Kd, and Ki to have the desired system response is known as "tuning" the controller. There are many PID controller tuning methods, and the tuning method this project utilizes is called the Genetic Algorithm tuning method, which uses machine Deep Learning to to find the optimal values of Kp, Ki, and Kd.

# 1) Project Description (EXPLAIN SCOPE OF PROJECT)
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

# 2) (DISCUSS THEORY BEHIND METHODS)
## PID Controller 
Proportional Integral Derivative Control, or PID control, is a type of control system that uses three different types of responses: proportional, integral, and derivative. The responses are controlled with an input and a process variable. The input is the basic input of the system, like a step response. The process variable is the thing that is being changed by the system. A sensor measures a real-world thing, like pressure, temperature, or force, and then sends a signal to the system. The PID is a closed loop system. This means that it takes the error in the system, measured by a sensor or transducer, and uses it to correct itself after the first loop of the system. The system will cycle until it sees that it has reached the desired output. 
The proportional response depends on the difference between the input, or set point, and the process variable.  When using the proportional response, the difference between the input and the process value is called the proportional gain. The gain is used to increase the speed of the system, but if it gets too high then the system will begin to fail or oscillate. 
The integral response is used to push the error of the steady state response to zero (Ni). It sums the amount of error that has happened while the system is running and adjusts the system so that the effect it has on the steady state is minimal. The derivative response is another way of adjusting the error so that it does not have a large effect on the steady state of the system (Control Station).  The derivative response is for the rate of change of the error of the system.

### Theory Background
### Challenges Faced & Future Edits 

## Genetic Algorithm 
A genetic algorithm models itself after the theory of evolution. It takes a program, makes slight changes to a number of copies, and sees which copy performs the best overall. Once the algorithm finds the best copy, it does the whole process all over again. The first program is called the initialization. Then, after the changes, or mutations, happen, the new ones are called individuals in a generation. The generation is the number of cycles that the process has gone through.
### Theory Background
### Challenges Faced & Future Edits


# 3) WALK THROUGH HOW TO USE CODE
## How to Install & Run the Project: What are the required dependancies? (MATLAB, Toolkits, Ball & pipe system, connection testing setup using Putty)
The Ball & Pipe system is coded using [MATLAB](https://matlab.mathworks.com), which can be downloaded off their website.  
The system uses the following MATLAB Toolkits: 

## How to Use the Project 
Provide instructions & Examples (finding from our testing). Include Screenshots 
Discuss structure & design principles used in the project   

### PID Controller Files
#### real_world.m (Top Level)

#### set_pwm.m

#### ir2y.m

#### read_data.m

### Genetic Algorithm Files
#### geneticAlgorithm.m (Get Tuned Variables)

#### pidtest.m

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
