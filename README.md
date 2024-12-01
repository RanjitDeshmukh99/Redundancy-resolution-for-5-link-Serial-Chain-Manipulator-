# Redundancy-resolution-for-5-link-Serial-Chain-Manipulator

**Motivation:** <br />
Developed 5-link Serial Chain Manipulator to track desired end effector position trajectories using open-loop and closed-loop control (redundancy-resolution) .
This project focuses on developing and implementing control strategies for a 5-link robotic serial chain arm manipulator to accurately track desired trajectories within a specified time. It aims to design effective open-loop and closed-loop control algorithms, exploring the impact of link geometry, orientation, and control techniques on robotic manipulators' performance.

**Objective:** <br />
To achieve a precise tracking of desired trajectory path by the end effector of a given 5-link serial chain robotic arm.

**Methodology:** <br />
Formulating kinematic equations for the 5-link serial chain manipulator 
Implementing different redundancy-resolution techniques for optimal manipulator control to observe and study the behavior of the manipulator.
Performing simulations on MATLAB-CoppeliaSim connected environment
Developing a GUI app for studying the effects of varying different parameters 

**Desired Trajectories:** <br />

<ins>Ellipse:</ins> <br />
semi-major axis: 3 units<br />
semi-minor axis: 0.5 units<br />
center: (1,1)<br />
tilted 15° w.r.t. horizontal<br />

![image](https://github.com/user-attachments/assets/fae6aaf4-f07c-4666-a622-3b7b984db284)

<ins>Square:</ins> <br />
side Length: 4.5 units<br />
center: (−0.5,1)<br />

![image](https://github.com/user-attachments/assets/fcdf4702-3411-4932-9a6d-75091dc71749)
<br />


## **Phase 1 : Formulation of 5-link Serial-Chain manipulator for kinematic simulation**
![image](https://github.com/user-attachments/assets/8ebeabdb-7ccb-4dab-b1d6-591260e09fe3)

L1=2
L2=2.5
L3=1
L4=1.5
L5=1<br />

resolving along x and y components<br />
$$𝑥=𝐿_1 𝑐𝑜𝑠𝜃_1+ 𝐿_2 𝑐𝑜𝑠𝜃_2+ 𝐿_3 𝑐𝑜𝑠𝜃_3+ 𝐿_4 𝑐𝑜𝑠𝜃_4+𝐿_5 𝑐𝑜𝑠𝜃_5$$<br />
$$𝑦 =𝐿_1 𝑠𝑖𝑛𝜃_1+ 𝐿_2 𝑠𝑖𝑛𝜃_2+ 𝐿_3 𝑠𝑖𝑛𝜃_3+ 𝐿_4 𝑠𝑖𝑛𝜃_4+𝐿_5 𝑠𝑖𝑛𝜃_5$$<br />

$$𝜃_1,𝜃_2,𝜃_3,𝜃_4,𝜃_5$$ are absolute angles

To study the effects of different open-loop redundancy resolution methods to problem at hand, 
We first consider the aforementioned equations and differentiate wrt time to get velocities,<br />
$$𝑥 ̇=−𝐿_1 𝑠𝑖𝑛𝜃_1 𝜃_1 ̇− 𝐿_2 𝑠𝑖𝑛𝜃_2 𝜃_2 ̇− 𝐿_3 𝑠𝑖𝑛𝜃_3 𝜃_3 ̇− 𝐿_4 𝑠𝑖𝑛𝜃_4 𝜃_4 ̇−𝐿_5 𝑠𝑖𝑛𝜃_5 𝜃_5 ̇$$ <br />
$$𝑦 ̇=  𝐿_1 𝑐𝑜𝑠𝜃_1 𝜃_1 ̇+ 𝐿_2 𝑐𝑜𝑠𝜃_2 𝜃_2 ̇+ 𝐿_3 𝑐𝑜𝑠𝜃_3 𝜃_3 ̇+ 𝐿_4 𝑐𝑜𝑠𝜃_4 𝜃_4 ̇+𝐿_5 𝑐𝑜𝑠𝜃_5 𝜃_5 ̇$$<br />

Putting everything in a matrix we get
![image](https://github.com/user-attachments/assets/85750f66-8099-4bfa-9a99-fd1e40aeb394)

We will use a compact notation $$𝑋 ̇=𝐽(𝜃 ) ̇$$<br />
where 𝐽 is the Jacobian<br />
<br />

## **Phase 2 : Redundancy resolution**<br />
<ins>i) Using the traditional pseudo-inverse solution</ins><br />
 $$𝑋 ̇=𝐽(𝜃 ) ̇$$<br />
 $$𝜃 ̇=𝐽^{−1} 𝑋 ̇$$<br />
 For inverse to exist J has to be a square matrix<br />
J has to be non-singular matrix (det⁡(𝐽)≠0)<br />
Hence, we use pseudo-inverse:<br />
$$𝜃 ̇=𝐽^+ 𝑋 ̇$$<br />
Where $$𝐽^+$$ is the pseudo-inverse of 𝐽<br />
We simulate this using ode45() solver in matlab to obtain the open-loop solution 
![image](https://github.com/user-attachments/assets/d36a8ddb-6640-4cf6-879f-fd5531910f22)
![image](https://github.com/user-attachments/assets/0e974964-6bd9-4255-886c-61c7677e18d9)

<ins>ii) Adding auxiliary constraints on the absolute joint angle space variables of the form to resolve redundancy:</ins><br />
For this problem, we use the least square solution:<br />
$$𝜃 ̇=𝐽^+ 𝑋 ̇+(𝐸−𝐽^+ 𝐽)𝑧$$<br />
And we formulate z based on the auxillary constraints below:<br />
a. $$𝜃_1 ̇=1,  𝜃_2 ̇=0,  𝜃_3 ̇=0$$ <br />
![image](https://github.com/user-attachments/assets/2bf2644d-7fd8-4c49-8b3f-ef161f248390)
![image](https://github.com/user-attachments/assets/7bfd38b9-0356-4291-a75e-de82c216bf9a)


b. $$𝜃_2 ̇+𝜃_2 ̇=0, 𝜃_3 ̇=0$$<br />
![image](https://github.com/user-attachments/assets/4b4f03c8-a4e0-40ea-bea6-417ff21b5f86)
![image](https://github.com/user-attachments/assets/5823f262-e3ce-4eca-b723-31f38f5169ee)


<ins>iii) Minimization of an artificial potential described on the joint space as a secondary manipulation criterion to the traditional pseudoinverse solution:</ins><br />
$$𝑉=(𝜃_1−𝜋/6  )^2+0.25(𝜃_2−𝜋/2  )^2+0.66(𝜃_3−𝜋/3  )^2$$<br />
$$𝜃 ̇=𝐽^+ 𝑋 ̇+(𝐸−𝐽^+ 𝐽)𝑧$$<br />
$$𝑧=−𝜕𝑉/𝜕𝜃$$<br />
$$𝑧=[(2(𝜃_1−𝜋/6)  2×0.25(𝜃_2−𝜋/2)  2×0.66(𝜃_3−𝜋/3  )]^𝑇$$<br />
![image](https://github.com/user-attachments/assets/e1fa7e7d-cef3-42b4-8a48-04f830bf98eb)
![image](https://github.com/user-attachments/assets/e374da01-2fe2-400f-b804-c09d3b86fe22)
<br />


## **Phase 3 : Joint-space control and Task-Space Control** <br />
The desired manipulation rates can be achieved by the manipulator using an appropriately modified closed-loop variant of Resolved motion-rate control. Hence, we will look at 2 types of controllers that allows the actual manipulation rates $$𝑋 ̇ $$ to track the desired manipulation rates $$𝑋^d ̇ $$ <br /> 
 
The previous equations were for an open loop formulation. <br />
Not always the initial point will lie on the desired trajectory !!<br />
Now we add a feedback term which takes the error between<br />
the desired output and the actual output and multiplies it with the gain K to make a closed loop controller.

<ins>i) Design of joint-space controller</ins><br />
$$𝜃 ̇=𝐽^+ 𝑋 ̇$$<br />
$$𝜃 ̇=𝐽^+ 𝑋 ̇+𝐾[𝜃^𝑑−𝜃]$$<br />
Where K is a diagonal matrix with each diagonal element<br />
(error dynamics time-constant 𝜏=3 𝑠𝑒𝑐)<br />
$$𝑘_{𝑑𝑖𝑎𝑔}=1/𝜏=1/3$$<br />
![image](https://github.com/user-attachments/assets/6a426cc5-b5ab-448e-8268-d6e428476b82)

<ins>ii) Design of closed-loop task-space controller</ins><br />
In this case we take the error between desired trajectory co-ordinates and actual trajectory co-ordinates in task space to multiply with controller gains to provide as a feedback. <br />
$$𝜃 ̇=𝐽^+ 𝑋 ̇+𝐾[𝑋^𝑑−𝑋]$$<br />
![image](https://github.com/user-attachments/assets/05d34c79-26e0-49c8-aeab-e53e58877b06)

Where K is a diagonal matrix<br />
$$𝐾=[−5 0 ; 0−10]$$ <br />
(Pole of error dynamics along X is - 5 and Y is -10)<br />
<br />


## **Phase 4 : MATLAB <--> CoppeliaSim**<br />
We model the given 5-link Serial Chain Manipulator in CoppeliaSim and connect it to MATLAB<br />
![image](https://github.com/user-attachments/assets/153d4e38-7e6d-4b25-b3a0-bb8bc6564332)
![image](https://github.com/user-attachments/assets/c2bdbb04-8203-4f7f-bdd1-53bcb2211b09)

Simulating open-loop 5-link Serial Chain Manipulator in CoppeliaSim-MATLAB connected environment<br />
![image](https://github.com/user-attachments/assets/911db975-fb40-43ed-ae19-2d1143f299d6)
![image](https://github.com/user-attachments/assets/bbcc393d-d5ca-485b-b091-5ab57b1a15ec)

Simulating closed-loop 5-link Serial Chain Manipulator in CoppeliaSim-MATLAB connected environment<br />
![image](https://github.com/user-attachments/assets/0ebab7d8-5ecc-4ad4-80c3-6ac91bb1c0bb)
![image](https://github.com/user-attachments/assets/8ec3bee7-2db1-47bc-9d90-791872dbcc55)
<br />


## **Phase 5 : Graphical User Interface**<br />
Developing a Graphical User Interface to allow us vary the various parameters of interest and study the effects<br />
![image](https://github.com/user-attachments/assets/009df49b-5ecb-4f55-a74f-11489d49e0b1)
And once we hit the plot button we can see the animation of the manipulator tracing the desired ellipse<br />
![image](https://github.com/user-attachments/assets/d0d24624-594b-4e66-ba8b-1d42f581ae23)
![image](https://github.com/user-attachments/assets/82b94b5f-3548-4457-b24e-df15f87e737a)


*Animation clips and MATLAB codes are provided*







 











