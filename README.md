# T-Obstacles

T-Obstacles is a project I developed for motion planning through moving obstacles. Its name comes from the motion of the obstacles; their position is dependent on time, or *t*.

This is implemented using a Rapidly-Exploring Random Tree (RRT), which moves randomly, but generally in the direction of the goal. As well as the traditional RRT which has random positions associated with each new node, this one also has random times associated with each node, as the obstacles are moving.

For a further high-level description of the finished project, see [this presentation](https://docs.google.com/presentation/d/1w7CK_WQaUkzIgz8D2IaMInBpTcrzGRvAR6ilEVV8K3g/edit?usp=sharing "this presentation").

## Technical Details

### Maths

This assumes a background in control theory, linear algebra, and calculus. If you're more interested in the code, you should probably read the [Implementation section](#implementation-section "Implementation section") instead.

Throughout this section, **bold** is used to indicate vectors and *italics* are used to indicate scalars. Due to the limitations of markdown, vector components will be written using horizontal matrices as opposed to the customary vertical ones.

#### Obstacles

It is assumed that we are working in c-space, where the axis variables represent the location of the robot. The robot is assumed to be a rigidbody and is constrained to the xy plane; hence the axis variables are *x* and *y*.

Each obstacle&#39;s velocity is constant. We are provided the initial location and rotation of each obstacle as **t<sub>0</sub>** = [*x<sub>0</sub>*, *y<sub>0</sub>*, *&theta;<sub>0</sub>*] and its velocities as **v** = [*v<sub>x</sub>*, *v<sub>y</sub>*, *v<sub>&theta;</sub>*]. By integrating these parameters, we can derive the obstacle&#39;s location at any time as **r(t)**. (For example, *r(t)<sub>x</sub>* is &int;<sup>t</sup><sub>o</sub>(*v<sub>x</sub>* dt) = *v<sub>x</sub>*t + *x<sub>0</sub>*.)

#### RRT Random Numbers

The current heuristic implemented with the RRT uses a normal distribution of standard deviation 0.2, which is weighted towards the goal.

In addition, branches are not created off of each node every loop cycle (as this grows exponentially and soon takes over the screen.) There is instead a random probability of generating one: a branch is generated when:

*n* &le; 1 - tanh(*x* / *a*)

where *n* is a random number in [0, 1), *x* is the number of branches already created, and *a* is a tunable variable. This function is demonstrated on [this Desmos graph](https://www.desmos.com/calculator/zw6bjoeph2 "this Desmos graph"). [This other graph](https://www.desmos.com/calculator/2iovtlu2fn "This other graph") demonstrates the average number of nodes created each loop cycle.

#### Collisions

We begin with a polygon *P* with *n* sides, where the side between points *a* and *b* is indicated as *P<sub>a, b</sub>*. We also begin with a line segment *L* with points *L1* and *L2*. If any side intersects the line segment, then clearly the line segment intersects the polygon. Thus, we consider each side *S* of the polygon in turn:

*S<sub>i</sub>* &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;where *i* = *P<sub>0, 1</sub>, P<sub>1, 2</sub>... P<sub>n-2, n-1</sub>, P<sub>n-1, 0</sub>*

Note how we loop back to the first point; otherwise we would be excluding a side.

As the sides are also line segments, we can also denote their endpoints as *S1<sub>i</sub>* and *S2<sub>i</sub>* .

The lines intersect if and only if

O(*L1*, *L2*, *S1<sub>i</sub>*) &ne; O(*L1*, *L2*, *S2<sub>i</sub>*) and
O(*S1<sub>i</sub>*, *S2<sub>i</sub>*, *L1*) &ne; O(*S1<sub>i</sub>*, *S2<sub>i</sub>*, *L2*)

where O represents the orientation function and is defined as:

O(**1**, **2**, **3**) &equiv; (*2<sub>y</sub>* - *1<sub>y</sub>*)(*3<sub>x</sub>* - *2<sub>x</sub>*) - (*3<sub>y</sub>* - *2<sub>y</sub>*)(*2<sub>x</sub>* - *1<sub>x</sub>*) > 0

In other words, if the orientation of points **1**, **2**, and **3** is clockwise, then O(**1**, **2**, **3**) is true. If their orientation is counterclockwise, then O(**1**, **2**, **3**) is false.

This algorithm was borrowed from the very useful [Geeks for Geeks tutorial](https://www.cdn.geeksforgeeks.org/check-if-two-given-line-segments-intersect/ "Geeks for Geeks tutorial") on intersection of line segments.

### Implementation

The graphics are done in Tkinter, with a backend of Python 2.

#### UI

The user has access to a slider bar along the right side which represents discrete time (ie: time divided into nice steps of 0.5 seconds). It begins at t=0, but cannot yet be moved. The user must first click on the screen to select their goal for the robot to reach. Then, the slider becomes usable.

#### Time
As the user drags the slider bar downwards, three things happen.

First, the obstacles update to their locations at that time, which are calculated based on the obstacles&#39; initial locations and velocities. (For more details, see [Math: Obstacles](#obstacles "Math: Obstacles").)

Second, the RRT updates. The RRT node graph is generated on the fly as the user drags the slider bar. For each tick the user moves the slider, new nodes are generated and stored. Each one is generated in a random direction&mdash;weighted towards the goal&mdash;with a random length. (For more, see [Math: RRT Random Numbers](#rrt-random-numbers "Math: RRT Random Numbers").)

Thirdly, we check the accessibility of every node in the RRT. This is done by comparing each connection between nodes to every obstacle. (For more, see [Math: Collisions](#collisions "Math: Collisions").) If that connection intersects an obstacle, then it cannot be traversed at the current time. In addition, none of the connections and nodes which rely on that connection can be accessed. They are all marked as such. 

Then we check all of the nodes which are accessible at this instant. If any of them are within 30 pixels of the goal, we mark this as a successful path and cap the amount the slider bar can reach.

Credit to [MEditor](https://pandao.github.io/editor.md/en.html) for helping me with making this document!