Note: This repo was modified and forked from https://bitbucket.org/siteks/kilobox/src/master/

Kilobox - A Kilobot simulator based on Box2D
============================================

This is a kilobot simulator based on the 2D physiscs simulation library
Box2D v2.3.0.

I've modified the standard testbed supplied with Box2D to add the kilobot 
simulation stuff, and hacked the cmake scripts so that I can build an
Xcode project with the standard commands.


Build instructions
==================

Tested on OSX 10.9, OSX 10.10, macOS Sierra 10.12, Ubuntu 14.04. Seems also working on macOS Mojave 10.14.2. 
Many deprecated warnings about glut with OSX builds. Not sure what dependencies are needed for Ubuntu, haven't tried with a clean install. Uses C++11.

```
git clone https://github.com/inmo-jang/kilobox.git
cd kilobox
mkdir kilobox_build
cd kilobox_build
cmake .. 
make
```

Example simulation
==================

- **(1) Communication Function Check**

    At ``kilobox_build`` folder,
        
        ./src/kilobox/kilobox ../worlds/simple_example.world
        

    Expected result will be https://youtu.be/lyxVm6HM0L4.

- **(2) Estimating the distances to food sources**

        ./src/kilobox/kilobox ../worlds/estimation_distance_to_task.world
        

Notes
=====

Directory structure (seems outdated):

    .
    ├── Box2D
    │   ├── Box2D
    │   │   ├── Collision
    │   │   │   └── Shapes
    │   │   ├── Common
    │   │   ├── Dynamics
    │   │   │   ├── Contacts
    │   │   │   └── Joints
    │   │   └── Rope
    │   ├── Build
    │   │   ├── vs2012
    │   │   └── xcode5
    │   │       └── Box2D.xcodeproj
    │   │           └── project.xcworkspace
    │   ├── Documentation
    │   │   ├── API
    │   │   │   └── html
    │   │   │       └── search
    │   │   └── images
    │   ├── HelloWorld
    │   ├── Testbed
    │   │   ├── Framework
    │   │   └── Tests
    │   ├── freeglut
    │   └── glui
    └── worlds
        └── bitmaps


The main Box2D library, under ``Box2D/Box2D``, is almost unchanged. The only changes have 
been to ``Box2D/Common/b2Settings.h``, tuning some values to be more appropriate
to the distance and velocity regime of the kilobots:

    b2_linearSlop               0.005   ->  0.0005
    b2_velocityThreshold        1.0     ->  0.01    
    b2_linearSleepTolerance     0.01    ->  0.001

The Testbed as supplied with the Box2D library in ``Testbed/Framework`` has been heavily 
hacked to give what I wanted for kilobot simulation. The main changes give a
similar command line interface to Stage and move away from the Box2D philosophy of 
accurate simulation appearance in GUI to as fast as possible sim in both GUI and 
command line.

``Testbed/Tests`` contains the code for the actual kilobot world and API emulation.

kiloworld.cpp/h
---------------
This contains the class for the simulated world. It is instantiated by the Testbench
and the constructor builds the Box2D structures to support the physics simulation. 
It does this by reading in a world configuration file that is directly compatible with
Stage. This file specifies the pose and controller for each robot that the world
contains. The example world file - ``test100.world`` - contain a lot of stuff for setting
the geometry of the floorplan and of the robot bodies that is used by Stage but currently
ignored by kilobox.

The worldfile is read using code in ``worldfile.cpp/h`` which is directly lifted from the
Stage codebase. Most commands are ignored, ``quit_time <n>`` is honoured as specifying
the simulation run time when in nongui mode. The model structure is fully parsed and
it is then assumed that all "position" models are kilobots and a kilobot is created for 
each "position" model with the specified pose. 

Each model is checked to see if it specifies a controller but the only controller 
actually instantiated is Evokilo1. Sufficient bits and pieces of the Stage environment
are recreated so that the environment the kilolib code is running in is quite similar
to that of the Stage kilobot simulation.

worldfile.cpp/h
---------------
Worldfile parsing code from Stage.

kilolib.cpp/h
-------------
This actually implements the kilobot model. Each kilobot found in the world 
configuration file will result in the construction of an instance of the controller,
in this case always ``Evokilo1``, which inherits from the class Kilobot.
Kilobot creates the physical model for Box2D, and internal state for providing
the kilolib API that user kilobot code is written against. Every physics timestep 
(currently 50Hz, 20ms), the main loop calls each kilobots ``Kilobot::update()`` method, 
which handles the robot kinematics in order to work out what forces to apply, and 
the kilobot message system. It also calls the ``loop()`` code in the controller.

If the sim is running a GUI, the ``Kilobot::render()`` method is called so the kilobot 
can draw itself

evokilo1.cpp/h
--------------
This is the code that would run on the real kilobot.

This is the only controller currently supported. It is basically compatible with the
real kilobots with some extra code to do position logging to a file. It runs the kilobot 
using a fully recurrent neural net, the weights of which are provided in the worldfile
defining the robot.








Adding a new controller
=======================

The above text was written some time ago, since then, further controllers have been
added for my own use but there is still no general mechanism. Here is how to add 
your own controller.

This is hacky and could definitely be more user-friendly. 

The intent was that a controller would be written in virtually the same way for the simulator
and for the kilobot. To that end, you need to write a class inheriting from Kilobot, with two
methods, ``setup()`` and ``loop()``. The Kilobot class provides the standard kilobot API, except for
``delay()``. 'global' variables go in the class declaration, and can be initialised because this 
is c++11. The ``setup()`` and ``loop()`` methods then use almost identical code to the kilobot
code with some small exceptions:

You can't use ``delay()``. Its very hard to implement nicely in a sim.

Setting the message callbacks has to be done with some extra syntax

    Kilobot:        kilo_message_tx = tx_message;
    Simulation:     kilo_message_tx = (message_tx_t)&controller_class_name::tx_message;

Some code has to be added elsewhere to fit everything together. So:

1) Write a controller class in ``evokilo1.cpp`` and ``evokilo1.h``, with the class inheriting from Kilobot
2) At the beginning of ``evokilo1.cpp``, add another line to initialise the logging file pointer
3) Modify ``kiloworld.cpp`` around line 210 so that a reference in the world file will
    result in the instantiation of your new controller


You can pass arguments to your controller class using ``--args="stuff"`` on the command line.

Logging is implemented with a singleton file pointer, so all instances of a controller write to the same file.
Specify a log file with the command line syntax:

    Testbed -g ../kilobox/worlds/orbit/world --args="log test.log"



Worldfile format
================

The worldfile code is mostly lifted from Stage. Parsin is a bit delicate, with poor error reporting. If something doesn't work, add spaces between tokens like brackets. # for comments.

An example:


    region( rectangle [0 0 3 2 2] )
    region( circle [0 0 0.2 1] )
    # Specify a rectangular region centered on (0,0) with size (3,2) metres, returning 2
    # overlayed with a circular region centered on (0,0) with radius 0.2 m, returning 1
    #
    # Using the api function get_environment() will return the topmost (last set) region value
    # at the point the kilobot is.

    define r0 position ( ctrl "evokilo2 -3.187500 3.093750" )
    # Defines robot type r0, with controller evokilo2, and parameters -3.187500 3.093750. The parameters are
    # passed as a list of words to the controller class on construction.

    global ( params [0.001759 0.095316 0.000971 0.028605 0.032000 20.000000 5.000000 5.000000 0.695555 0.100000 0.110767 0.000078 0.006223 0.011603 0.843987] )
    # Specify simulator parameters. These are:
    #       Name                Default Meaning
    #   1   sigma_vbias         0.0013  stddev of gaussian used to set velocity bias
    #   2   sigma_omegabias     0.0675  stddev of gaussian used to set angular velocity bias
    #   3   sigma_vnoise        0.001   stddev of gaussian added to velocity at each timestep
    #   4   sigma_omeganoise    0.01    stddev of gaussian added to angular velocity at each timestep
    #   5   dia                 0.032   diameter of kilobot
    #   6   density             20.0    density of kilobot in kg/m^2
    #   7   lineardamp          5.0     linear velocity damping coefficient    
    #   8   angulardamp         5.0     angular velocity damping coefficient
    #   9   friction            0.8     mu between kilobots
    #   10  restitution         0.1     restitution between kilobots
    #   11  senserad            0.1     sensor radius, distance messages can travel
    #   12  speedconst          1.14e-4 constant to convert the motor speed (70) to linear 'wheel' velocity 
    #   13  wheeloffset         0.005   offset of wheel axis from centre of robot
    #   14  wheeldist           0.015   distance between wheels
    #   15  msgsuccess          0.95    probability a message will be successfully received
    #
    # The bias and noise figures were obtained by measuring the actual performance of ~20
    # calibrated kilobots in forward and turning motion.
    #
    # The kilobot motion is approximated by two-wheel kinematics, these constants give roughly 
    # the right motion

    quit_time 300
    # Run the simulation for 300 simulated seconds. Ignored in GUI mode.

    r0( pose [ -0.100000  -0.100000 0 -131.628872 ])
    r0( pose [ -0.250000  -0.150000 0 -90.1234 ])
    # Instantiate two robots of type r0 at position (-0.1,-0.1) and (-0.25,-0.15) metres.



Running a simulation
====================

Testbed <worldfile>

Will start the GUI. This is a hacked version of the Box2D one. Set the speed with the
radio buttons. You can drag the kilobots while the sim is running. Use 'z' and 'x' to zoom
in and out in the main window.


Options:

    -g              Run in non-gui mode, respect the "quit_time" command in worldfile
    --args "str"    Pass "str" to all controllers
    --seed <num>    Set the random seed. All controllers have their own random number generator
                    so that it is possible to run reproducible sims. 
                    The seed each one gets is <num> + id.



Speed
=====

Running with 400 robots  for 300 simulated second gives:

    time ./Testbed/Testbed -g ../kilobox/worlds/test_400_kilobots.world
    ...
    real	1m0.487s
    user	0m58.934s
    sys	0m0.233s


So 5x real time, or racc of 300*400/60 = 2000


25 robots for 3000 simulated seconds take 2.12 seconds, or racc of 3000*25/2.12 = 35000

Performance doesn't scale very well..



