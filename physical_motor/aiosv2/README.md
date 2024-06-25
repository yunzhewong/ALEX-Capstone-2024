details about why each file is built the way it is:

last updated 25/06/24

THE CODE IS THE BEST SOURCE OF TRUTH

- AiosSocket.py

This file is associated with the packaging and sending of data across the various "sockets" (i.e. Ethernet socket, ROS Interface).

As the physical motor receives messages packaged in a JSON format, this is sent in this form (PhysicalSocket).

Similarly, the ROS socket receives the same JSON format to send commands (RosSocket).

The AiosSocket class manages how the commands are sent to each of the socket, depending on which files are turned on or off (AiosSocket).

Change the sending methods by modifying the ROS_ACTIVE and PHYSICAL_ACTIVE flags

- ConnectedMotor.py

This file is associated with the sending of particular motor commands. Note that to send a motor command, the ip of the motor, as well as the active AiosSocket must be included in its constructor. This file can be thought of as sending the literal controls to the motor. It is a more neatly packaged aios.py.

That is, from this class we can send position, velocity, and current commands as well as request data from the motor.

Note that most of the functions do not read data coming back (except PIDConfig) as the reading occurs via the DataStream class through another thread. This is done to ignore the main-thread-blocking properties of reading off a socket.

- constants.py

This file contains the constants as described by the Motor - they should not be changed as they reflect inherent motor software constants that exist on the other side of the Ethernet port.

- CVP.py

This is a dataclass that hold an object for ease of use with the received CVP from a motor. It can be printed in a formatted way.

- DataStream.py

This file contains a class that reads all of the data coming off the AiosSocket. No matter what source/motor requested data, the data is funneled back through the same socket. Hence if we have motors simultaneously calling for CVPs, there could by misaligned due to the ordering of socket data. This class sorts and organises the data into each motor. Furthermore, a single reading thread ensures that the reads do not block the main, command sending thread.

- SafeMotorOperation.py

This file adds to the ConnectedMotor with a few safety considerations, as well as the use of the DataStream.

The SafetyConfiguration class describes a configuration that can be applied to the motor to apply safety limits - i.e. current, velocity, and position limits. The existence of the SafetyConfiguration class requires the use of a DataStream. If we want to check the CVP as frequently as required for safety checks, we need the data to stream in as fast as possible.

The SafeMotor class describes a motor with a list of safety features built into it - it is a higher level abstraction on the low level implementation of the motor commands. Note the use of locks on the get/set CVP values, as we want to avoid race conditions as CVP can be updated from another thread. The safety configuration, control mode checking and the like are used on this class to make a motor operate safely.

- serverConstants.py

Ignore this file, I think it might not be used soon

- TwinMotor.py

This file contains the TwinMotor class which can be thought of as forming a mental model for the testing setup. That is, it contains a top and a bottom motor which are appropriately labelled. I would imagine another implementation of this object is required for our full Exoskeleton.

It also provides a setup + teardown function that ensures that any resources associated with the motor are taken care of.

When we want to connect to the motor, we need to make sure we enable it at the beginning, and turn it off at the end. We also want to make sure that it always checks for errors at every single time step. We also want it to handle errors gracefully and as expected, every time. The part that we want to control is the control message at any given timestep. Hence, we give the ability to choose a control message (given through the actions callback), and handle everything else through the function.

- DataLog.py

The datalog class is just a dataclass that can help hold recorded data, and plot + download the data in a CSV format. It requires CVPs to be added to it with a known timestamp.

- dataGathering.py

This function provides a similar interface to that of the TwinMotor, which is knowing that our behaviour when logging data is always the same. The gather data function which is exported from this file allows for a command function to be passed as a callback, which describes the command to send at the given time step. The rest of the CVP collection, plotting, and download is handled from this one location.
