ence463-realtime
================

This is a project that I completed for ENCE463 (Embedded Software Engineering). The project spec says:

> Students are to design and implement a program for a real-time application on the Stellaris LM3S1968 board 
> using the FreeRTOS real time operating system. 

Note that I've uploaded my full project directory -- my own work is found inside the ```Src``` directory. I did this
so that it would be easier to recover after an instance of catastrophic data loss.

Here's the project proposal I wrote:

\section{Introduction}

The purpose of this project is to build an Attitude and Heading Reference System (AHRS) which can determine its own 
orientation in space. External sensor chips are used to provide data on linear acceleration, angular rotation, and 
the direction of magnetic north. The Stellaris evaluation board retrieves this data, processes it using a sensor 
fusion algorithm, and displays the resulting orientation data on the on-board LCD.

\section{Sensor Chips}

Magnetic field measurements are provided by HMC5883L magnetometer. The device continuously makes measurements at 75Hz 
and places measured data in data output registers. If the master does not ensure that the data register is accessed 
before the completion of the next measurement, the data output registers are over-written with the new measurement. The 
Data Ready pin is low for 250 $\mu$sec when data is placed in the data output registers.

Acceleration and rotation measurements are provided by the MPU6050 inertial measurement chip. This chip operates in the 
same way as the HMC5884L magnetometer, except that the acceleration and rotation measurements have a maximum frequency 
of 1 kHz and 8kHz respectively.

Both chips have an \itc interface, and both chips will be connected to the same \itc bus. Each data register for each 
sensor must be read sequentially; there is no way to read them in parallel. However, the sensors operate at different 
frequencies and therefore do not produce measurements in a fixed order.

Each sensor device is associated with a thread that is dedicated to reading data off that device. After reading a 
complete measurement, the thread shall wait on a semaphore. When the DATA\_READY pin on the sensor goes high, the 
associated interrupt shall increment the semaphore. In this way, the DATA\_READY interrupts shall cause the reader 
thread to wake up, read the new data, and go back to sleep.

The sensor data shall be pushed into a triple buffer and processed by a sensor fusion thread.

\section{Sensor Fusion Algorithm and Display}

Each sensor suffers from different sources of errors. The magnetometer is sensitive to soft-iron effects; the 
accelerometer is affected by non-gravity accelerations, and the gyroscope is very sensitive to drift. A Direction 
Cosine Matrix (DCM) filter is used to fuse the data sources into a single orientation estimate.

The DCM filter acts as a processor-intensive background task. It amplifies the need for efficient interrupts and 
efficient foreground handler tasks.

The LCD screen is updated to display each orientation estimate as soon as it is calculated. The GUI is simple and 
text-based, in order to allow more effort to be put into the real-time elements of the project.

\section{Timing and Frequency Limits}

The magnetometer, accelerometer, and gyroscope have maximum measurement rates of 75 Hz, 1 kHz, and 8 kHz respectively. 
The program must be able to read each measurement before it is over-written by the next measurement, and it must also 
be able to process this data using a background task.

Once the program is able to achieve all the specified goals, the microcontroller clock frequency shall be reduced 
(using the internal clock) until the program is unable to keep up with the sensors.

\section{Testing}

The key element of concern is the product's performance as a real-time system. As such, the sensor fusion algorithm is 
primarily important as a constant background load. 

Orientation accuracy shall be quantified by moving the device through a set of simple movements and comparing the 
calculated orientation to the expected orientation. Test movements include moving the device in a straight line along 
a flat surface, and rotating the device in a single axis without any linear motion.

Performance shall be tested by measuring the clock frequency at which it fails to operate. After each orientation 
estimate is calculated, a pin is toggled. Under normal operation the pin should toggle at 4 kHz\footnote{4 kHz is half 
of the output rate of the fastest sensor.}, independant of the microcontroller clock speed. Once the clock speed drops 
below a threshold, the processor will become overloaded, will not be able to process raw data as fast as it is produced,
and will toggle the pin at less than 4 kHz.
