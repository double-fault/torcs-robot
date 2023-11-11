# torcs-robot
A simple torcs robot. Main robot code is in `driver.cpp`. A demo video of the robot [over one lap of the Wheel 1 track can be seen here.](https://drive.google.com/file/d/1-oBCfAb8D0HyH8xtm2CRk2N_OoIijZc4/view) 

Rough overview of the project is as follows. 

# Coming up with a racing line

1. Simplistic implementation of the K1999 algorithm, described in [Appendix C of Remi Coulom's PhD Thesis](https://www.remi-coulom.fr/Publications/Thesis.pdf).
2. To make the racing line smooth, have put a cubic spline through the various discrete track nodes. Eventually the cubic spline is also discretized, but it is a lot smoother, as the initial track nodes were kept at relatively large distances to allow for a quick convergence of the K1999 algorithm.

A close-up of the racine line generated on the last section of the Wheel 1 track:

![image](https://github.com/double-fault/torcs-robot/assets/12422882/132cc346-a253-4dd5-b38c-af01f7bd53a1)

# Acceleration, Braking and Gear changes

Have used a simplistic model ![image](https://github.com/double-fault/torcs-robot/assets/12422882/6b7255bb-3e62-48ac-ae2f-2ac071abc80d)

Downforce has been modeled as follows, with alpha and beta being arbitrary coefficients found by trial-and-error. Can also be found by proper aerodynamic modeling, but getting the coefficient by trial-and-error works decent enough for a simple robot. 

![image](https://github.com/double-fault/torcs-robot/assets/12422882/af742e38-36f4-47cc-8734-985eee5e93b6)

On every segment of the track we are either accelerating or braking. Using the above equations we can get the max allowed speed on every segment of the track. 

A simple approach to figure out whether we need to brake or not is the following; we look at a segment some L distance ahead. Let the current speed be u and the maximum alloweed speed on the segment we are looking at be v. If v < u, we calculate the distance it would take for the car to reduce its speed from u to v. Call this distance brake distance d. If d > L, we brake. We get d by assuming maximum retardation force when we brake, which gives us an integral: 

![image](https://github.com/double-fault/torcs-robot/assets/12422882/1f22a1ff-a4be-4627-9663-ee5b3ca49e1b)

We change gears at fixed engine RPM values. 



