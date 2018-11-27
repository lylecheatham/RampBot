# RampBot

This robot was fully designed and implemented by us to act as a search and rescue robot satisying several objectives. Namely, the robot was required to traverse over a 1 m tall wall via a thin, steep ramp and to locate a post on the opposite side and then return to the start point. 

An early version of our robot:
![alt text](https://raw.githubusercontent.com/lylecheatham/RampBot/master/resources/proto.jpg)

An early ramp climb:
![alt text](https://raw.githubusercontent.com/lylecheatham/RampBot/master/resources/ramp.png)

The preliminary software architecture was designed using a dual-controller approach to offload more complex portions of the navigation as seen below:
![alt text](https://raw.githubusercontent.com/lylecheatham/RampBot/master/resources/SoftwareFlow.png)

In the final design this dual processor approach was scrapped as a much simpler navigation method was used, preventing the need for more processing power than the teensy would provide.



