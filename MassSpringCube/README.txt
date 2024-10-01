<Please submit this file with your solution.>

CSCI 520, Assignment 1

Yukun Zhou

================

<Description of what you have accomplished>
<Also, explain any extra credit that you have implemented.>

Executable file is called jello.exe in bin/release, using 
start "jello.exe" "../../jello.w" to start it.

Basic:
The simulation of a cube by the mass spring system. In total, I built 6 structure strings, 20 shear strings, and 6 bend strings per mass point. Corresponding elascity and dump forces and extra force fields are considered to comupte the velocity. For the force field，trilinear interpolation is used.

The collision detection and response. I checked the coordinates to detect collision.Penalty method is used to response the collide. 

Extra credit:
1. Inclined plane. Related codes are mainly in the collision_detect function in physics.cpp. Of course, I suppose the cube can move in the side of the plane where the original point is. We checked the value of the plane equation with the coordinate of each point to determine whether collision happened. The plane equation is used to compute the distances between the plane and the points inside.

Sometimes the points may go inside the cube by collsions but for a while it will recover the shape by the springs we set in the basic part.

2. Drag a point. It's mainly handled in ProcessClick and ProcessDrag functions in physics.cpp. When the mouse clicks a point, call ProcessClick. We need to get the depth of the pixel of the current mouse position   in the screen space, and transform this to the world space. And compare the distance between each surface point and this world space coordinate to get the nearest surface point.

Then ProcessDrag will handle it when mouse is pressed and moved. This time I use the previous depth to unproject the mouse position, as if there is a plane, paralleling to the screen, and across the previous position of the selected point. To avoid unstoppable movement, I set all velocities to 0 when a point is clicked at first. It can move when we start to drag.

Sometimes we may click to the point behind in the wireframe mode, so in that case the strings will bounce dramatically. We can observe the point is bouncing back and forth, as we expected.  