# <strong>ERC Robotics Hackathon 2022: Project MILF🅱️ot</strong>

## <strong>Automation</strong>
<p>Aditya Kurande</p>
<p>Arjun Puthli</p>
<p>Tejas Sovani</p>

## <strong>Electronics</strong>
<p>Dhruv Patel</p>
<p>Aditya Mallik</p>

## <strong>Mechanical</strong>
<p>Smrithi Lokesh Seramalanna</p>

## <strong>Running the automation package:</strong>

We were responsible for fixing the launch file, so we should get a treat :P

```
$ roslaunch robotics_hackathon_automation automation task.launch
```
After running the above command you should see a gazebo window open. In the terminal tab you ran the command in you should see a progress report as to how much of the path planning process has been completed. Upon reaching 100% a matplotlib window will open which will display the path the bot will take. Upon closing this tab, the bot will start to move. 

```
$ rostopic echo cleaning_mode
```

Run the above command in another tab to see the current cleaning mode of the robot.

## <strong>[Electronics Simulation](https://www.tinkercad.com/things/hT2ufMqzMAE-modified-copy-of-hackathon-electronics/)</strong> 

<br>

## <strong>Automation Summary</strong>

<p>We first tried implementing RRT* to find the path, however it was ineffecient. The time required to compute the path was inconsistent and would take anywhere between 5 and 15 minutes. We then used a  combination of PRM and A* which again was inconsistentm mostly due to the random nature of PRM. Finally, we implemented an algorithm to generate a grid of nodes, keeping the search algorithm the same (A*)</p>

<p>For path-planning we have a grid of points and deletes whichever points coincide with obstacles. These points are then connected to all points lying less than a diagonals distance in order to form a road map for our A* algorithm. We then obtain the planned path in form of an array using the A* algorithm. In addition to this we made a straight line detector which checks if n points of the final path are in a straight line, and deletes the n-2 points in between to prevent intermediate stops on a straight line.</p>

<p>For colour detection we are using the in-built camera of turtlebot to capture video footage. Then making use of the individual frames, and a 3x3 array of colour sensors positioned evenly, the bot can detect the colour markers on the floor.</p>

<p>We started out with a simple bang-bang controller which served well for testing purposes. We improved upon it by implementing a PID controller which overall saves travel time</p>