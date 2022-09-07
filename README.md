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