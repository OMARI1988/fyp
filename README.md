################################################################################
#                               3D Tic Tac Toe                                 # 
################################################################################

This document describe how to start baxter and set up the 3d tic tac toe game
Done by a third year student at the University of Leeds:
Ben Hirsh

Edited by:
Muhannad Al-omari

#######################
# switching baxter ON #
#######################

 *** Baxter won't move, so don't worry ***

To switch baxter ON, 

1- connect the power cable to the back side of baxter (make sure the power
   outlet can handle more than 6 Amps)
   
2- Press the White power button in the back side of baxter.

* Baxter is now on, it won't move so don't worry about switching it ON

* Wait for the screen (Baxter face) to show Rethink Robotics logo


###################################
# connecting baxter to the laptop #
###################################

 *** Baxter won't move, so don't worry ***
 
To connect baxter to your laptop,
1- connect the etherner cable to your laptop, connect the other end to the 
   back of baxter.
*** after connecting the cable you should see that Robotics network is now
    connected.
2- open a terminal on your laptop (ctrl+alt+t = will open a new terminal window)
3- you can connect as many terminal windows to baxter as you want, 

--- to connect a window in terminal to baxter
    1- cd /ros_ws
    2- ./baxter.sh
    
--- to open a new window in terminal 
    1- ctrl+shift+t


################################################
# First thing to do after connecting to baxter #
################################################

 !!! WARNING !!! Baxter will MOVE its arms after executing the next commands.
 
In a window connected to Baxter,
 
--- to untuck baxter's arms (move the arms in front of baxter) 
    1- rosrun baxter_tool tuck_arms.py -u

--- to tuck baxter's arms (move the arms behind baxter) 
    1- rosrun baxter_tool tuck_arms.py -t
    
 ??? Debug ???
 In case the arms don't move or you get an error, 
 1- make sure Robotics network is connected.
 2- make sure the window you are using is connected
 3- make sure the emergency button (red and yellow button connected to Baxter)
    is released
 4- do this 
    rosrun baxter_tools enable_robot.py -e
 5- if both are connected, restart Baxter by pressing the white power button
    untill its OFF, then switch it ON again. 



#####################
# prepare the scene #
#####################
1- set up the table
2- place the A3 paper board which has 9 squares



################################
# Calibrate the object colours #
################################

You probably need this part only once every few hours, as long as the light
don't change a lot. 

1- place a few green block and a few blue block 
2- on a window that is connected to baxter run
---    rosrun fyp Object_finder.py 
       drag the different bars to detect only the green objects first, then 
       the blue objects.
---    hit done



##################
# 3D TIC TAC TOE #
##################

make sure to untuck Baxter arms as explained above.

to start a game,
1- clear the board
2- run the following line on a window that is connected to baxter,
---     rosrun fyp New3DtictacBaxter.py

to stop the game,
---     ctrl+c on the same window
---     if it doesn't stop use (ctrl+\)
---     if it doesn't work restart laptop
