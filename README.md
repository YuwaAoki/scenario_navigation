# scenario_navigation  

### Requirement  

orne_navigation  
https://github.com/open-rdc/orne_navigation  

### Execute  

- Simulator (without vison)  
1) Launch the simulator  
`roslaunch orne_bringup orne_alpha_sim.launch`

2) Move robot to initial position  

3) Select a scenario in "navigation.launch"  
`<param name="scenario_path" value="$(find scenario_navigation)/config/Scenarios/scenario01.txt" />`

4) Execute the scenario navigation  
`roslaunch scenario_navigation navigation.launch`
