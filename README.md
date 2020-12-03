# ECSE211 Project: Team 02 - Atom
## Project Description
  The purpose of this project is to create an autonomous vehicle that can navigate from a corner positioned with a random angle. It should find its way across the bridge inside the playing field and then go identify and push the containers into the target bin. After it finishes all the containers or the time runs out, the robot should drive itself back to the starting point. For details please check our Requirement document Requirements_5.02.docx

## Content of repo
### controller
**T02Controller is the brain of our project.** It contains main controll logic behind each move of our robot.
  
### protos
  **T02_ATOMAV6 folder contains our specific protos for our robot T02_ATOMAV6.**
  
  protos folder contains every detailed characteristics of each part needed to construct the WeBots world inwhich the project will be running.
  Despite the portos provided by the users(professors and TA teams of ECSE211), our robot's body assembly and each part is created inside LeoCad and generated through [WeBots_converter](http://ecse211.ece.mcgill.ca/converter.html).
  
### server
  A folder that set up the server for demo and final competition. The parameters will be transferred to robot through WI-FI.
  
### worlds
  **DpmProject_ATOMAV6.wbt** is the file containing our robot model.
  
  The simulation world our team use for testing purposes.
