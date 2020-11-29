package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Helper.*;
import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.convertAngle;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.readUsDistance;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.readUsDistance2;
import static simlejos.ExecutionController.*;
import java.lang.*;
import java.util.ArrayList;
import java.util.Set;
import ca.mcgill.ecse211.playingfield.Point;
import ca.mcgill.ecse211.playingfield.Region;
import ca.mcgill.ecse211.test.Testcontroller;
import simlejos.hardware.ev3.LocalEV3;

/**
 * Main class of the program.
 * This the mainly entry of Team02 Atom's project. In this Main class, a sequential controller is responsible for 
 * all avtivities of robot during the competition. Beside the Main class, it has other 8 classes to support any 
 * methods inside this class. Three threads: main, odometer, detector is running parallelly during the project.
 */
public class Main {

  /**
   * The number of threads used in the program (main, odometer), other than the
   * one used to perform physics steps.
   */
  public static final int NUMBER_OF_THREADS = 3;

  
  
  
  /** Main entry point. */
  
  public static void main(String[] args) {
    
    initialize();

    identifySelf();
    // Start the odometer thread
    new Thread(odometer).start();   
    // Start the detector thread
    new Thread(detector).start();
    
   Testcontroller.rampUp();
    
      
//   Navigation.moveStraightFor(3);
/*
    UltrasonicLocalizer.localize();
    LightLocalizer.localize();
    Helper.BeepNtimes(3);

    setOdometer();

    moveToBridge();

    moveToSearchZone();
    Helper.BeepNtimes(3);

    moveAndSearch();
    
    /* Then go back to initial */
  //  moveBackToBridge();
    
    
 //   moveToInitial();


    // start the detector thread after initial localizing


  }

  /**
   * This method adjust the robot to new location and searching for container.
   */
  public static void moveAndSearch() {
//    for (Point p : getWayPoints()) {
    ArrayList<Point> points = getWayPoints();
    boolean successPush = false;
    int i = 0;
    while (i < points.size()) {
      successPush = false;
      elapsedTime = System.currentTimeMillis() - startTime;
      if(elapsedTime > 270000) {
        return;
      }
      Point p = points.get(i);
      Navigation.navigateTo(p);
      Navigation.turnTo(135);
      // sweep for 90 degree sector
      long startTime = System.currentTimeMillis();
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);

      ReinitializeDoubleUsensors();

      leftMotor.rotate(convertAngle(-90), true);
      rightMotor.rotate(-convertAngle(-90), true);
      while (System.currentTimeMillis() - startTime < 3500) {      // polling
        int bottomSensorData = downMedianFiltering(down_dists);
        int topSensorData = topMedianFiltering(top_dists);
        if (bottomSensorData < VALID_OFFSET) {
          if (topSensorData > bottomSensorData + US_DIFF_THRESHOLD) {
            calculateAndPush(bottomSensorData);
            Navigation.goRamp();
            i = 0;      // reset to first search point after finish a success push
            successPush = true;
          }
        } 
      }
      
      if(!successPush)
        i++;
    }
    System.out.println("All position covered, moveAndSearch done");
  }

  /**
   * This method returns a list of search Point that the robot should be performing search at.
   * @return a list of search Point for robot to perform search.
   * @author Zichen Chang
   */
  public static ArrayList<Point> getWayPoints(){
    ArrayList<Point> wayPoints = new ArrayList<Point>();
    for (double i = searchZone.ll.x + 0.5; i < Ramp.left.x; i += 2) {
      for (double j = searchZone.ll.y + 0.5; j < Ramp.right.y; j += 2) {
        wayPoints.add(new Point(i, j));
      }
    }
    return wayPoints;
  }

  /**
   * This method calculate the container's position and push container up the ramp.
   * @param int botReading the reading get from bot sensor, filtered in cm
   * @author Zichen Chang
   */
  public static void calculateAndPush(int botReading) {
    var xyt = odometer.getXyt();

    odometer.printPositionXY();

    double dist = botReading / 100d + BOTTOM_CENTER;        // in meter
    double x = dist * Math.sin(Math.toRadians(xyt[2])); 
    double y = dist * Math.cos(Math.toRadians(xyt[2]));

    System.out.print("the delta x is: " + x);
    System.out.println("\tthe delta y is: " + y);

    Point target = new Point((xyt[0] + x) / TILE_SIZE, (xyt[1] + y) / TILE_SIZE);
    System.out.println("The point is: " + target);
    
    if(xyt[0]/TILE_SIZE < target.x) {      // if robot to the left of container
      target.x = target.x - 0.9;
      Navigation.navigateTo(target);
//      System.out.println("This is when finished \n");
    } else if(xyt[0]/TILE_SIZE > target.x) {    // if robot to the right of container
      target.x = target.x + 0.9;
      Navigation.navigateTo(target);
    }
  }


  /**
   * This method drive the robot to the starting point of the searchZone.
   * @author Zichen Chang
   */
  public static void moveToSearchZone() {
    if (isLand.ur.x < startZone.ll.x) {
      // move leftwards
      var sz = new Point(searchZone.ur.x - ROBOT_OFFSET, (tun.ll.y + tun.ur.y) / 2);
      Navigation.navigateTo(sz);
      odometer.setY((tun.ll.y + tun.ur.y) / 2 * TILE_SIZE);
      odometer.setTheta(270);
    } else if (isLand.ll.x > startZone.ur.x) {
      // move rightwards
      var sz = new Point(searchZone.ll.x + ROBOT_OFFSET, (tun.ll.y + tun.ur.y) / 2);
      Navigation.navigateTo(sz);
      odometer.setY((tun.ll.y + tun.ur.y) / 2 * TILE_SIZE);
      odometer.setTheta(90);
    } else if (isLand.ll.y > startZone.ur.y) {
      // move upwards
      var sz = new Point((tun.ur.x + tun.ll.x) / 2, searchZone.ll.y + ROBOT_OFFSET);
      Navigation.navigateTo(sz);
      odometer.setX((tun.ur.x + tun.ll.x) / 2 * TILE_SIZE);
      odometer.setTheta(0);
    } else if (isLand.ur.y < startZone.ll.y) {
      // move downwards
      var sz = new Point((tun.ur.x + tun.ll.x) / 2, searchZone.ur.y - ROBOT_OFFSET);
      Navigation.navigateTo(sz);
      odometer.setX((tun.ur.x + tun.ll.x) / 2 * TILE_SIZE);
      odometer.setTheta(180);
    }
  }

  /**
   * This method drive the robot to the InitialPosition.
   * @author Zichen Chang
   */
  public static void moveToInitial() {
    if (startZone.ur.x < isLand.ll.x) {
      // move leftwards
      var sz = new Point(searchZone.ur.x - ROBOT_OFFSET, (tun.ll.y + tun.ur.y) / 2);
      Navigation.navigateTo(sz);
      odometer.setY((tun.ll.y + tun.ur.y) / 2 * TILE_SIZE);
      odometer.setTheta(270);
      Navigation.navigateTo(initial);
    } else if (startZone.ll.x > isLand.ur.x) {
      // move rightwards
      var sz = new Point(searchZone.ll.x + ROBOT_OFFSET, (tun.ll.y + tun.ur.y) / 2);
      Navigation.navigateTo(sz);
      odometer.setY((tun.ll.y + tun.ur.y) / 2 * TILE_SIZE);
      odometer.setTheta(90);
      Navigation.navigateTo(initial);
    } else if (startZone.ll.y > isLand.ur.y) {
      // move upwards
      var sz = new Point((tun.ur.x + tun.ll.x) / 2, searchZone.ll.y + ROBOT_OFFSET);
      Navigation.navigateTo(sz);
      odometer.setX((tun.ur.x + tun.ll.x) / 2 * TILE_SIZE);
      odometer.setTheta(0);
      Navigation.navigateTo(initial);
    } else if (startZone.ur.y < isLand.ll.y) {
      // move downwards
      var sz = new Point((tun.ur.x + tun.ll.x) / 2, searchZone.ur.y - ROBOT_OFFSET);
      Navigation.navigateTo(sz);
      odometer.setX((tun.ur.x + tun.ll.x) / 2 * TILE_SIZE);
      odometer.setTheta(180);
      Navigation.navigateTo(initial);
    }
  }

  /**
   * This method will drive the robot in front of bridge when driving back.
   * 
   * @author Zichen Chang
   */
  private static void moveBackToBridge() {
    if (startZone.ur.x < isLand.ll.x) {
      // move leftwards
      var bridge = new Point(tun.ur.x + ROBOT_OFFSET, (tun.ll.y + tun.ur.y) / 2);
      Navigation.navigateTo(bridge);
      odometer.setY((tun.ll.y + tun.ur.y) / 2 * TILE_SIZE);
    } else if (startZone.ll.x > isLand.ur.x) {
      // move rightwards
      var bridge = new Point(tun.ll.x - ROBOT_OFFSET, (tun.ll.y + tun.ur.y) / 2);
      Navigation.navigateTo(bridge);
      odometer.setY((tun.ll.y + tun.ur.y) / 2 * TILE_SIZE);
    } else if (startZone.ll.y > isLand.ur.y) {
      // move upwards
      var bridge = new Point((tun.ur.x + tun.ll.x) / 2, tun.ll.y - ROBOT_OFFSET);
      Navigation.navigateTo(bridge);
      odometer.setX((tun.ur.x + tun.ll.x) / 2 * TILE_SIZE);
    } else if (startZone.ur.y < isLand.ll.y) {
      // move downwards
      var bridge = new Point((tun.ur.x + tun.ll.x) / 2, tun.ur.y + ROBOT_OFFSET);
      Navigation.navigateTo(bridge);
      odometer.setX((tun.ur.x + tun.ll.x) / 2 * TILE_SIZE);
    }
  }
  
  
  /**
   * This method will drive the robot in front of bridge.
   * 
   * @author Zichen Chang
   */
  private static void moveToBridge() {
    if (isLand.ur.x < startZone.ll.x) {
      // move leftwards
      var bridge = new Point(tun.ur.x + ROBOT_OFFSET, (tun.ll.y + tun.ur.y) / 2);
      System.out.println("Bridge is at: " + bridge);
      Navigation.navigateTo(bridge);
      odometer.setY((tun.ll.y + tun.ur.y) / 2 * TILE_SIZE);
    } else if (isLand.ll.x > startZone.ur.x) {
      // move rightwards
      var bridge = new Point(tun.ll.x - ROBOT_OFFSET, (tun.ll.y + tun.ur.y) / 2);
      System.out.println("Bridge is at: " + bridge);
      Navigation.navigateTo(bridge);
      odometer.setY((tun.ll.y + tun.ur.y) / 2 * TILE_SIZE);
    } else if (isLand.ll.y > startZone.ur.y) {
      // move upwards
      var bridge = new Point((tun.ur.x + tun.ll.x) / 2, tun.ll.y - ROBOT_OFFSET);
      System.out.println("Bridge is at: " + bridge);
      Navigation.navigateTo(bridge);
      odometer.setX((tun.ur.x + tun.ll.x) / 2 * TILE_SIZE);
    } else if (isLand.ur.y < startZone.ll.y) {
      // move downwards
      var bridge = new Point((tun.ur.x + tun.ll.x) / 2, tun.ur.y + ROBOT_OFFSET);
      System.out.println("Bridge is at: " + bridge);
      Navigation.navigateTo(bridge);
      odometer.setX((tun.ur.x + tun.ll.x) / 2 * TILE_SIZE);
    }
  }

  /**
   * Set the odometer value based on parameters passed to robot
   * 
   * @author Zichen Chang
   */
  private static void setOdometer() {
    if (Corner == 0) {
      odometer.setXytInTailSize(1, 1, 0);
      initial = new Point(1, 1);
    } else if (Corner == 1) {
      odometer.setXytInTailSize(14, 1, 270);
      odometer.printPositionXY();
      initial = new Point(14, 1);
    } else if (Corner == 2) {
      odometer.setXytInTailSize(14, 8, 180);
      initial = new Point(14, 8);
    } else if (Corner == 3) {
      odometer.setXytInTailSize(1, 8, 90);
      initial = new Point(1, 8);
    } else {
      System.out.println("Error: Corner is not well identified");
    }
  }

  /**
   * This method process the parameters passed to robot in first.
   * 
   * @author Zichen Chang
   */
  private static void identifySelf() {
    // if self team is the redTeam
    if (redTeam == team) {
      Corner = redCorner;
      Ramp = rr;
      Point left = Ramp.left;
      Point right = Ramp.right;
      if (left.x == right.x) {
        if (left.y < right.y) {
          orient = "EAST";
        } else {
          orient = "WEST";
        }
      } else if (left.y == right.y) {
        if (left.x < right.x) {
          orient = "SOUTH";
        } else {
          orient = "NORTH";
        }
      }
      startZone = red;
      isLand = island;
      tun = tnr;
      searchZone = szr;
    } else if (greenTeam == team) {
      Corner = greenCorner;
      Ramp = gr;
      startZone = green;
      isLand = island;
      tun = tng;
      searchZone = szg;
    }
    startTime = System.currentTimeMillis();
  }

  /**
   * Example using WifiConnection to communicate with a server and receive data
   * concerning the competition such as the starting corner the robot is placed
   * in.<br>
   * 
   * <p>
   * Keep in mind that this class is an <b>example</b> of how to use the Wi-Fi
   * code; you must use the WifiConnection class yourself in your own code as
   * appropriate. In this example, we simply show how to get and process different
   * types of data.<br>
   * 
   * <p>
   * There are two variables you MUST set manually (in Resources.java) before
   * using this code:
   * 
   * <ol>
   * <li>SERVER_IP: The IP address of the computer running the server application.
   * This will be your own laptop, until the beta beta demo or competition where
   * this is the TA or professor's laptop. In that case, set the IP to the default
   * (indicated in Resources).</li>
   * <li>TEAM_NUMBER: your project team number.</li>
   * </ol>
   * 
   * <p>
   * Note: You can disable printing from the Wi-Fi code via
   * ENABLE_DEBUG_WIFI_PRINT.
   * 
   * @author Michael Smith, Tharsan Ponnampalam, Younes Boubekeur, Olivier
   *         St-Martin Cormier
   */
  public static void wifiExample() {
    System.out.println("Running...");

    // Example 1: Print out all received data
    System.out.println("Map:\n" + wifiParameters);

    // Example 2: Print out specific values
    System.out.println("Red Team: " + redTeam);
    System.out.println("Green Zone: " + green);
    System.out.println("Island Zone, upper right: " + island.ur);
    System.out.println("Red tunnel footprint, lower left y value: " + tnr.ll.y);

    // Example 3: Compare value
    if (szg.ll.x >= island.ll.x && szg.ll.y >= island.ll.y) {
      System.out.println("The green search zone is on the island.");
    } else {
      System.err.println("The green search zone is in the water!");
    }

    // Example 4: Calculate the area of a region
    System.out.println("The island area is " + island.getWidth() * island.getHeight() + ".");
  }

  /**
   * Initializes the robot logic. It starts a new thread to perform physics steps
   * regularly.
   */
  private static void initialize() {
    // Run a few physics steps to make sure everything is initialized and has
    // settled properly
    for (int i = 0; i < 50; i++) {
      performPhysicsStep();
    }

    // We are going to start two threads, so the total number of parties is 2
    setNumberOfParties(NUMBER_OF_THREADS);

    // Does not count as a thread because it is only for physics steps
    new Thread(() -> {
      while (performPhysicsStep()) {
        sleepFor(PHYSICS_STEP_PERIOD);
      }
    }).start();
  }

}