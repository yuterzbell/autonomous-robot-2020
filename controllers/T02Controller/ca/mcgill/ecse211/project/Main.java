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
 * 
 * TODO Describe your project overview in detail here (in this Javadoc comment).
 */
public class Main {

  /**
   * The number of threads used in the program (main, odometer), other than the
   * one used to perform physics steps.
   */
  public static final int NUMBER_OF_THREADS = 2;

  /** Main entry point. */
  public static void main(String[] args) {
    initialize();


    identifySelf();
    // Start the odometer thread
    new Thread(odometer).start();   
    //    Navigation.moveStraightFor(3.0);
    new Thread(detector).start();

    UltrasonicLocalizer.localize();
    LightLocalizer.localize();
    Helper.BeepNtimes(3);

    setOdometer();

    moveToBridge();

    moveToSearchZone();
    Helper.BeepNtimes(3);

    moveAndSearch();

    // start the detector thread after initial localizing


  }



  /**
   * This method adjust the robot to new location and searching for container.
   */
  public static void moveAndSearch() {
    for (Point p : getWayPoints()) {
      System.out.println("Next search Point is: " + p);
      Navigation.navigateTo(p);
      Navigation.turnTo(90);
      // sweep for 360 deg
      long startTime = System.currentTimeMillis();
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.rotate(convertAngle(-90), true);
      rightMotor.rotate(-convertAngle(-90), true);
      while (System.currentTimeMillis() - startTime < 4000) {      // polling
        int bottomSensorData = readUsDistance();
        int topSensorData = readUsDistance2();
        if (bottomSensorData < VALID_OFFSET) {

          System.out.println("down sensor is in valid distance");
          System.out.println("Top sensor " + topSensorData);
          System.out.println("Down sensor " + bottomSensorData);

          if (topSensorData > bottomSensorData + US_DIFF_THRESHOLD) {
            calculateAndPush();
            isContainer = false;
          }
        }
        System.out.println("All position covered");
      }
    }

  }

  /**
   * This method returns a list of search Point that the robot should be performing search at.
   * @return a list of search Point for robot to perform search.
   * @author Zichen Chang
   */
  public static ArrayList<Point> getWayPoints(){
    ArrayList<Point> wayPoints = new ArrayList<Point>();
    for (double i = searchZone.ll.x + 0.5; i < searchZone.ur.x; i += 2) {
      for (double j = searchZone.ll.y + 0.5; j < searchZone.ur.y; j += 2) {
        wayPoints.add(new Point(i, j));
      }
    }
    return wayPoints;
  }

  /**
   * This method calculate the container's position and push container up the ramp.
   * @author Zichen Chang
   */
  public static void calculateAndPush() {
    var xyt = odometer.getXyt();
    double dist = ObjectDetection.getbottomSensorData() / 100d;   // dist in meter
    double x = dist * Math.sin(Math.toRadians(xyt[2])); 
    double y = dist * Math.cos(Math.toRadians(xyt[2]));
    Point target = new Point((xyt[0] + x) / TILE_SIZE, (xyt[1] + y) / TILE_SIZE);
    System.out.println("The point is: " + target);
    Navigation.navigateTo(target);
  }


  /**
   * This method drive the robot to the starting point of the searchZone.
   */
  public static void moveToSearchZone() {
    if (isLand.ur.x < startZone.ll.x) {
      // move leftwards
      var sz = new Point(searchZone.ur.x - ROBOT_OFFSET, (tun.ll.y + tun.ur.y) / 2);
      System.out.println("searchzone is at: " + sz);
      Navigation.navigateTo(sz);
      odometer.setY((tun.ll.y + tun.ur.y) / 2 * TILE_SIZE);
      odometer.setTheta(270);
    } else if (isLand.ll.x > startZone.ur.x) {
      // move rightwards
      var sz = new Point(searchZone.ll.x + ROBOT_OFFSET, (tun.ll.y + tun.ur.y) / 2);
      System.out.println("searchzone is at: " + sz);
      Navigation.navigateTo(sz);
      odometer.setY((tun.ll.y + tun.ur.y) / 2 * TILE_SIZE);
      odometer.setTheta(90);
    } else if (isLand.ll.y > startZone.ur.y) {
      // move upwards
      var sz = new Point((tun.ur.x + tun.ll.x) / 2, searchZone.ll.y + ROBOT_OFFSET);
      System.out.println("searchZone is at: " + sz);
      Navigation.navigateTo(sz);
      odometer.setX((tun.ur.x + tun.ll.x) / 2 * TILE_SIZE);
      odometer.setTheta(0);
    } else if (isLand.ur.y < startZone.ll.y) {
      // move downwards
      var sz = new Point((tun.ur.x + tun.ll.x) / 2, searchZone.ur.y - ROBOT_OFFSET);
      System.out.println("Bridge is at: " + sz);
      Navigation.navigateTo(sz);
      odometer.setX((tun.ur.x + tun.ll.x) / 2 * TILE_SIZE);
      odometer.setTheta(180);
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
      odometer.setTheta(270);
    } else if (isLand.ll.x > startZone.ur.x) {
      // move rightwards
      var bridge = new Point(tun.ll.x - ROBOT_OFFSET, (tun.ll.y + tun.ur.y) / 2);
      System.out.println("Bridge is at: " + bridge);
      Navigation.navigateTo(bridge);
      odometer.setY((tun.ll.y + tun.ur.y) / 2 * TILE_SIZE);
      odometer.setTheta(90);
    } else if (isLand.ll.y > startZone.ur.y) {
      // move upwards
      var bridge = new Point((tun.ur.x + tun.ll.x) / 2, tun.ll.y - ROBOT_OFFSET);
      System.out.println("Bridge is at: " + bridge);
      Navigation.navigateTo(bridge);
      odometer.setX((tun.ur.x + tun.ll.x) / 2 * TILE_SIZE);
      odometer.setTheta(0);
    } else if (isLand.ur.y < startZone.ll.y) {
      // move downwards
      var bridge = new Point((tun.ur.x + tun.ll.x) / 2, tun.ur.y + ROBOT_OFFSET);
      System.out.println("Bridge is at: " + bridge);
      Navigation.navigateTo(bridge);
      odometer.setX((tun.ur.x + tun.ll.x) / 2 * TILE_SIZE);
      odometer.setTheta(180);
    }
  }

  /**
   * Set the odometer value based on parameters passed to robot
   * 
   * @author Zichen Chang
   */
  private static void setOdometer() {
    // TODO Auto-generated method stub
    if (Corner == 0) {
      odometer.setXytInTailSize(1, 1, 0);
    } else if (Corner == 1) {
      odometer.setXytInTailSize(14, 1, 270);
    } else if (Corner == 2) {
      odometer.setXytInTailSize(14, 8, 180);
    } else if (Corner == 3) {
      odometer.setXytInTailSize(1, 8, 90);
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