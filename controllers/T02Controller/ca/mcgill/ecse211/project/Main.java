package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Helper.*;
import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.*;
import java.lang.*;
import ca.mcgill.ecse211.playingfield.Point;
import ca.mcgill.ecse211.test.Testcontroller;
import simlejos.hardware.ev3.LocalEV3;

/**
 * Main class of the program.
 * 
 * TODO Describe your project overview in detail here (in this Javadoc comment).
 */
public class Main {

  /**
   * The number of threads used in the program (main, odometer), other than the one used to perform physics steps.
   */
  public static final int NUMBER_OF_THREADS = 3;

  /** Main entry point. */
  public static void main(String[] args) {
    initialize();
    
    // Start the odometer thread
    new Thread(odometer).start();   
//    Navigation.moveStraightFor(3.0);
    new Thread(detector).start();

    
//    Testcontroller.readDown();
//    Testcontroller.readTop();
    
    
    // testing for readings
//    ReinitializeDoubleUsensors();
//    int down = downMedianFiltering(down_dists);
//    int top = topMedianFiltering(top_dists);
//    System.out.println("Top readings:" + top + "\nDown readings: " + down);
//    System.out.println("Is there a container? " + ObjectDetection.containerDetect());
   
     // start the detector thread after initial localizing
   

/*       
    var bridge = new Point(tnr.ll.x - ROBOT_OFFSET, tnr.getHeight() / 2 + tnr.ll.y);
    System.out.println("Bridge is at: " + bridge);
    Navigation.navigateTo(bridge);
    odometer.setY((tnr.getHeight()/2 + tnr.ll.y) * TILE_SIZE);
       
    var searchZone = new Point(szr.ll.x + ROBOT_OFFSET, tnr.getHeight() / 2 + tnr.ll.y);
    System.out.println("SearchZone is at: " + searchZone);
    Navigation.navigateTo(searchZone);
    Helper.BeepNtimes(3);
  
    // first turn the robot to 180 deg
    odometer.setTheta(90);
    Navigation.turnBy(Navigation.minimalAngle(odometer.getXyt()[2], 180));
    // then detect container
    while(!ObjectDetection.containerDetect()) {
      System.out.println("Keep searching");
      sleepFor(500);
    }
    // when find the container
    Helper.BeepNtimes(3);
    */
    
  }
  
  /**
   * This method will drive the robot in front of bridge.
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
      throw new RuntimeException();
    }
  }

  /**
   * This method process the parameters passed to robot in first.
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
   * Example using WifiConnection to communicate with a server and receive data concerning the competition such as the
   * starting corner the robot is placed in.<br>
   * 
   * <p>
   * Keep in mind that this class is an <b>example</b> of how to use the Wi-Fi code; you must use the WifiConnection
   * class yourself in your own code as appropriate. In this example, we simply show how to get and process different
   * types of data.<br>
   * 
   * <p>
   * There are two variables you MUST set manually (in Resources.java) before using this code:
   * 
   * <ol>
   * <li>SERVER_IP: The IP address of the computer running the server application. This will be your own laptop, until
   * the beta beta demo or competition where this is the TA or professor's laptop. In that case, set the IP to the
   * default (indicated in Resources).</li>
   * <li>TEAM_NUMBER: your project team number.</li>
   * </ol>
   * 
   * <p>
   * Note: You can disable printing from the Wi-Fi code via ENABLE_DEBUG_WIFI_PRINT.
   * 
   * @author Michael Smith, Tharsan Ponnampalam, Younes Boubekeur, Olivier St-Martin Cormier
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
   * Initializes the robot logic. It starts a new thread to perform physics steps regularly.
   */
  private static void initialize() {
    // Run a few physics steps to make sure everything is initialized and has settled properly
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