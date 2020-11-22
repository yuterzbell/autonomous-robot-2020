package ca.mcgill.ecse211.project;
import static ca.mcgill.ecse211.project.Resources.*;
import simlejos.hardware.ev3.LocalEV3;
import static simlejos.ExecutionController.*;
import ca.mcgill.ecse211.playingfield.Point;

public class Helper {

   /**
   * Converts the input distance to an angle to rotate, based on the
   * formula {@code (arc length) = (radius) * (theta)}.
   * 
   * @param distance The length of the arc traced out by the turn
   * @return
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Turns the robot by a specified angle. Note that this method is different from
   * {@code Navigation.turnTo()}. For example, if the robot is facing 90 degrees,
   * calling {@code turnBy(90)} will make the robot turn to 180 degrees, but
   * calling {@code Navigation.turnTo(90)} should do nothing (since the robot is
   * already at 90 degrees).
   * 
   * @param angle the angle by which to turn, in degrees
   */
  public static void turnBy(double angle) {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    leftMotor.rotate(angleToDist(angle), true);
    rightMotor.rotate(-angleToDist(angle), false);
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the
   * robot by that angle.
   * 
   * @param angle the input angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int angleToDist(double angle) {
    return convertDistance(Math.PI * BASE_WIDTH * angle / 360.0);
  }
  
  /**
   * Moves the robot straight for the given distance.
   * 
   * @param distance in meters, may be negative
   * @author Andre-Walter Panzini
   */
  public static void moveStraightFor(double distance) {
    int wheelRotation = Helper.convertDistance(distance);
    
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    leftMotor.rotate(wheelRotation, true);
    rightMotor.rotate(wheelRotation, false);
  } 
  
  /**
   * Moves the robot straight.
   * @author Zichen Chang
   */
  public static void moveStraight() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    leftMotor.forward();
    rightMotor.forward();
  } 
  
  /**
   * Stop the Robot.
   * @author Zichen Chang
   */
  public static void stop() { 
    leftMotor.stop();
    rightMotor.stop();
  } 
  
  /**
   * This method control robot beep n times.
   * @param n
   */
  public static void BeepNtimes(int n) {
    for(int i = 0; i < n; i++) {
      LocalEV3.getAudio().beep(); // beeps once
      sleepFor(20000);
    }
  }
  
  /**
   *  This method reinitialize the readings for both sensors, should be called before medianFiltering.
   *  @author Zichen Chang
   */
  public static void ReinitializeDoubleUsensors() {
    int i = 0;                // counter of initializing dist[]
    int down = UltrasonicLocalizer.readUsDistance();
    int top = UltrasonicLocalizer.readUsDistance2();
//    System.out.println("Initial top reading = " + top);
    while (i < down_dists.length) {
      down_dists[i] = down;
      top_dists[i] = top;
      i++;
    }
  }
  
  
  /**
   * Use median window filtering to filter data read from down sensor
   * @author Zichen Chang
   * @return  median of the window
   */
  public static int downMedianFiltering(int[] arr) {
    // shift data window to left by 1
    for (int j = 0; j < arr.length - 1; j++) {
      arr[j] = arr[j + 1];
    }
    int dist = UltrasonicLocalizer.readUsDistance();      // get distance of current loop
    arr[arr.length - 1] = dist;

    // return the filtered data
    return UltrasonicLocalizer.getMedian(arr);
  }
  
  /**
   * Use median window filtering to filter data read from down sensor
   * @author Zichen Chang
   * @return  median of the window
   */
  public static int topMedianFiltering(int[] arr) {
    // shift data window to left by 1
    for (int j = 0; j < arr.length - 1; j++) {
      arr[j] = arr[j + 1];
    }
    int dist = UltrasonicLocalizer.readUsDistance2();      // get distance of current loop
    arr[arr.length - 1] = dist;

    // return the filtered data
    return UltrasonicLocalizer.getMedian(arr);
  }
  
  /**
   * This method returns current location of robot as a Point.
   * @return current Location as Point.
   * @author Zichen Chang
   */
  public static Point curLocation() {
    var xyt = odometer.getXyt();
    var cur = new Point(xyt[0] / TILE_SIZE, xyt[1] / TILE_SIZE);
    return cur;
  } 
  
}
