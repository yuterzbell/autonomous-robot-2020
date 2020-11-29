package ca.mcgill.ecse211.project;
import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.*;
import java.util.Arrays;

/**
 * The UsLocalizer class implements the ultrasonic localization techniques,
 * either rising or falling edge.
 */
public class UltrasonicLocalizer {

  // Instance and class variables
  /** Thresh of d. */
  private static final int D_THRESH = 30;   // 30 cm
  /** Thresh to account for noise. */
  private static final int K = 2;           // 1 cm
  /** Array index of theta.*/
  private static final int THETA = 2;      
  /** The poll sleep time, in milliseconds. */
  public static final int POLL_SLEEP_TIME = 10000;
  /** Buffer (array) to store US samples. */
  private static float[] usData = new float[usSensor.sampleSize()];
  /** Buffer (array) to store US samples. */
  private static float[] usData2 = new float[usSensortop.sampleSize()];
  /** The distance remembered by the filter() method. */
  private static int prevDistance;
  /** The number of invalid samples seen by filter() so far. */
  private static int invalidSampleCount;
//  /** The window of data measured by usensor. */
//  private static int[] down_dists = new int[9];
  /** Delta theta account for correction of heading. */
  private static double dtheta;
  /** First falling edge. */
  private static double alpha1;
  /** Second falling edge. */
  private static double alpha2;
  /** Angle for back wall is detected. */
  private static double Alpha;
  /** Second failing edge. */
  private static double beta1;
  /** Angle for left wall is detected. */
  private static double beta2;
  /** Angle for left wall is detected. */
  private static double Beta;
  /** Flag to inform which angle is to be measured this time. */
  private static boolean[] Sequence = {false, false, false, false};
  /** Distance measured by current loop. */
  private static int dist;
  /** Distance filtered last loop. */
  private static int prevmedian;
  /** Distance filtered current loop. */
  private static int curmedian;

  /**
   * This method is called inside main function and will adjust heading of robot
   * to 0 deg using only ultrasonic sensor.
   * @author Zichen
   */
  public static void localize() {
    initializeData();

    initializeHeading();

    // Then read data repeatedly to get ALpha and Beta
    // set the program ready to take alpha1
    Sequence[0] = true;
    while (true) { 
      prevmedian = curmedian;       // take record of last reading
      curmedian = Helper.downMedianFiltering(down_dists);
      
      if (prevmedian >= D_THRESH + K && curmedian < D_THRESH + K && Sequence[0]) {
        System.out.println("angle1 detected!!!");
        alpha1 = odometer.getXyt()[THETA];
        Sequence[0] = false;
        Sequence[1] = true;
        continue;
      }
      if (prevmedian >= D_THRESH - K && curmedian < D_THRESH - K && Sequence[1]) {
        System.out.println("angle2 detected!!!");
        alpha2 = odometer.getXyt()[THETA];  
        Alpha = ((alpha1 + alpha2) / 2) % 360;
        Sequence[1] = false;
        Sequence[2] = true;
        // after finished recording alpha, now go counter-clockwise
        rightMotor.forward();
        leftMotor.backward();
        continue;
      }
      
      // then detect 2nd falling edge
      if (prevmedian >= D_THRESH + K && curmedian < D_THRESH + K && Sequence[2]) {
        System.out.println("angle3 detected!!!");
        beta1 = odometer.getXyt()[THETA];         
        Sequence[2] = false;
        Sequence[3] = true;
        continue;
      }
      if (prevmedian >= D_THRESH - K && curmedian < D_THRESH - K && Sequence[3]) {
        System.out.println("angle4 detected!!!");
        beta2 = odometer.getXyt()[THETA]; 
        Beta = ((beta1 + beta2) / 2) % 360;
        Sequence[3] = false;
        break;
      }
      sleepFor(500);        // restrict sampling frequency
    }
    // stop the motors to calculate result
    leftMotor.stop();
    rightMotor.stop();
    dtheta = calculate_dtheta(Alpha, Beta);
    correctOdometer(dtheta);
    // get current position of robot and turn to right direction
    double latest = odometer.getXyt()[THETA];
    // turn robot to the south and then turn 180 deg
    turnBy(-latest);
    turnBy(180);
    odometer.setTheta(0.0);

  }

  /**
   * This method correct the reading of odometer.
   * @param double correction angle
   * @return void
   */
  private static void correctOdometer(double correction) {
    double old = odometer.getXyt()[THETA];
    odometer.setTheta((old + (360 + correction) % 360) % 360); 
  }


  /**
   * This method takes two double theta as parameters and calculate dtheta.
   * @return  correction of data add to the odometer
   */
  private static double calculate_dtheta(double a, double b) {
    double dtheta = 0.0;
    System.out.println("Alpha: " + Alpha + ", Beta: " + Beta);
    if (a < b) {
      dtheta = 45 - (a + b) / 2;
    } else {
      dtheta = 225 - (a + b) / 2;
    }
    System.out.println("dtheta: " + dtheta);
    return dtheta;
  }


  /**
   * This method initialize the data of median filter.
   * @return void
   */
  private static void initializeData() {
    sleepFor(TIMEOUT_PERIOD);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    // initialize the window of our data
    int i = 0;                // counter of initializing dist[]
    while (i < down_dists.length) {
      down_dists[i] = 255;
      i++;
    }
  }

  /**
   * Use median window filtering to filter data.
   * @return  median of the window
   */
  private static int medianFiltering(int[] arr) {
    // shift data window to left by 1
    for (int j = 0; j < arr.length - 1; j++) {
      arr[j] = arr[j + 1];
    }
    int dist = readUsDistance();      // get distance of current loop
    arr[arr.length - 1] = dist;

    // return the filtered data
    return getMedian(arr);
  }

  /**
   * This method initialize the heading to robot away from wall.
   * @return void
   */
  private static void initializeHeading() {
    System.out.println("Robot is finding a open ground.....");
    rightMotor.backward();
    leftMotor.forward();

    dist = readUsDistance();
    while (dist <= OPEN_THRESH) {
      dist = readUsDistance();
//      System.out.println(dist);
    }
    System.out.println("Open ground found! Reinitializing.....");
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    rightMotor.backward();
    leftMotor.forward();
    System.out.println("Robot moving......");
  }

  /**
   *  This method is mainly from lab2. It turns the heading of robot by a certain degree.
   * @param angle turning angle
   */
  public static void turnBy(double angle) {
    double ab_angle = Math.abs(angle);
    int wheelRotation = convertAngle(ab_angle);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    // If ab_angle = 0, do nothing (we stay at same 
    if (angle > 0) {    // If angle is bigger than 0, turn right
      leftMotor.rotate(wheelRotation, true);
      rightMotor.rotate(-wheelRotation, false);
    } else if (angle < 0) {
      leftMotor.rotate(-wheelRotation, true);
      rightMotor.rotate(wheelRotation, false);
    }
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   * @param angle the input angle in degrees
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    double wheelRotations;
    wheelRotations = angle * HETODEGREE; 
    return (int) Math.round(wheelRotations);
  }


  /**
   * Returns median of a given array.
   * @param arr input array
   * @return the filtered data of the window
   */
  public static int getMedian(int[] arr) {
    // System.out.println(arr[0] + ", " + arr[1] + ", " + arr[2] + ", " + arr[3] + ", " + arr[4]);
    int[] copy = arr.clone();
    Arrays.sort(copy);
    int median = 0;
    if (copy.length % 2 == 0) {
      median = (copy[copy.length / 2] + copy[copy.length / 2 - 1]) / 2;
    } else {
      median = copy[copy.length / 2];
    }
    return median;
  }


  /** Returns the filtered distance between the US sensor and an obstacle in cm. 
   *  @return filtered rationale data
   */
  public static int readUsDistance() {
    usSensor.fetchSample(usData, 0);
    // extract from buffer, cast to int, and filter
    return filter((int) (usData[0] * 100.0));
  }
  
  /** Returns the filtered distance between the US sensor and an obstacle in cm. 
   *  @return filtered rationale data
   */
  public static int readUsDistance2() {
    usSensortop.fetchSample(usData2, 0);
    // extract from buffer, cast to int, and filter
    return filter((int) (usData2[0] * 100.0));
  }

  /**
   * Rudimentary filter - toss out invalid samples corresponding to null signal.
   * 
   * @param distance raw distance measured by the sensor in cm
   * @return the filtered distance in cm
   */
  static int filter(int distance) {
    if (distance >= MAX_SENSOR_DIST && invalidSampleCount < INVALID_SAMPLE_LIMIT) {
      // bad value, increment the filter value and return the distance remembered from before
      invalidSampleCount++;
      return prevDistance;
    } else {
      if (distance < MAX_SENSOR_DIST) {
        invalidSampleCount = 0; // reset filter and remember the input distance.
      }
      prevDistance = distance;
      return distance;
    }
  }

}
