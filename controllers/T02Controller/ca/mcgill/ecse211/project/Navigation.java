package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static java.lang.Math.*;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.*;

import ca.mcgill.ecse211.playingfield.Point;

/**
 * The Navigation class mainly provides several motion modes for the robot to move in different scenarios. 
 * 
 */
public class Navigation {

  /** Do not instantiate this class. */
  private Navigation() {}

  /**
   * Navigate robot first in y-axis, then in x-axis.
   * @param destination- target point
   * @author Zichen Chang
   */
  public static void navigateTo(Point destination) {
    var xyt = odometer.getXyt();

    int travelY = 0;
    int travelX = 0;
    if (xyt[1] / TILE_SIZE > destination.y) {
      travelY = -1;
    } else if (xyt[1] / TILE_SIZE < destination.y){
      travelY = 1;
    }
    if (xyt[0] / TILE_SIZE > destination.x) {
      travelX = -1;
    } else if (xyt[0] / TILE_SIZE < destination.x) {
      travelX = 1;
    }

    // move in y first 
    var pointX = new Point(xyt[0] / TILE_SIZE, destination.y);
    travelTo(pointX, 0, travelY);
    //    System.out.println("Finished y-direction move");
    //    odometer.printPositionXY();
    // move in x then
    xyt = odometer.getXyt();
    var pointY = new Point(destination.x, xyt[1] / TILE_SIZE);
    travelTo(pointY, travelX, 0);
    //    System.out.println("Finished x-direction move");
    //    odometer.printPositionXY();
  }

  /** Travels to the given destination. */
  public static void travelTo(Point destination, int travelFactorX, int travelFactorY) {
    var xyt = odometer.getXyt();
    var currentLocation = new Point(xyt[0] / TILE_SIZE, xyt[1] / TILE_SIZE);
    var currentTheta = xyt[2];
    var destinationTheta = getDestinationAngle(currentLocation, destination);

    turnBy(minimalAngle(currentTheta, destinationTheta));
    int moveInX = 1;
    if (destination.x != xyt[0] / TILE_SIZE) {
      moveInX = 1;
    } else if (destination.y != xyt[1] / TILE_SIZE) {
      moveInX = 0;
    }
    if(DETECT_WATER){
      ColorDetection.moveStraightWithLineCorrectionAndWaterDetection(moveInX, distanceBetween(currentLocation, destination));
    }else{
      //      moveStraightWithLineCorrection(moveInX, distanceBetween(currentLocation, destination));
      moveStraightWithObjectAvoidance(distanceBetween(currentLocation, destination), travelFactorX, travelFactorY);
    }    
  }

  /** Navigates the robot to the ramp. */
  public static void goRamp() {
    Point left = Ramp.left;
    Point right = Ramp.right;
    System.out.println();
    var xyt = odometer.getXyt();


    double xRamp = (left.x + right.x)/2;
    double yRamp = (left.y + right.y)/2;



    if(left.y < right.y && left.x == right.x) {
      System.out.println("1");
      xRamp = xRamp +1;
      Point ramp = new Point(xRamp, yRamp);
      navigateTo(ramp);
      turnTo(270);
      //moveRobotBackwardsFromRamp();

    }
    if(left.y > right.y && left.x == right.x) {
      System.out.println("2");
      xRamp = xRamp -1;
      Point ramp = new Point(xRamp, yRamp);
      navigateTo(ramp);
      turnTo(90);
      // moveRobotBackwardsFromRamp();
    }


    if(left.x < right.x && left.y == right.y) {
      System.out.println("3");
      yRamp = yRamp -1;
      System.out.println(xRamp);
      Point ramp = new Point(xRamp, yRamp);
      navigateTo(ramp);
      turnTo(0);
      //moveRobotBackwardsFromRamp();

    }

    if(left.x > right.x && left.y == right.y) {
      System.out.println("4");
      yRamp = yRamp +1;
      Point ramp = new Point(xRamp, yRamp);
      navigateTo(ramp);
      turnTo(180);
      // moveRobotBackwardsFromRamp();
    }




  }

  /**
   * Moves the robot backwards one tile after dropping the container in the bin
   */
  public static void moveRobotBackwardsFromRamp(){
    int bottomReading = -1;
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    System.out.println("in");
    while(true){
      forward();
      bottomReading = readUsDistance();
      if(bottomReading > 80){
        leftMotor.stop();
        rightMotor.stop();
        moveStraightFor(-(TILE_SIZE)*4);
        break;
      }
    } 
  }

  /** Returns the angle that the robot should point towards to face the destination in degrees. */
  public static double getDestinationAngle(Point current, Point destination) {
    return (toDegrees(atan2(destination.x - current.x, destination.y - current.y)) + 360) % 360;
  }

  /** Returns the signed minimal angle from the initial angle to the destination angle. */
  public static double minimalAngle(double initialAngle, double destAngle) {
    var dtheta = destAngle - initialAngle;
    if (dtheta < -180) {
      dtheta += 360;
    } else if (dtheta > 180) {
      dtheta -= 360;
    }
    return dtheta;
  }

  /**
   * Turns the robot with a minimal angle towards the given input angle in degrees, no matter what its current
   * orientation is. This method is different from {@code turnBy()}.
   * 
   * @param angle final angle to turn to
   */
  public static void turnTo(double angle) {

    turnBy(minimalAngle(Odometer.getOdometer().getXyt()[2], angle));

  }


  /** Returns the distance between the two points in tile lengths. */
  public static double distanceBetween(Point p1, Point p2) {
    var dx = p2.x - p1.x;
    var dy = p2.y - p1.y;
    return sqrt(dx * dx + dy * dy);
  }

  // TODO Bring Navigation-related helper methods from Labs 2 and 3 here

  /**
   * Moves the robot straight for the given distance.
   * 
   * @param distance in feet (tile sizes), may be negative
   */
  public static void moveStraightFor(double distance) {
    setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(distance * TILE_SIZE), true);
    rightMotor.rotate(convertDistance(distance * TILE_SIZE), false);
  }

  /**
   * Moves the robot for a certain distance and corrects the orientation of the robot
   * when it crosses a black line.
   * 
   * @param moveInX integer indicating robot is move in x-direction if equal to 1
   * @param distance number of tiles (in feet) to move straight for
   */
  public static void moveStraightWithLineCorrection(int moveInX, double distance) { 
    double inMeters = distance * TILE_SIZE;
    double distanceChange = 0;

    while (inMeters > TILE_SIZE) {    // while distance left can cover another tile
      if (DETECT_FLAG) {
        ObjectDetection.objectAvoidance();
      }
      var xyt0 = odometer.getXyt();

      LightLocalizer.moveUntilBlackLineDetected();
      Helper.moveStraightFor(COLOR_SENSOR_TO_WHEEL_DIST);        // TODO remember to change the speed here to FORWARD_SPEED

      var xyt1 = odometer.getXyt();

      if (moveInX == 1) {
        double x = xyt1[0] / TILE_SIZE;             // calculate the x-coordinate
        odometer.setX(Math.round(x) * TILE_SIZE);   // round the x-coordinate
        if (xyt1[2] < 100 && xyt1[1] > 80) {
          odometer.setTheta(90);
        } else if (xyt1[2] < 280 && xyt1[1] > 260){
          odometer.setTheta(270);
        }
      } else if (moveInX == 0){
        double y = xyt1[1] / TILE_SIZE;             // calculate the x-coordinate
        odometer.setY(Math.round(y) * TILE_SIZE);   // round the x-coordinate
        if (xyt1[2] < 10 && xyt1[1] > 350) {
          odometer.setTheta(0);
        } else if (xyt1[2] < 190 && xyt1[1] > 170){
          odometer.setTheta(180);
        }
      }
      distanceChange = Math.sqrt(Math.pow((xyt0[1] - xyt1[1]), 2) + Math.pow((xyt0[0] - xyt1[0]), 2));
      inMeters -= distanceChange;
    }
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    System.out.println("Here");
    moveStraightFor(inMeters / TILE_SIZE);
  }

  /** Moves the robot forward for an indeterminate distance. */
  public static void forward() {
    setSpeed(FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();
  }

  /** Moves the robot backward for an indeterminate distance. */
  public static void backward() {
    setSpeed(FORWARD_SPEED);
    leftMotor.backward();
    rightMotor.backward();
  }

  /**
   * Turns the robot by a specified angle. Note that this method is different from
   * {@code turnTo()}. For example, if the robot is facing 90 degrees, calling
   * {@code turnBy(90)} will make the robot turn to 180 degrees, but calling
   * {@code turnTo(90)} should do nothing (since the robot is already at 90 degrees).
   * 
   * @param angle the angle by which to turn, in degrees
   */
  public static void turnBy(double angle) {
    setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(angle), true);
    rightMotor.rotate(-convertAngle(angle), false);
  }




  /** Rotates motors clockwise. */
  public static void clockwise() {
    setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();
  }

  /** Rotates motors counterclockwise. */
  public static void counterclockwise() {
    setSpeed(ROTATE_SPEED);
    leftMotor.backward();
    rightMotor.forward();
  }

  /** Stops both motors. This also resets the motor speeds to zero. */
  public static void stopMotors() {
    leftMotor.stop();
    rightMotor.stop();
  }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance the input distance in meters
   * @return the wheel rotations necessary to cover the distance in degrees
   */
  public static int convertDistance(double distance) {
    return (int) toDegrees(distance / WHEEL_RAD);
  }

  /**
   * Converts input angle to total rotation of each wheel needed to rotate robot by that angle.
   * 
   * @param angle the input angle in degrees
   * @return the wheel rotations (in degrees) necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(toRadians((BASE_WIDTH / 2) * angle));
  }

  /**
   * Sets the speed of both motors to the same values.
   * 
   * @param speed the speed in degrees per second
   */
  public static void setSpeed(int speed) {
    setSpeeds(speed, speed);
  }

  /**
   * Sets the speed of both motors to different values.
   * 
   * @param leftSpeed the speed of the left motor in degrees per second
   * @param rightSpeed the speed of the right motor in degrees per second
   */
  public static void setSpeeds(int leftSpeed, int rightSpeed) {
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
  }

  /**
   * Sets the acceleration of both motors.
   * 
   * @param acceleration the acceleration in degrees per second squared
   */
  public static void setAcceleration(int acceleration) {
    leftMotor.setAcceleration(acceleration);
    rightMotor.setAcceleration(acceleration);
  }

  /**
   * Moves robot in a straight while checking each tile for potential obstacle and avoiding it accordingly
   * 
   * @param distance the distance to be moved in a straight line
   * @param travelFactorX indicates if robot is moving along x axis (-1 for -x and +1 for +x)
   * @param travelFactorY indicates if robot is moving along y axis (-1 for -y and +1 for +y)
   */
  public static void moveStraightWithObjectAvoidance(double distance, int travelFactorX, int travelFactorY) {

    double curr_odo = 0;
    if (travelFactorY == 0) {
      curr_odo = odometer.getXyt()[0] / TILE_SIZE;
    } else {
      curr_odo = odometer.getXyt()[1] / TILE_SIZE;
    }

    double stepFactor = 0;

    while (distance != 0) {
      if (ObjectDetection.obstacleDetect()) {
        stepFactor = dodge(travelFactorX, travelFactorY);
        if (travelFactorY == 0) {
          curr_odo = odometer.getXyt()[0] / TILE_SIZE;
        } else {
          curr_odo = odometer.getXyt()[1] / TILE_SIZE;
        }
        distance = distance - stepFactor;
      } else {
        if (distance > 1) {
          moveStraightFor(1);
          if (travelFactorY == 0) {
            curr_odo = curr_odo + (travelFactorX);
            distance--;
            odometer.setX(curr_odo * TILE_SIZE);
          } else {
            curr_odo = curr_odo + (travelFactorY);
            distance--;
            odometer.setY(curr_odo * TILE_SIZE);
          }
        } else {
          moveStraightFor(distance);
          if (travelFactorY == 0) {
            curr_odo = curr_odo + (travelFactorX * distance);
            odometer.setX(curr_odo * TILE_SIZE);
          } else {
            curr_odo = curr_odo + (travelFactorY * distance);
            odometer.setY(curr_odo * TILE_SIZE);
          }
          break;
        }
      }
    }

  }

  /**
   * Moves the robot in a half square motion (updating all odometer values) to avoid collision with a potential obstacle
   * 
   * @param travelFactorX indicates if robot is moving along x axis (-1 for -x and +1 for +x)
   * @param travelFactorY indicates if robot is moving along y axis (-1 for -y and +1 for +y)
   */
  public static double dodge(int travelFactorX, int travelFactorY) { //0 for up, 1 for right, 2 for down, 3 for left

    //    double stepFactor = 3.5 * (OBJ_DIST / 100) / TILE_SIZE; 
    //    System.out.println("dist= " + OBJ_DIST + "step= " + stepFactor);
    double stepFactor = 0;
    if (OBJ_DIST > 15) {
      stepFactor = 2.5;
    } else {
      stepFactor = 2;
    }

    double straight_odo = 0;
    double lateral_odo = 0;

    if (travelFactorY == 0) {
      straight_odo = odometer.getXyt()[0] / TILE_SIZE;
      lateral_odo = odometer.getXyt()[1] / TILE_SIZE;
    } else {
      straight_odo = odometer.getXyt()[1] / TILE_SIZE;
      lateral_odo = odometer.getXyt()[0] / TILE_SIZE;
    }

    turnBy(90);

    moveStraightFor(1);
    if (travelFactorY == 0) {
      lateral_odo = lateral_odo + (-travelFactorX);
      odometer.setY(lateral_odo * TILE_SIZE);
    } else {
      lateral_odo = lateral_odo + (-travelFactorY);
      odometer.setX(lateral_odo * TILE_SIZE);
    }

    turnBy(-90);

    Navigation.moveStraightFor(stepFactor);
    if (travelFactorY == 0) {
      straight_odo = straight_odo + (stepFactor * travelFactorX);
      odometer.setX(straight_odo * TILE_SIZE);
    } else {
      straight_odo = straight_odo + (stepFactor * travelFactorY);
      odometer.setY(straight_odo * TILE_SIZE);;
    }

    //Restore x and theta
    Navigation.turnBy(-90);

    Navigation.moveStraightFor(1);
    if (travelFactorY == 0) {
      lateral_odo = lateral_odo + (travelFactorX);
      odometer.setY(lateral_odo * TILE_SIZE);
    } else {
      lateral_odo = lateral_odo + (travelFactorY);
      odometer.setX(lateral_odo * TILE_SIZE);
    }

    Navigation.turnBy(90);

    return stepFactor;
  }

  /**
   * Method Drives robot in Y.
   */
  public static void searchInY() {

    double curr_y = odometer.getXyt()[1] / TILE_SIZE;

    if ((szr.ur.y - curr_y) < (curr_y - szr.ll.y)) {
      searchUpwards();
      searchDownwards();
    } else {
      searchDownwards();
      searchUpwards();
    }

  }
  
  /**
   * Method drives robot upwards.
   */
  public static void searchUpwards() {

    double curr_y = odometer.getXyt()[1] / TILE_SIZE;

    //Searching Upwards
    turnBy(-90);
    odometer.setTheta(0);    

    while ((szr.ur.y - curr_y) > 1) {
      if (!ObjectDetection.detect()) {
        moveStraightFor(1);
        curr_y = curr_y + 1;
        odometer.setY(curr_y * TILE_SIZE);
      } else {
        if ((szr.ur.y - (curr_y + 1) < 1)) { //Checking if obstacle is at top of column
          break;
        } else {
          ObjectDetection.avoidObjectInX(true, false);
          curr_y = odometer.getXyt()[1] / TILE_SIZE;
        }
      }
    }
  }
  
  
  /**
   * Method drives robot downwards.
   */
  public static void searchDownwards() {

    double curr_y = odometer.getXyt()[1] / TILE_SIZE;

    //Searching Downwards
    turnBy(180);
    odometer.setTheta(180);

    while ((curr_y - szr.ll.y) > 1) {
      if (!ObjectDetection.detect()) {
        moveStraightFor(1);
        curr_y = curr_y - 1;
        odometer.setY(curr_y * TILE_SIZE);
      } else {
        if (((curr_y - 1) - szr.ll.y < 1)) { //Checking if obstacle is at bottom of column
          break;
        } else {
          ObjectDetection.avoidObjectInX(false, true);
          curr_y = odometer.getXyt()[1] / TILE_SIZE;
        }
      }
    }
  }
}
