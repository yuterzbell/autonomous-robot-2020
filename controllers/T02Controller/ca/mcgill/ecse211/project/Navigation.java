package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static java.lang.Math.*;
import java.util.ArrayList;
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
    boolean flagX = false;
    boolean flagY = false;
    double differenceX = destination.x - odometer.getXytInTileSize()[0];
    double differenceY = destination.y - odometer.getXytInTileSize()[1];
    if(Math.abs(differenceY) > 0.1) {flagY = true;}
    if(Math.abs(differenceX) > 0.1) {flagX = true;}
    // move in y first
    System.out.println("abssY"+Math.abs(differenceY));
    if(flagY) {
    var pointX = new Point(xyt[0] / TILE_SIZE, destination.y);
    travelTo(pointX, 0, travelY);
    }
    // move in x then
    System.out.println("abssX"+Math.abs(differenceX));
    if(flagX) {
    xyt = odometer.getXyt();
    var pointY = new Point(destination.x, xyt[1] / TILE_SIZE);
    travelTo(pointY, travelX, 0);
    }
  }

  /** Travels to the given destination. 
   * 
   * @param Point destination
   * @param int travelFactorX  1 indicating to right, -1 indicating to left
   * @param int travelFactorY  1 indicating to up, -1 indicating to bottom  
   */
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

      moveStraightWithObjectAvoidanceAndLineCorrection(distanceBetween(currentLocation, destination), travelFactorX, travelFactorY);   
  }

  /**
   * When odometer is facing 90 degrees
   */
  public static void alignToTheRightSide(){
    moveStraightFor(-0.4);
    turnBy(-90);
    moveStraightFor(0.7);
    turnBy(90);
    moveStraightFor(0.9);
    turnBy(90);
    moveStraightFor(0.7);
  }

  /**
   * When odometer is facing 270
   */
  public static void alignToTheLeftSide(){
    moveStraightFor(-0.4);
    turnBy(90);
    moveStraightFor(0.8);
    turnBy(-90);
    moveStraightFor(0.8);
    turnBy(- 90);
    moveStraightFor(0.8);
  }

    /**
   * When odometer is facing 0  
   */
  public static void alignReserve(){
    moveStraightFor(-0.3);
    turnBy(90);
    moveStraightFor(0.7);
    turnBy(-90);
    moveStraightFor(1.5);
    turnBy(- 90);
    moveStraightFor(0.7);
    turnBy(- 90);
    moveStraightFor(0.7);
  }

  public static void determineMovement(double deltaLocation[], double theta, boolean flag, double rampx, double rampy){
    //Case to move down and move left
    if(flag == true) {
      if(deltaLocation[1]<0 && deltaLocation[0]<0 ){
        System.out.println("1.1");
        if(!(theta>170 && theta<190)){        
        
          if(theta>350 || theta<10){
            alignReserve();
          }
          else if(theta>85 && theta<95){
            alignToTheRightSide();
          }
          else if(theta>260 && theta<280){
            alignToTheLeftSide();
          }
          
        }
        
        double difference = rampy - odometer.getXytInTileSize()[1];
        double travelY = difference + odometer.getXytInTileSize()[1] + 0.4;
        double travelX = odometer.getXytInTileSize()[0];
        Point goTo = new Point(travelX, travelY);
        STORE_TORQUE = true;
        navigateTo(goTo);
        STORE_TORQUE = false;
        calculateMass();
        alignToTheRightSide();
        
        double difference2 = rampx - odometer.getXytInTileSize()[0];
        double travelY2 = odometer.getXytInTileSize()[1];
        double travelX2 = difference2 + odometer.getXytInTileSize()[0] + 0.4;
        Point goTo2 = new Point(travelX2, travelY2);
        navigateTo(goTo2);
        moveRobotBackwardsFromRamp();
        
        }
        
      
      //Case to move down and move RIGHT
      if(deltaLocation[1]<0 && deltaLocation[0]>0 ){
        System.out.println("2.1");
        
        if(!(theta>170 && theta<190)) {
          
          if(theta>260 && theta < 280){
            alignToTheLeftSide();
        
          }
          else if(theta>350 || theta<10){
            alignReserve();
         
          }
          else if(theta>85 && theta<95){
            alignToTheRightSide();
            
            }
          }
        double difference = rampy - odometer.getXytInTileSize()[1];
        double travelY = difference + odometer.getXytInTileSize()[1] + 0.4;
        double travelX = odometer.getXytInTileSize()[0];
        Point goTo = new Point(travelX, travelY);
        STORE_TORQUE = true;
        navigateTo(goTo);
        STORE_TORQUE = false;
        calculateMass();
        alignToTheLeftSide();
        
        double difference2 = rampx - odometer.getXytInTileSize()[0];
        double travelY2 = odometer.getXytInTileSize()[1];
        double travelX2 = difference2 + odometer.getXytInTileSize()[0] - 0.4;
        Point goTo2 = new Point(travelX2, travelY2);
        navigateTo(goTo2);
        moveRobotBackwardsFromRamp();
        
        }
  
       

        //Case to move up and move RIGHT
      if(deltaLocation[1]>0 && deltaLocation[0]>0 ){
        System.out.println("3.1");
        if(!(theta>350 || theta<10)){
          if(theta>85 && theta < 100){
            alignToTheLeftSide();
            
          }
          else if(theta>170 && theta<190){
            alignReserve();
            
          }
          else if(theta>260 || theta<280){
            alignToTheRightSide();
            
          }
          
        }
        double difference = rampy - odometer.getXytInTileSize()[1];
        double travelY = difference + odometer.getXytInTileSize()[1] - 0.4;
        double travelX = odometer.getXytInTileSize()[0];
        Point goTo = new Point(travelX, travelY);
        STORE_TORQUE = true;
        navigateTo(goTo);
        STORE_TORQUE = false;
        calculateMass();
        alignToTheRightSide();
        
        double difference2 = rampx - odometer.getXytInTileSize()[0];
        double travelY2 = odometer.getXytInTileSize()[1];
        double travelX2 = difference2 + odometer.getXytInTileSize()[0] - 0.4;
        Point goTo2 = new Point(travelX2, travelY2);
        navigateTo(goTo2);
        moveRobotBackwardsFromRamp();
       }

      if(deltaLocation[1]>0 && deltaLocation[0]<0 ){
        System.out.println("4.1");
        if(!(theta>350 || theta<10)){
          if(theta>85 && theta < 100){
            alignToTheLeftSide();
          
          }
          else if(theta>170 && theta<190){
            alignReserve();
            
          }
          else if(theta>350 || theta<10){
            alignToTheRightSide();
            
          }
          
        
        }
        double difference = rampy - odometer.getXytInTileSize()[1];
        double travelY = difference + odometer.getXytInTileSize()[1] - 0.4;
        double travelX = odometer.getXytInTileSize()[0];
        Point goTo = new Point(travelX, travelY);
        STORE_TORQUE = true;
        navigateTo(goTo);
        STORE_TORQUE = false;
        calculateMass();
        alignToTheLeftSide();
        
        double difference2 = rampx - odometer.getXytInTileSize()[0];
        double travelY2 = odometer.getXytInTileSize()[1];
        double travelX2 = difference2 + odometer.getXytInTileSize()[0] + 0.4;
        Point goTo2 = new Point(travelX2, travelY2);
        navigateTo(goTo2);
        moveRobotBackwardsFromRamp();
        
       }
      
      
      
      
    }else {
      
    
    if(deltaLocation[1]<0 && deltaLocation[0]<0 ){
      System.out.println("1");
      if(!(theta>260 && theta<280)){
        if(theta>350 || theta<10){
          alignToTheLeftSide();
        }
        else if(theta>85 && theta<95){
          alignReserve();
        }
        else if(theta>175 && theta<185){
          alignToTheRightSide();
        }
      }
      double difference = rampx - odometer.getXytInTileSize()[0];
      double travelX = difference + odometer.getXytInTileSize()[0] + 0.4;
      double travelY = odometer.getXytInTileSize()[1];
      Point goTo = new Point(travelX, travelY);
      STORE_TORQUE = true;
      navigateTo(goTo);
      STORE_TORQUE = false;
      calculateMass();
      alignToTheLeftSide();
      
      double difference2 = rampy - odometer.getXytInTileSize()[1];
      double travelY2 = difference2 + odometer.getXytInTileSize()[1] + 0.4;
      double travelX2 = odometer.getXytInTileSize()[0];
      Point goTo2 = new Point(travelX2, travelY2);
      navigateTo(goTo2);
      moveRobotBackwardsFromRamp();
     }

    //Case to move down and move RIGHT
    if(deltaLocation[1]<0 && deltaLocation[0]>0 ){
      System.out.println("2");
      if(!(theta>85 && theta<95)){
        if(theta>260 && theta < 280){
          alignReserve();
        }
        else if(theta>350 || theta<10){
          alignToTheRightSide();
        }
        else if(theta>175 && theta<185){
          alignToTheLeftSide();
        }
      }
      double difference = rampx - odometer.getXytInTileSize()[0];
      double travelX = difference + odometer.getXytInTileSize()[0] - 0.4;
      double travelY = odometer.getXytInTileSize()[1];
      Point goTo = new Point(travelX, travelY);
      STORE_TORQUE = true;
      navigateTo(goTo);
      STORE_TORQUE = false;
      calculateMass();
      alignToTheRightSide();
      
      double difference2 = rampy - odometer.getXytInTileSize()[1];
      double travelY2 = difference2 + odometer.getXytInTileSize()[1] + 0.4;
      double travelX2 = odometer.getXytInTileSize()[0];
      Point goTo2 = new Point(travelX2, travelY2);
      navigateTo(goTo2);
      moveRobotBackwardsFromRamp();
     }

      //Case to move up and move RIGHT
    if(deltaLocation[1]>0 && deltaLocation[0]>0 ){
      System.out.println("3");
      if(!(theta<80 && theta>100)){
        if(theta>260 && theta < 280){
          alignReserve();
        }
        else if(theta>170 && theta<190){
          alignToTheLeftSide();
        }
        else if(theta>350 || theta<10){
          alignToTheRightSide();
        }
      }
        double difference = rampx - odometer.getXytInTileSize()[0];
        double travelX = difference + odometer.getXytInTileSize()[0] - 0.4; 
        double travelY = odometer.getXytInTileSize()[1];
        Point goTo = new Point(travelX, travelY);
        STORE_TORQUE = true;
        navigateTo(goTo);
        STORE_TORQUE = false;
        calculateMass();
        alignToTheLeftSide();
        
        double difference2 = rampy - odometer.getXytInTileSize()[1];
        double travelY2 = difference2 + odometer.getXytInTileSize()[1] - 0.4;
        double travelX2 = odometer.getXytInTileSize()[0];
        System.out.println("nextY:"+travelY2);
        Point goTo2 = new Point(travelX2, travelY2);
        navigateTo(goTo2);
        moveRobotBackwardsFromRamp();

     }


     //Case to move up and move RIGHT
    if(deltaLocation[1]>0 && deltaLocation[0]<0 ){
      System.out.println("4");
      if(!(theta>260 && theta < 280)){
        if(theta>350 || theta<10){
          alignToTheLeftSide();
        }
        else if(theta>170 && theta<190){
          alignToTheRightSide();
        }
        else if(theta>80 && theta<100){
          alignReserve();
        }
      }
      
      double difference = rampx - odometer.getXytInTileSize()[0];
      double travelX = difference + odometer.getXytInTileSize()[0] + 0.4;
      double travelY = odometer.getXytInTileSize()[1];
      Point goTo = new Point(travelX, travelY);
      STORE_TORQUE = true;
      navigateTo(goTo);
      STORE_TORQUE = false;
      calculateMass();
      alignToTheRightSide();
      
      double difference2 = rampy - odometer.getXytInTileSize()[1];
      double travelY2 = difference2 + odometer.getXytInTileSize()[1] - 0.4;
      double travelX2 = odometer.getXytInTileSize()[0];
      Point goTo2 = new Point(travelX2, travelY2);
      navigateTo(goTo2);
      moveRobotBackwardsFromRamp();
     }
    
    }
    
  }

  /** Navigates the robot to the ramp. */
  public static void goRamp() {
    
    Point left = Ramp.left;
    Point right = Ramp.right;
    double heading = 0;
    var xyt = odometer.getXyt();
    double xRamp = (left.x + right.x)/2;
    double yRamp = (left.y + right.y)/2;
    var USReadingsLower = readUsDistance();
    var USReadingsUpper = readUsDistance2();
    double blockLocation[] = new double[2];
    boolean onSide = false;
    xyt = odometer.getXyt();
 //   if(USReadingsLower<15 && USReadingsUpper > 25){
      blockLocation[0] = odometer.getXytInTileSize()[0];
      blockLocation[1] = odometer.getXytInTileSize()[1];
  //  }
   
    // case 270 - up towards the left
    if(left.y < right.y && left.x == right.x) {
      onSide = true;
    }
    
    if(left.y > right.y && left.x == right.x) {
     onSide = true;
    }
    
    System.out.println(onSide);
    
    System.out.println("yramp"+yRamp);
    System.out.println(blockLocation[1]);
    double deltaLocation[] = {xRamp - blockLocation[0],yRamp - blockLocation[1]};
    System.out.println(deltaLocation[0]);
    System.out.println(deltaLocation[1]);
    determineMovement(deltaLocation, xyt[2], onSide, xRamp, yRamp);


    // // case 270 - up towards the left
    // if(left.y < right.y && left.x == right.x) {
    //   System.out.println("1");
    //   xRamp = xRamp +1;
    //   Point ramp = new Point(xRamp, yRamp);
    //   navigateTo(ramp);
    //   turnTo(270);
    //   //moveRobotBackwardsFromRamp();

    // }
    // // case 90 - up on the right
    // if(left.y > right.y && left.x == right.x) {
    //   System.out.println("2");
    //   xRamp = xRamp -1;
    //   Point ramp = new Point(xRamp, yRamp);
    //   navigateTo(ramp);
    //   turnTo(90);
    //   // moveRobotBackwardsFromRamp();
    // }

    // // case 0 - going up north
    // if(left.x < right.x && left.y == right.y) {
    //   System.out.println("3");
    //   yRamp = yRamp -1;
    //   System.out.println(xRamp);
    //   Point ramp = new Point(xRamp, yRamp);
    //   navigateTo(ramp);
    //   turnTo(0);
    //   //moveRobotBackwardsFromRamp();

    // }
    // // case 180 - going down 
    // if(left.x > right.x && left.y == right.y) {
    //   System.out.println("4");
    //   yRamp = yRamp +1;
    //   Point ramp = new Point(xRamp, yRamp);
    //   navigateTo(ramp);
    //   turnTo(180);
    //   // moveRobotBackwardsFromRamp();
    // }

  }

  /**
   * Moves the robot backwards one tile after dropping the container in the bin
   */
  /*
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
  */
  /** Navigates the robot to the ramp. */
  /*
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




  }*/

  /**
   * Moves the robot backwards one tile after dropping the container in the bin
   */
  public static void moveRobotBackwardsFromRamp(){
  

    moveStraightFor(1.2);

    moveStraightFor(-1.5);
    /*
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
      
    } */
    
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
  public static void moveStraightWithObjectAvoidanceAndLineCorrection(double distance, int travelFactorX, int travelFactorY) {
    
    if (AVOID_FLAG) {
      double curr_odo = 0;
      if (travelFactorY == 0) {
        curr_odo = odometer.getXyt()[0] / TILE_SIZE;
      } else {
        curr_odo = odometer.getXyt()[1] / TILE_SIZE;
      }
  
      double stepFactor = 0;
  
      while (distance != 0) {
        if (ObjectDetection.obstacleDetect(10)) {
          if (distance > TILE_SIZE) {
            stepFactor = dodge(travelFactorX, travelFactorY);
            if (stepFactor == -99) {
              System.out.println("Fatal Error!");
              return;
            }
            if (travelFactorY == 0) {
              curr_odo = odometer.getXyt()[0] / TILE_SIZE;
            } else {
              curr_odo = odometer.getXyt()[1] / TILE_SIZE;
            }
          } else {
            System.out.println("test");
            distance = 0;
          }
          distance = distance - stepFactor;
        } else {
          if (distance > 0.2) {
            //moveStraightFor(1);
            ColorDetection.moveStraightWithLineCorrectionAndWaterDetection(Math.abs(travelFactorX), 0.2);
            if (travelFactorY == 0) {
              curr_odo = curr_odo + (travelFactorX * 0.2);
              distance = distance - 0.2;
              odometer.setX(curr_odo * TILE_SIZE);
            } else {
              curr_odo = curr_odo + (travelFactorY * 0.2);
              distance = distance - 0.2;
              odometer.setY(curr_odo * TILE_SIZE);
            }
          } else {
            //moveStraightFor(distance);
            ColorDetection.moveStraightWithLineCorrectionAndWaterDetection(Math.abs(travelFactorX), distance);
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
    } else {
      ColorDetection.moveStraightWithLineCorrectionAndWaterDetection(Math.abs(travelFactorX), distance);
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
    
    int dodgeFactor = 0;
    while (dodgeFactor == 0) {
      turnBy(90);
      if (!ObjectDetection.obstacleDetect(20)) {
        dodgeFactor = 1; //Cut to left
        turnBy(-90);
        break;
      } else {
        turnBy(-180);
        if (!ObjectDetection.obstacleDetect(20)) {
          dodgeFactor = -1; //Cut to right
          turnBy(90);
          break;
        } 
        turnBy(90);
      }
      turnBy(180);
      moveStraightFor(0.2);
      if (travelFactorY == 0) {
        straight_odo = straight_odo + (stepFactor * travelFactorX);
        odometer.setX(straight_odo * TILE_SIZE);
      } else {
        straight_odo = straight_odo + (stepFactor * travelFactorY);
        odometer.setY(straight_odo * TILE_SIZE);;
      }
      if (ObjectDetection.obstacleDetect(10)) {
        return -99;
      }
      turnBy(-180);
      stepFactor++;
    }
    

    turnBy(dodgeFactor*90);

    //moveStraightFor(1);
    ColorDetection.moveStraightWithLineCorrectionAndWaterDetection(Math.abs(travelFactorY), 1);
    if (travelFactorY == 0) {
      lateral_odo = lateral_odo + (-travelFactorX);
      odometer.setY(lateral_odo * TILE_SIZE);
    } else {
      lateral_odo = lateral_odo + (-travelFactorY);
      odometer.setX(lateral_odo * TILE_SIZE);
    }

    turnBy(dodgeFactor*-90);

    //moveStraightFor(stepFactor);
    ColorDetection.moveStraightWithLineCorrectionAndWaterDetection(Math.abs(travelFactorX), stepFactor);
    if (travelFactorY == 0) {
      straight_odo = straight_odo + (stepFactor * travelFactorX);
      odometer.setX(straight_odo * TILE_SIZE);
    } else {
      straight_odo = straight_odo + (stepFactor * travelFactorY);
      odometer.setY(straight_odo * TILE_SIZE);;
    }

    //Restore x and theta
    turnBy(dodgeFactor*-90);

    //moveStraightFor(1);
    ColorDetection.moveStraightWithLineCorrectionAndWaterDetection(Math.abs(travelFactorX), 1);
    if (travelFactorY == 0) {
      lateral_odo = lateral_odo + (travelFactorX);
      odometer.setY(lateral_odo * TILE_SIZE);
    } else {
      lateral_odo = lateral_odo + (travelFactorY);
      odometer.setX(lateral_odo * TILE_SIZE);
    }

    turnBy(dodgeFactor*90);

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
  

/** Method calculates the mass of the container*/
  
  public static void calculateMass() {
    
    double mass =0;
    double averageTorque = 0;
    double sum = 0;
    double count = 0;
    int floor = (int) Math.round(allTorque.size()*0.3);
    int sealing = (int) Math.round(allTorque.size()*0.8);
    for (int i = floor; i <sealing; i++) {
      //System.out.println(allTorque.get(i));
      sum += allTorque.get(i);
      count++;
    }

    averageTorque = sum / count;
    
   // System.out.println("sum"+sum);
   // System.out.println("count"+count);
    System.out.println(averageTorque);
    
    
    
    if(averageTorque < 0.18) {
      mass = 1.0;
      System.out.println("Container with weight "+mass+"kg identified (+2points)");
      
    }
    
    if(averageTorque < 0.22 && averageTorque > 0.18) {
      mass = 2.0;
      System.out.println("Container with weight "+mass+"kg identified (+3points)");
    }
    
    if(averageTorque < 0.26 && averageTorque > 0.22) {
      mass = 3.0;
      System.out.println("Container with weight "+mass+"kg identified (+4points)");
    }
    
    if(averageTorque > 0.26) {
      mass = 3.0;
      System.out.println("Container with weight "+mass+"kg identified (+4points)");
    }
    allTorque.clear();
    
  }
}
