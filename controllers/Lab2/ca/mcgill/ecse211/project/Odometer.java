package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.waitUntilNextStep;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * The odometer class keeps track of the robot's (x, y, theta) position.
 *
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 * @author Younes Boubekeur
 * @author Olivier St-Martin Cormier
 */
public class Odometer implements Runnable {
  
  /** The x-axis position in meters. */
  private volatile double x;
  
  /** The y-axis position in meters. */
  private volatile double y;
  
  /** The orientation (heading) in degrees. */
  private volatile double theta;
  
  /** The instantaneous change in position (dx, dy, dTheta). */
  private static double[] deltaPosition = new double[3];

  // Thread control tools
  
  /**  Fair lock for concurrent writing. */
  private static Lock lock = new ReentrantLock(true);
  
  /** Indicates if a thread is trying to reset any position parameters. */
  private volatile boolean isResetting = false;

  /** Lets other threads know that a reset operation is over. */
  private Condition doneResetting = lock.newCondition();

  /** The singleton odometer instance. */
  private static Odometer odo;

  // Motor-related variables
  /** The previous motor tacho counts (from previous iteration of while loop). */
  private static int[] prevTacho = new int[2];
  /** The current motor tacho counts. */
  private static int[] currTacho = new int[2];
  
  private static final int LEFT = 0;
  private static final int RIGHT = 1;
  
  /** x coordinate of starting point in meters. */
  private static final double SX = 0.1524;
  /** y coordinate of starting point in meters. */
  private static final double SY = 0.1524;


  /**
   * This is the default constructor of this class. It initiates all variables once. It
   * cannot be accessed externally.
   */
  private Odometer() {
    setXyt(SX, SY, 0);
  }

  /**
   * Returns the Odometer Object. Use this method to obtain an instance of Odometer.
   *
   * @return the Odometer Object
   */
  public static synchronized Odometer getOdometer() {
    if (odo == null) {
      odo = new Odometer();
    }
    return odo;
  }

  /*
   * This method is where the logic for the odometer will run.
   */
  @Override public void run() {
    // TODO Complete the following tasks
    // Reset motor tacho counts to zero
    
    while (true) {
      // Update previous and current tacho counts (add 3 more lines)
      prevTacho[LEFT] = currTacho[LEFT];

      // TODO Implement this method below so it updates the deltaPosition
      updateDeltaPosition(prevTacho, currTacho, theta, deltaPosition);

      // Update odometer values by completing and calling the relevant method

      // Print odometer information to the console

      // Wait until the next physics step to run the next iteration of the loop
    }
  }
  
  /**
   * Updates the robot deltaPosition (x, y, theta) given the motor tacho counts.
   *
   * @param prev the previous tacho counts of the motors
   * @param curr the current tacho counts of the motors
   * @param theta the current heading (angle) of the robot
   * @param deltas the deltaPosition array (x, y, theta)
   */
  public static void updateDeltaPosition(int[] prev, int[] curr, double theta, double[] deltas) {
    theta = Math.toRadians(theta);
    double dx = 0;
    double dy = 0;
    double dtheta = 0;
    
    // TODO Calculate changes in x, y, theta based on current and previous tachometer counts:
    // Compute left and right wheel displacements
    // Compute change in heading and x and y components of displacement

    // Set deltas like this
    deltas[0] = dx;
    deltas[1] = dy;
    deltas[2] = Math.toDegrees(dtheta);
  }

  /**
   * Prints odometer information to the console.
   */
  public void printPosition() {
    lock.lock();
    // TODO
    System.out.println("Print odometer x, y, theta here"); 
    lock.unlock();
  }
  
  /**
   * Returns the Odometer data.
   *
   * <p>{@code odoData[0] = x, odoData[1] = y; odoData[2] = theta;}
   *
   * @return the odometer data.
   */
  public double[] getXyt() {
    double[] position = new double[3];
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }
      position[0] = x;
      position[1] = y;
      position[2] = theta;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return position;
  }

  /**
   * Increments the current values of the x, y and theta instance variables given deltaPositions.
   * Useful for odometry.
   */
  public void updateOdometerValues() {
    lock.lock();
    isResetting = true;
    try {
      x += deltaPosition[0];
      
      // TODO Update y and theta. Remember to keep theta within 360 degrees
      
      isResetting = false;
      doneResetting.signalAll(); // Let the other threads know we are done resetting
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overrides the values of x, y and theta. Use for odometry correction.
   *
   * @param x the value of x
   * @param y the value of y
   * @param theta the value of theta in degrees
   */
  public void setXyt(double x, double y, double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      this.y = y;
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites x. Use for odometry correction.
   *
   * @param x the value of x
   */
  public void setX(double x) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites y. Use for odometry correction.
   *
   * @param y the value of y
   */
  public void setY(double y) {
    lock.lock();
    isResetting = true;
    try {
      this.y = y;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites theta. Use for odometry correction.
   *
   * @param theta the value of theta
   */
  public void setTheta(double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

}
