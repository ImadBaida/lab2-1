package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.NUMBER_OF_THREADS;
import static ca.mcgill.ecse211.project.Resources.SQUARE_LENGTH;
import static ca.mcgill.ecse211.project.Resources.odometer;

import simlejos.ExecutionController;


/**
 * Main class of the program.
 */
public class Main {

  /**
   * Main entry point.
   */
  public static void main(String[] args) {
    // We are going to start one extra thread, so the total number of parties is 2
    ExecutionController.setNumberOfParties(NUMBER_OF_THREADS);
    ExecutionController.performPhysicsStepsInBackground();
    
    //start the odometer thread
    Thread odo = new Thread(odometer);
    odo.start();
    
    SquareDriver.driveInASquare(SQUARE_LENGTH);

    
    
    System.exit(0);
  }

}
