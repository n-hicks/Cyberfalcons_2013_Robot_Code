/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

/**
 * Property of CyberFalcons Robotics
 */
public class CyberFalcons2013 extends IterativeRobot {

    /* CONSTANTS */
    /**
     * Deadzone tolerance for the joysticks. The keyword 'final' means that this
     * variable can not be assigned to (changed) elsewhere in the code.
     */
    final double DEADZONE = 0.1;
    final double flyWheelAcceleration = -0.04; // must be a division of 1;   Current cycles to top speed = 25
    final double normalShotSpeed = -1.0; // value for changing shooter wheel speed for non-feeder shots
    final double feederShotSpeed = -1.0; // value for changing shooter wheel speed for feeder shots
    final int closeShotValue = 455; // value from loftPot for successful closer shot  
    final int farShotValue = 545;//568; // value from loftPot for successful farther shot  
    final int angleTolerance = 15; // value that loftPot returns may be +- before focus
    final int focusTolerance = 3; // value that loftPot returns may be +- after focus
    final int maxHeightValue = 390; // control value to stop loft
    final int minHeightValue = 630; // control value to stop loft
    final int maxArmValue = 1000; // control value to stop arm
    final int minArmValue = -1000; // control value to stop arm
    final int climbStopValue = 1000;//100; // armPot value to stop winch movement when climbing
    final int shooterFeedingValue = 495; // ideal value for feeding disks
    final int armSlotFeedingValue = 310; // ideal value for feeding disks from slot
    final int armShovelFeedingValue = 87; // ideal value for feeding disks from shovel
    /*
     *   VALUES FROM TESTING
     * arm feeding: 310     from slot
     * arm loading: 87      with shovel
     * arm hang: 345
     * lawyer hang: 108
     * raw tested hang = 30 degree arm to frame           FIND POT ANGLE
     * shooter feeding: 495
     */
    /* Electronic/Controller Variables */
    // Xbox controllers
    XBoxController xboxDriver;
    XBoxController xboxOperator;
    // Drive motors
    Victor vicDriveRight;
    Victor vicDriveLeft;
    // Shooter Motors
    Relay bettyRelay;
    Victor shooterWheel;
    Victor shooterLoft;
    // Betty sensors
    DigitalInput bettyOpen;
    // Loft Sensors
//    DigitalInput loftUpperStop;
//    DigitalInput loftLowerStop;
    AnalogChannel loftPot;
    // Arm Motors
    Victor armVictor;
    Relay winchRelay;
    // Arm Sensors
    AnalogChannel armPot;
    // Lights
    DigitalOutput lightsOutput1;
    DigitalOutput lightsOutput2;
    DigitalOutput lightsOutput3;
    DigitalInput lightSwitch1;
    Timer lightsTimer;
    /* Operation Variables */
    boolean teleopActive = false;
    boolean controlFlip = true;
    boolean controlFlipClean = true;
    boolean climbMode = false;
    double shootingSpeed = 0.0;
    int starterDelay = 0;
    boolean canShoot = false;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        // Xbox Controllers
        xboxDriver = new XBoxController(1); // USB port 1
        xboxOperator = new XBoxController(2); // USB port 2

        // Drive Motors
        vicDriveRight = new Victor(/*cRIO slot*/1, /*PWM channel*/ 1);
        vicDriveLeft = new Victor(/*cRIO slot*/1, /*PWM channel*/ 2);

        // Shooter Motors
        bettyRelay = new Relay(/*cRIO slot*/1, /*PWM channel*/ 1, Relay.Direction.kBoth);
        shooterWheel = new Victor(/*cRIO slot*/1, /*PWM channel*/ 3);
        shooterLoft = new Victor(/*cRIO slot*/1, /*PWM channel*/ 4);

        // Betty Sensors
        bettyOpen = new DigitalInput(/*cRIO slot*/1,/*IO channel*/ 10);

        // Arm Motors
        armVictor = new Victor(/*cRIO slot*/1, /*PWM channel*/ 5);
        winchRelay = new Relay(/*cRIO slot*/1, /*PWM channel*/ 3, Relay.Direction.kBoth);

        // Loft Sensors
//        loftUpperStop       = new DigitalInput (/*cRIO slot*/ 1,/*IO channel*/ 3);
//        loftLowerStop       = new DigitalInput (/*cRIO slot*/ 1,/*IO channel*/ 4);
        loftPot = new AnalogChannel(/*cRIO slot*/1,/*Analog channel*/ 2);

        // Arm Sensors
        armPot = new AnalogChannel(/*cRIO slot*/1,/*Analog channel*/ 4);

        // Lights
        lightsOutput1 = new DigitalOutput(/*cRIO slot*/1,/*IO channel*/ 11);
        lightsOutput2 = new DigitalOutput(/*cRIO slot*/1,/*IO channel*/ 12);
        lightsOutput3 = new DigitalOutput(/*cRIO slot*/1,/*IO channel*/ 13);
        lightSwitch1 = new DigitalInput(/*cRIO slot*/1,/*IO channel*/ 6);
        lightsTimer = new Timer();
    }

    public void autonomousInit() {
        lightsOutput1.set(true);
        lightsOutput2.set(true);
        lightsOutput3.set(false);

        canShoot = false;

        vicDriveRight.set(0);
        vicDriveLeft.set(0);
        armVictor.set(0);
        winchRelay.set(Relay.Value.kOff);
        shooterWheel.set(0);
        shooterLoft.set(0);
        bettyRelay.set(Relay.Value.kOff);
        starterDelay = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
//        int loftPotValue = loftPot.getValue();
//        // ajust for far shot
//        if (!canShoot) {
//            if (loftPotValue < farShotValue - 8) {
//                starterDelay = 0;
//                shooterLoft.set(-0.5);
//
//            }
//            else if (loftPotValue > farShotValue + 8) {
//                starterDelay = 0;
//                shooterLoft.set(0.5);
//
//            }
//            else { // if ajusted, shoot
//                canShoot = true;
//                shooterLoft.set(0);
//            }
//        }
//        if(canShoot) {
        if (shootingSpeed > normalShotSpeed) {  // accelerating shooter speed until top speed(1.0)
            shootingSpeed = shootingSpeed + flyWheelAcceleration;
        } else {
            shootingSpeed = normalShotSpeed; // ensures desired value and no higher
        }
        shooterWheel.set(shootingSpeed);
        if (shootingSpeed == normalShotSpeed) {
            starterDelay++;
            if (starterDelay > 60) {
                bettyRelay.setDirection(Relay.Direction.kForward);
                bettyRelay.set(Relay.Value.kOn);
            } else if (starterDelay > 310) { //kill flywheel and betty
                bettyRelay.set(Relay.Value.kOff);
                shooterWheel.set(0);
            } else {
                bettyRelay.set(Relay.Value.kOff);
            }
        }
//        }

    }

    /**
     * Called every time tele-op mode starts
     */
    public void teleopInit() {
        vicDriveRight.set(0);
        vicDriveLeft.set(0);
        armVictor.set(0);
        winchRelay.set(Relay.Value.kOff);
        shooterWheel.set(0);
        shooterLoft.set(0);
        bettyRelay.set(Relay.Value.kOff);
        lightsOutput1.set(true);
        lightsOutput2.set(true);
        lightsOutput3.set(true);
        lightsTimer.reset();
        lightsTimer.start();

        teleopActive = false;
        controlFlip = true;
        controlFlipClean = true;
        shootingSpeed = 0.0;
        starterDelay = 0;
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Watchdog.getInstance().feed(); // Tell watchdog we are running

        // activate the robot by pushing start button
        if (xboxDriver.getBtnSTART()) {
            teleopActive = true;
        }
        if (!teleopActive) {
            return;         //only run if teleop is active
        }
        if (lightsTimer.get() > 90000) {
            lightsOutput1.set(true);
            lightsOutput2.set(true);
            lightsOutput3.set(false);
        }

        drive();
        shooterUse();
//        shooterLoft();
        armUse();

    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        Watchdog.getInstance().feed(); // Tell watchdog we are running
        drive();
        shooterUse();
//        shooterLoft();
        armUse();
        System.out.println("loftPot value: " + loftPot.getValue() + "       armPot Value: " + armPot.getValue());
        if (xboxDriver.getBtnBACK()) { // when driver controls, hold button BACK to unwind winch, release to stop, only for testing
            winchRelay.set(Relay.Value.kOn);
            winchRelay.setDirection(Relay.Direction.kForward);
        }
    }

    public void drive() {
        //Driving controls
            /* flips which direction on the joysticks is 'forward'
         * the variable 'controlFlipClean' needs some explanation:
         * it forces the driver to release the button before it recognizes
         * another click. */
        if (xboxDriver.getBtnL3()) {    // left stick click
            if (controlFlipClean) {
                controlFlipClean = false;
                controlFlip = !controlFlip;
            }
        } else {
            controlFlipClean = true;
        }

        //controls right side
        double yRight = -xboxDriver.getRightY();
        //controls left side
        double yLeft = xboxDriver.getLeftY();

        //trigger values will change the maximum speed of the robot
        double throttle = xboxDriver.getTriggers();

        //half speed
        if (throttle > 0.5) {
            yRight = yRight * 0.5;
            yLeft = yLeft * 0.5;
        }
        //quarter speed
        if (throttle < -0.5) {
            yRight = yRight * 0.25;
            yLeft = yLeft * 0.25;
        }

        // flip controls if we need to. Make them go 'backwards'.
        // We need to swap and yRight and yLeft values.
        if (controlFlip) {
            double tempRight = yRight;
            yRight = yLeft;
            yLeft = tempRight;
        }

        /* DEADZONE **
         * The springs in the joysticks wear out so they don't always come
         * back to the centre. To fix this we're going to ignore a small region
         * around the centre.
         */
        if (yRight > -DEADZONE && yRight < DEADZONE) {
            yRight = 0;
        }
        if (yLeft > -DEADZONE && yLeft < DEADZONE) {
            yLeft = 0;
        }


        vicDriveRight.set(yRight);
        vicDriveLeft.set(yLeft);
    } // end of drive

    public void shooterUse() {
        // Betty operation
        if (xboxOperator.getBtnLB()) {    //hold right bumper to fire 
            starterDelay++;
            if (xboxOperator.getTriggers() < -0.5) { // feeder shot
                if (shootingSpeed > feederShotSpeed) {  // accelerating shooter speed until top speed
                    shootingSpeed = shootingSpeed + flyWheelAcceleration;
                } else {
                    shootingSpeed = feederShotSpeed; // ensures desired value
                }
            } else { // normal shot
                if (shootingSpeed > normalShotSpeed) {  // accelerating shooter speed until top speed
                    shootingSpeed = shootingSpeed + flyWheelAcceleration;
                } else {
                    shootingSpeed = normalShotSpeed; // ensures desired value
                }
            }
            shooterWheel.set(shootingSpeed);
            if (xboxOperator.getBtnRB() && starterDelay > 60) { // shoot normal shot
                bettyRelay.setDirection(Relay.Direction.kForward);
                bettyRelay.set(Relay.Value.kOn);
            } else if (xboxOperator.getBtnB() && starterDelay > 60) { // shoot normal shot
                bettyRelay.setDirection(Relay.Direction.kReverse);
                bettyRelay.set(Relay.Value.kOn);
            } else {
                bettyRelay.set(Relay.Value.kOff);
            }

        } else if (bettyOpen.get()) { // close betty if not shooting
            shootingSpeed = 0.0;
            bettyRelay.setDirection(Relay.Direction.kForward);
            bettyRelay.set(Relay.Value.kOn);
            shooterWheel.set(0);
            starterDelay = 0;
        } else {
            shootingSpeed = 0.0;
            bettyRelay.set(Relay.Value.kOff);
            shooterWheel.set(0);
            starterDelay = 0;
        }
    } // end of shooter use

    public void shooterLoft() {
        //Ajust loft
        double operatorLeftY = xboxOperator.getLeftY(); // operator has loft control
        int loftPotValue = loftPot.getValue();
        if (xboxOperator.getBtnX()) { // hold to ajust for close shot
            if (loftPotValue < closeShotValue - focusTolerance) {
                if (loftPotValue < closeShotValue - angleTolerance) {
                    shooterLoft.set(-1);
                } else {
                    shooterLoft.set(-0.001);
                }
            } else if (loftPotValue > closeShotValue + focusTolerance) {
                if (loftPotValue > closeShotValue + angleTolerance) {
                    shooterLoft.set(1);
                } else {
                    shooterLoft.set(0.001);
                }
            } else {
                shooterLoft.set(0);
            }
        } else if (xboxOperator.getBtnY()) { // hold to ajust for far shot
            if (loftPotValue < farShotValue - focusTolerance) {
                if (loftPotValue < farShotValue - angleTolerance) {
                    shooterLoft.set(-1);
                } else {
                    shooterLoft.set(-0.001);
                }
            } else if (loftPotValue > farShotValue + focusTolerance) {
                if (loftPotValue > farShotValue + angleTolerance) {
                    shooterLoft.set(1);
                } else {
                    shooterLoft.set(0.001);
                }
            } else {
                shooterLoft.set(0);
            }
        } else if (xboxOperator.getBtnB()) { // hold to ajust for feeding
            if (loftPotValue < shooterFeedingValue - focusTolerance) {
                if (loftPotValue < shooterFeedingValue - angleTolerance) {
                    shooterLoft.set(-1);
                } else {
                    shooterLoft.set(-0.001);
                }
            } else if (loftPotValue > shooterFeedingValue + focusTolerance) {
                if (loftPotValue > shooterFeedingValue + angleTolerance) {
                    shooterLoft.set(1);
                } else {
                    shooterLoft.set(0.001);
                }
            } else {
                shooterLoft.set(0);
            }
        } else if (loftPotValue > maxHeightValue && loftPotValue < minHeightValue && ((operatorLeftY > .3) || operatorLeftY < -.3)) { //move loft with left operator stick
            shooterLoft.set(operatorLeftY);
        } else {
            shooterLoft.set(0);//don't move loft
        }
    } //end of shooter loft

    public void armUse() {
        //Arm contols
        int armPotValue = armPot.getValue();
        if (xboxDriver.getBtnB()) { // arm for slot feeding
            if (armPotValue < armSlotFeedingValue - angleTolerance) {
                armVictor.set(-.2);
            } else if (armPotValue < armSlotFeedingValue - focusTolerance) {
                armVictor.set(-.02);
            } else if (armPotValue > armSlotFeedingValue + angleTolerance) {
                armVictor.set(.5);
            } else if (armPotValue > armSlotFeedingValue + focusTolerance) {
                armVictor.set(.05);
            } else {
                armVictor.set(0);
            }
        } else if (xboxDriver.getBtnY()) { // arm for shovel feeding
            if (armPotValue < armShovelFeedingValue - angleTolerance) {
                armVictor.set(-.2);
            } else if (armPotValue < armShovelFeedingValue - focusTolerance) {
                armVictor.set(-.02);
            } else if (armPotValue > armShovelFeedingValue + angleTolerance) {
                armVictor.set(.5);
            } else if (armPotValue > armShovelFeedingValue + focusTolerance) {
                armVictor.set(.05);
            } else {
                armVictor.set(0);
            }
        } else {
            if (xboxDriver.getBtnLB() && armPotValue > minArmValue) {
                armVictor.set(-.2);  // driver left bumper = down, right bumper = up;
            } else if (xboxDriver.getBtnRB() && armPotValue < maxArmValue) {
                armVictor.set(.5);
            } else {
                armVictor.set(0);
            }
            if (xboxDriver.getBtnA() && armPotValue < climbStopValue) { // when driver controls, hold button A to use winch, release to stop
                winchRelay.set(Relay.Value.kOn);
                winchRelay.setDirection(Relay.Direction.kReverse);
            } else {
                winchRelay.set(Relay.Value.kOff);
            }
        }
    }  // end of arm use
} // End of class; should not be end of a function