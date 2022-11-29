package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * MainAutonomous.java
 *
 *
 * A Linear OpMode class to be an autonomous method for both Blue & Red alliance sides where we pick
 * which side of the alliance bridge we start off at with gamepad1 as well as selecting alliance
 * color and whether we park under the alliance bridge closer or further from the game field wall.
 * Also will detect the position and deliver the skystone using machine vision and move the
 * foundation.
 *
 * Mec_Odo_AutonomousLevel6_Statey is written to use machine vision and SkyStone delivery to our
 * autonomous route with the help intake jaws that suck in a stone at any orientation using a
 * "touch it-own it" approach.  A servo and two motors make up TC-73/Bucky's arm and stack stones as
 * well as our team marker.

 * This autonomous is used for our State Championship(February 7-8, 2020).
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back.
 */

@Autonomous(name="MainAutonomous", group="CatAuto")

public class MainAutonomous extends LinearOpMode {

    /* Declare OpMode members. */

    CatHW_Async robot = new CatHW_Async();    // All the hardware classes init here.
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;

    private ElapsedTime runningTime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        robot.init(hardwareMap, this);



        /*
        Init Delay Option Select:
         */

        // After init is pushed but before Start we can change the delay using dpad up/down //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive() && !isStopRequested()) {
            if (this.isStopRequested()) {
                // Leave the loop if STOP is pressed
                return;
            }
            if (gamepad1.dpad_up && (delayTimer.seconds() > 0.8)) {
                // Increases the amount of time we wait
                timeDelay += 1;
                delayTimer.reset();
            }
            if (gamepad1.dpad_down && (delayTimer.seconds() > 0.8)) {
                // Decreases the amount of time we wait
                if (timeDelay > 0) {
                    // No such thing as negative time
                    timeDelay -= 1;
                }
                delayTimer.reset();
            }
            if (((gamepad1.x) && delayTimer.seconds() > 0.5)) {
                // Changes Alliance Sides
                if (robot.isRedAlliance && !robot.isLeftAlliance) {

                    robot.isRedAlliance = true;
                    robot.isLeftAlliance = true;

                } else if (robot.isRedAlliance && robot.isLeftAlliance) {

                    robot.isLeftAlliance = true;
                    robot.isRedAlliance = false;
                } else if (!robot.isRedAlliance && robot.isLeftAlliance) {

                    robot.isLeftAlliance = false;
                    robot.isRedAlliance = false;
                } else if (!robot.isRedAlliance && !robot.isLeftAlliance) {

                    robot.isLeftAlliance = false;
                    robot.isRedAlliance = true;
                }
                delayTimer.reset();
            }





            /*
             * LED code:
             */
            if (CatHW_Async.isRedAlliance && !CatHW_Async.isLeftAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if(CatHW_Async.isRedAlliance && CatHW_Async.isLeftAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            }else if(!CatHW_Async.isRedAlliance && !CatHW_Async.isLeftAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }else if(!CatHW_Async.isRedAlliance && CatHW_Async.isLeftAlliance){
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            }



            /*
             * Telemetry while waiting for PLAY:
             */
            robot.drive.updateOdo();
            telemetry.addData("Pos","%.3f %.3f %.3f",robot.drive.realSense.getXPos(),robot.drive.realSense.getYPos(), robot.drive.realSense.getRotation());
            dashboardTelemetry.addData("Analysis Red", robot.eyes.pipeline.avgRed);
            dashboardTelemetry.addData("Analysis Blue", robot.eyes.pipeline.avgBlue);
            dashboardTelemetry.addData("Analysis Green", robot.eyes.pipeline.avgGreen);
            dashboardTelemetry.addData("Position", robot.eyes.pipeline.avgValue);
            telemetry.addData("Position", robot.eyes.pipeline.avgValue);
            telemetry.addData("POS ","Is Left:%s", robot.isLeftAlliance);
            if(robot.isLeftAlliance && robot.isRedAlliance){
                telemetry.addData("Alliance","Red, Left");
            }else if(!robot.isLeftAlliance && robot.isRedAlliance){
                telemetry.addData("Alliance","Red, Right");
            }else if(robot.isLeftAlliance && !robot.isRedAlliance){
                telemetry.addData("Alliance","Blue, Left");
            }else if(!robot.isLeftAlliance && !robot.isRedAlliance){
                telemetry.addData("Alliance","Blue, Right");
            }

            dashboardTelemetry.update();
            telemetry.update();


            /*
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             */


        }



        /*
         * Runs after hit start:
         * DO STUFF FOR the OPMODE!!!
         */
        runningTime.reset();
        if(robot.isLeftAlliance){
            left();
        }else if(!robot.isLeftAlliance){
            right();
        }
    }
    public void left(){
        CatHW_Vision.UltimateGoalPipeline.conePosition conePos = robot.eyes.getConePos();
        telemetry.addData("Position",conePos.toString());
        telemetry.update();
        robot.jaws.grabPos();
        robot.robotWait(1);
        robot.jaws.setLiftMiddlePole(1);
        robot.drive.quickDrive(3.5,8,0,1,5);
        robot.drive.setTightTolerance();
        //robot.drive.quickDrive(11,37,40,.3,5);
        robot.drive.quickDrive(15,33,0,.25,5);

        telemetry.addData("Pos","%.3f %.3f %.3f",robot.drive.realSense.getXPos(),robot.drive.realSense.getYPos(), robot.drive.realSense.getRotation());
        telemetry.update();
        robot.robotWait(2);
        robot.jaws.unGrab();
        robot.robotWait(2);
        robot.drive.setLooseTolerance();

        /*while (runningTime.seconds() < 20){
            robot.
        }*/
        robot.jaws.setLiftBottom(1);
        switch(conePos) {
            case NONE:
            case RIGHT:
                robot.drive.quickDrive(26, 27, 0, .5, 5);
                break;
            case MIDDLE:
                robot.drive.quickDrive(4, 26, 0, .5, 5);
                break;
            case LEFT:
                robot.drive.quickDrive(-22.5, 26, 0, .5, 6);
                break;
        }
        robot.robotWait(2);




    }
    public void right(){
        CatHW_Vision.UltimateGoalPipeline.conePosition conePos = robot.eyes.getConePos();
        int i = 1293;

        //Grabs Cone and lifts the lift
        robot.jaws.grabPos();
        robot.robotWait(1);
        robot.jaws.setLiftMiddlePole(1);
        robot.drive.setLooseTolerance();

        //Drives to the medium junction and drops the cone
        robot.drive.quickDrive(3.5,20,0,.5,5);
        robot.drive.setTightTolerance();
        robot.drive.quickDrive(-8,32.5,0,.25,5);
        robot.robotWait(1);
        robot.jaws.unGrab();
        robot.jaws.setLiftBottom(1);
        robot.robotWait(.5);
        robot.drive.quickDrive(-8,29,0,.5,5);

        //drives over to the stack of cones and picks one up
        robot.drive.setNormalTolerance();
        robot.drive.quickDrive(4,29,0,.5,5);
        robot.drive.quickDrive(4,47,0,.5,5);

        while(runningTime.seconds() < 20){

            robot.jaws.setLiftHeight(i);
            robot.robotWait(.5);
            robot.drive.setTightTolerance();
            robot.drive.quickDrive(29,58,90,.25,5);
            robot.robotWait(.25);
            robot.jaws.grabPos();
            robot.robotWait(.5);
            robot.jaws.setLiftMiddlePole(1);

            //drives over to drop cone onto short junction
            robot.robotWait(1);
            robot.drive.setNormalTolerance();
            robot.drive.quickDrive(24,55,90,.3,5);
            robot.jaws.setLiftLowPole(1);
            robot.drive.quickDrive(24,55,180,.3,5);
            robot.drive.setTightTolerance();
            robot.drive.quickDrive(24,49,180,.2,5);
            robot.robotWait(.5);
            robot.jaws.unGrab();
            if(runningTime.seconds() > 24){
                robot.robotWait(.5);
                break;
            }
            robot.drive.quickDrive(26,55,180,.3,5);
            robot.jaws.setLiftLowPole(1);
            robot.drive.quickDrive(26,55,90,.3,5);
            i -= 149;
        }
        robot.drive.quickDrive(26,53,180,.25,5);
        robot.jaws.setLiftBottom(1);

        robot.drive.setLooseTolerance();
        /*switch(conePos) {
            case NONE:
            case RIGHT:
                robot.drive.quickDrive(28, 27, 0, .5, 5);
                break;
            case MIDDLE:
                robot.drive.quickDrive(6, 27, 0, .5, 5);
                break;
            case LEFT:
                robot.drive.quickDrive(-23, 27, 0, 1, 6);
                break;
        }*/


    }
}
