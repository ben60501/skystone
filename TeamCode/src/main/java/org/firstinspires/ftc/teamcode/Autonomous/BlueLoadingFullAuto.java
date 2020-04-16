package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlueLoadingFullAuto extends LinearOpMode {

    private Robot robot = new Robot();
    private int skystonePosition = 2;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.outtakeAction(Robot.Outtake.Release);

        robot.releaseFoundation();

        waitForStart();

        robot.releaseFoundation();

        //Turn Around to Start
        //robot.backwards(12, 0.75);
        //robot.turnImu(120, 1);
        robot.deployIntake();
        sleep(500);
        robot.stopIntaking();


        if (skystonePosition == 0) {
            robot.resetAngle();
            robot.left(12);
            robot.turnToZero();

            robot.forward(5, 0.6);
            robot.turnImu(15, 0.25);
            robot.outtakeAction(Robot.Outtake.PrepareForIntake);

            robot.intake();
            robot.forward(12, 0.2);
            robot.turnImu(-10, 0.25);

            robot.turnImu(10, 0.75);
            robot.turnImu(-10,0.75);
            sleep(1000);

            robot.outtakeAction(Robot.Outtake.Grab);
            sleep(750);
            robot.outtakeAction(Robot.Outtake.Release);
            robot.stopIntaking();

            robot.outtakeAction(Robot.Outtake.Grab);

            robot.resetAngle();
            robot.right(2);
            robot.turnToZero();

            robot.outtakeAction(Robot.Outtake.Release);

            sleep(500);

            robot.outtakeAction(Robot.Outtake.PrepareForIntake);

            robot.intakeOut();
            sleep(100);
            robot.intake();
            sleep(350);
            robot.stopIntaking();

            robot.outtakeAction(Robot.Outtake.Down);

            robot.outtakeAction(Robot.Outtake.Grab);

            robot.backwards(12, 0.6);

            robot.outtakeAction(Robot.Outtake.Release);
            sleep(500);
            robot.outtakeAction(Robot.Outtake.Down);
            robot.outtakeAction(Robot.Outtake.Grab);

            robot.turnImu(-40, 0.75);

            robot.backwards(2, 0.75); //1 more
        } else if (skystonePosition == 1) {
            //Align to Intake
            robot.resetAngle();
            robot.left(7);
            robot.resetAngle();
            robot.forward(12, 0.6);
            robot.left(9);

            robot.resetAngle();
            robot.outtakeAction(Robot.Outtake.PrepareForIntake);

            //Intake Block
            robot.intake();
            robot.forward(10, 0.2);
            sleep(1000);

            robot.turnImu(-5, 0.75);
            robot.turnImu(10, 0.75);
            robot.turnImu(-5, 0.75);

            robot.outtakeAction(Robot.Outtake.Grab);
            sleep(750);
            robot.outtakeAction(Robot.Outtake.Release);

            robot.stopIntaking();

            robot.outtakeAction(Robot.Outtake.Grab);

            robot.turnToZero();

            robot.backwards(12, 0.6);

            robot.outtakeAction(Robot.Outtake.Release);

            robot.turnImu(-25, 0.75);
            robot.resetAngle();

            robot.outtakeAction(Robot.Outtake.PrepareForIntake);

            robot.intakeOut();
            sleep(100);
            robot.intake();
            sleep(350);
            robot.stopIntaking();

            robot.outtakeAction(Robot.Outtake.Down);

            robot.outtakeAction(Robot.Outtake.Grab);

            robot.right(3);

            robot.backwards(7, 0.6);

            robot.outtakeAction(Robot.Outtake.Release);

            //robot.turnToZero();
        } else if (skystonePosition == 2) {
            robot.forward(20, 0.6);

            robot.resetAngle();
            robot.left(10.5);
            //robot.turnToZero();

            robot.outtakeAction(Robot.Outtake.PrepareForIntake);
            robot.intake();
            robot.forward(10, 0.2);

            sleep(1000);

            //robot.turnImu(-5, 0.75);
            //robot.turnImu(10, 0.75);
            //robot.turnImu(-5, 0.75);

            robot.outtakeAction(Robot.Outtake.Grab);
            sleep(750);
            robot.outtakeAction(Robot.Outtake.Release);

            //robot.stopIntaking();

            robot.outtakeAction(Robot.Outtake.Grab);

            robot.backwards(16, 0.6);

            robot.outtakeAction(Robot.Outtake.Release);

            //robot.turnToZero();

            robot.turnImu(-30, 0.75);
            robot.resetAngle();

            robot.outtakeAction(Robot.Outtake.PrepareForIntake);

            robot.intakeOut();
            sleep(100);
            robot.intake();
            sleep(350);
            robot.stopIntaking();

            robot.outtakeAction(Robot.Outtake.Down);

            robot.outtakeAction(Robot.Outtake.Grab);

            robot.right(7);

            robot.backwards(12, 0.6);

            robot.turnToZero();

            robot.sleep(500);
        }

        //sleep(1000);

        robot.backwardsLong(58, 1);

        //robot.right(3);

        //robot.turnToZero();

        robot.turnImu(-90, 0.75);

        robot.backwards(6, 0.3);

        robot.grabFoundation();

        sleep(1000);

        robot.forward(30, 0.65);

        robot.left(10);

        robot.leftSlow(12, 0.6);

        robot.right(3);

        robot.turnImuSmall(50, 0.5);

        /*

        robot.releaseFoundation();

        sleep(1000);

        robot.turnImuSmall(70, 0.5);

        robot.forward(20, 0.75);

        //robot.turnImu(70, 0.4);
        //robot.left(16);

 */
    }
}
