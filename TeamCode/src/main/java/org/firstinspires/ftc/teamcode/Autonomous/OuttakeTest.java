package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class OuttakeTest extends LinearOpMode{

    private Robot robot = new Robot();

    double time = 0;

    private enum OuttakeState {
        RaisingStone, SwingingOut, LoweringStone, ReleasingStone, RaisingEmpty, SwingingIn, LoweringEmpty
    }

    private OuttakeState currentOuttakeState = OuttakeState.RaisingStone;

    @Override
    public void runOpMode() throws InterruptedException {
            robot.init(hardwareMap);

            resetStartTime();

            waitForStart();

            robot.outtakeAction(Robot.Outtake.Grab);

            sleep(750);

            robot.outtakeAction(Robot.Outtake.PowerUp);

            while (opModeIsActive()) {
                if (currentOuttakeState == OuttakeState.RaisingStone) {
                    if (robot.motorOuttake.getCurrentPosition() <= -1500) {
                        robot.motorOuttake.setPower(0.1);
                        currentOuttakeState = OuttakeState.SwingingOut;
                    }
                }

                if (currentOuttakeState == OuttakeState.SwingingOut) {
                    if (time == 0) {
                        robot.outtakeAction(Robot.Outtake.Out);
                        time = getRuntime();
                    }

                    if (getRuntime() - time >= 1) {
                        currentOuttakeState = OuttakeState.LoweringStone;
                        time = 0;
                    }
                }

                if (currentOuttakeState == OuttakeState.LoweringStone) {
                    robot.outtakeAction(Robot.Outtake.PowerDown);

                    if (robot.motorOuttake.getCurrentPosition() >= -50) {
                        robot.outtakeAction(Robot.Outtake.Stop);
                        currentOuttakeState = OuttakeState.ReleasingStone;
                    }
                }


            }
            /*
            robot.outtakeAction(Robot.Outtake.Grab);

            sleep(750);

            robot.outtakeAction(Robot.Outtake.Up);

            robot.outtakeAction(Robot.Outtake.Out);

            sleep(1000);

            robot.outtakeAction(Robot.Outtake.Down);

            robot.outtakeAction(Robot.Outtake.Release);

            robot.sleep(750);

            robot.outtakeAction(Robot.Outtake.Up);

            robot.outtakeAction(Robot.Outtake.In);

            sleep(1000);

            robot.outtakeAction(Robot.Outtake.Down);

             */
    }

}
