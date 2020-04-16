package org.firstinspires.ftc.teamcode.Autonomous;

//import com.qualcomm.hardware.bosch.BNO055IMU;
import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDController;

public class Robot extends LinearOpMode {
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorOuttake, motorIntakeRight, motorIntakeLeft, motorIntakeMiddle;
    private Servo servoOuttakeRotate, servoOuttakeClaw, servoFoundationRight, servoFoundationLeft;

    private double power = 0.6, turnPower = 0.3, globalAngle, counts, countSub, correction, mfrPower=1, mflPower=1, mbrPower=1, mblPower=1, currentTarget=0;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    PIDController pidStrafe, PIDSpeed;

    private static final double COUNTS_PER_MOTOR_REV = 537.6;
    private static final double DRIVE_GEAR_REDUCTION = 0.98;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    enum Outtake {
        Up, Down, Out, PrepareForIntake, In, Grab, Release, PowerUp, PowerDown, Stop
    }

    public void init(HardwareMap hwdMap) {
        motorFrontRight = hwdMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft = hwdMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight = hwdMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft = hwdMap.get(DcMotor.class, "motorBackLeft");
        motorOuttake = hwdMap.get(DcMotor.class, "motorOuttake");
        motorIntakeRight = hwdMap.get(DcMotor.class, "motorIntakeRight");
        motorIntakeLeft = hwdMap.get(DcMotor.class, "motorIntakeLeft");
        motorIntakeMiddle = hwdMap.get(DcMotor.class, "motorIntakeMiddle");

        servoOuttakeRotate = hwdMap.get(Servo.class, "servoOuttakeRotate");
        servoOuttakeClaw = hwdMap.get(Servo.class, "servoOuttakeClaw");
        servoFoundationLeft = hwdMap.get(Servo.class, "servoFoundationLeft");
        servoFoundationRight = hwdMap.get(Servo.class, "servoFoundationRight");

        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorOuttake.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeRight.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeLeft.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeMiddle.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorOuttake.setPower(0);
        motorIntakeRight.setPower(0);
        motorIntakeLeft.setPower(0);
        motorIntakeMiddle.setPower(0);

        motorOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //servoOuttakeRotate.setPosition(.78);
        //servoFoundationRight.setPosition(0.75);
        //servoFoundationLeft.setPosition(0.25);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hwdMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        pidStrafe = new PIDController(0.03, .00006, 0);

        pidStrafe.setSetpoint(0);
        pidStrafe.setOutputRange(0, 1);
        pidStrafe.setInputRange(-90, 90);
        pidStrafe.enable();

        PIDSpeed = new PIDController(5, 0.05, 0);

        PIDSpeed.setSetpoint(0);
        PIDSpeed.setOutputRange(0, 0.82);
        PIDSpeed.setInputRange(0, 1200);
        PIDSpeed.enable();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
    }

    void arc() {
        double leftPower = 0, rightPower = 0;
        double degrees = 90;

        resetAngle();

        motorFrontRight.setPower(-0.5);
        motorFrontLeft.setPower(0.25);
        motorBackRight.setPower(-0.5);
        motorBackLeft.setPower(0.25);

        while (getAngle() < degrees) {}

        // turn the motors off.
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }

    void forward(double inches, double power) {

        int rightTarget = motorFrontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
        int leftTarget = motorFrontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);

        while (motorFrontRight.getCurrentPosition() >= rightTarget && motorFrontLeft.getCurrentPosition() >= leftTarget) {

            motorFrontRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackRight.setPower(power);
            motorBackLeft.setPower(power);
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    void forwardPID(double inches) {

        int rightTarget = motorFrontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
        int leftTarget = motorFrontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);

        PIDSpeed.setInputRange(0, (inches * COUNTS_PER_INCH));
        counts = 0;
        countSub = motorFrontRight.getCurrentPosition();
        double half = (inches * COUNTS_PER_INCH)/2;
        power = 1;

        while (motorFrontRight.getCurrentPosition() >= rightTarget && motorFrontLeft.getCurrentPosition() >= leftTarget) {

            counts = Math.abs(motorFrontRight.getCurrentPosition() - countSub);

            if(counts > half){
                power = 1 - PIDSpeed.performPID(counts * (-1));
            }else{
                power = PIDSpeed.performPID(counts * (-1)) + 0.18;
            }

            motorFrontRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackRight.setPower(power);
            motorBackLeft.setPower(power);
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    void backwards(double inches, double power) {
        int rightTarget = motorFrontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        int leftTarget = motorFrontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

        while (motorFrontRight.getCurrentPosition() <= (rightTarget) && motorFrontLeft.getCurrentPosition() <= (leftTarget)) {

            motorFrontRight.setPower(-power);
            motorFrontLeft.setPower(-power);
            motorBackRight.setPower(-power);
            motorBackLeft.setPower(-power);
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }

    void backwardsPID(double inches) {

        int rightTarget = motorFrontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        int leftTarget = motorFrontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

        PIDSpeed.setInputRange(0, (inches * COUNTS_PER_INCH));
        counts = 0;
        countSub = motorFrontRight.getCurrentPosition();
        double half = (inches * COUNTS_PER_INCH)/2;
        power = 1;

        while (motorFrontRight.getCurrentPosition() <= rightTarget && motorFrontLeft.getCurrentPosition() <= leftTarget) {

            counts = Math.abs(motorFrontRight.getCurrentPosition() - countSub);

            if(counts > half){
                power = 1 - PIDSpeed.performPID(counts * (-1));
            }else{
                power = PIDSpeed.performPID(counts * (-1)) + 0.18;
            }

            motorFrontRight.setPower(-power);
            motorFrontLeft.setPower(-power);
            motorBackRight.setPower(-power);
            motorBackLeft.setPower(-power);
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    void backwardsLong(double inches, double power) {
        int rightTargetOne = motorFrontRight.getCurrentPosition() + (int) ((inches - 5) * COUNTS_PER_INCH);
        int leftTargetOne = motorFrontLeft.getCurrentPosition() + (int) ((inches - 5) * COUNTS_PER_INCH);

        int rightTargetTwo = motorFrontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        int leftTargetTwo = motorFrontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

        motorFrontRight.setPower(-power);
        motorFrontLeft.setPower(-power);
        motorBackRight.setPower(-power);
        motorBackLeft.setPower(-power);

        while (motorFrontRight.getCurrentPosition() <= (rightTargetOne) && motorFrontLeft.getCurrentPosition() <= (leftTargetOne)) {}

        motorFrontRight.setPower(-0.5);
        motorFrontLeft.setPower(-0.5);
        motorBackRight.setPower(-0.5);
        motorBackLeft.setPower(-0.5);

        while (motorFrontRight.getCurrentPosition() <= (rightTargetTwo) && motorFrontLeft.getCurrentPosition() <= (leftTargetTwo)) {}

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }

    void right(double inches) {
        inches *= 2;
        int rightTarget = motorFrontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        int leftTarget = motorFrontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);

        motorFrontRight.setPower(-0.75);
        motorFrontLeft.setPower(0.75*0.87);
        motorBackRight.setPower(0.75*0.718);
        motorBackLeft.setPower(-0.75*0.848);

        while (motorFrontRight.getCurrentPosition() <= rightTarget && motorFrontLeft.getCurrentPosition() >= leftTarget) {}

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }

    void left(double inches) {
        inches *= 1.5;
        int rightTarget = motorFrontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
        int leftTarget = motorFrontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

        motorFrontRight.setPower(0.75);
        motorFrontLeft.setPower(-0.75);
        motorBackRight.setPower(-0.75);
        motorBackLeft.setPower(0.75);

        while (motorFrontRight.getCurrentPosition() >= rightTarget && motorFrontLeft.getCurrentPosition() <= leftTarget) {}

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }

    void rightSlow(double inches, double power) {
        inches *= 2;
        int rightTarget = motorFrontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        int leftTarget = motorFrontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);

        motorFrontRight.setPower(-power);
        motorFrontLeft.setPower(power*0.87);
        motorBackRight.setPower(power*0.718);
        motorBackLeft.setPower(-power*0.848);

        while (motorFrontRight.getCurrentPosition() <= rightTarget && motorFrontLeft.getCurrentPosition() >= leftTarget) {}

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }

    void leftSlow(double inches, double power) {
        inches *= 1.5;
        int rightTarget = motorFrontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
        int leftTarget = motorFrontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(-power);
        motorBackRight.setPower(-power);
        motorBackLeft.setPower(power);

        while (motorFrontRight.getCurrentPosition() >= rightTarget && motorFrontLeft.getCurrentPosition() <= leftTarget) {}

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }

    void foundationCurve() {
        int rightTarget = motorFrontRight.getCurrentPosition() - (int) (100 * COUNTS_PER_INCH);
        int leftTarget = motorFrontLeft.getCurrentPosition() - (int) (200 * COUNTS_PER_INCH);

        motorFrontRight.setPower(0.25);
        motorFrontLeft.setPower(0.125);
        motorBackRight.setPower(0.25);
        motorBackLeft.setPower(0.125);

        while (motorFrontRight.getCurrentPosition() >= rightTarget && motorFrontLeft.getCurrentPosition() >= leftTarget) {}

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }

    void grabFoundation() {
        servoFoundationRight.setPosition(0.725);
        servoFoundationLeft.setPosition(0.75);

        sleep(250);
    }

    void releaseFoundation() {
        servoFoundationRight.setPosition(1);
        servoFoundationLeft.setPosition(1);
    }


    void deployIntake() {
        motorIntakeLeft.setPower(1);
        motorIntakeRight.setPower(1);
        sleep(500);
        stopIntaking();
    }

    void intake() {
        motorIntakeRight.setPower(0.35);
        motorIntakeLeft.setPower(-0.35);
        motorIntakeMiddle.setPower(1);
    }

    void intakeOut() {
        motorIntakeRight.setPower(-0.35);
        motorIntakeLeft.setPower(0.35);
        motorIntakeMiddle.setPower(-1);
    }

    void stopIntaking() {
        motorIntakeRight.setPower(0);
        motorIntakeLeft.setPower(0);
        motorIntakeMiddle.setPower(0);
    }

    void outtakeAction(Outtake action) {
        if (action == Outtake.Up) {
            motorOuttake.setPower(0.75);

            while (motorOuttake.getCurrentPosition() > -1500) {}

            motorOuttake.setPower(0.1);
        } else if (action == Outtake.Down) {
            motorOuttake.setPower(-0.5);

            while (motorOuttake.getCurrentPosition() < 0) {}

            motorOuttake.setPower(0);
        } else if (action == Outtake.PrepareForIntake) {
            motorOuttake.setPower(0.75);

            while (motorOuttake.getCurrentPosition() > -100) {}

            motorOuttake.setPower(0.1);
        } else if (action == Outtake.Grab) {
            servoOuttakeClaw.setPosition(0.45);
        } else if (action == Outtake.Release) {
            servoOuttakeClaw.setPosition(0);
        } else if (action == Outtake.Out) {
            servoOuttakeRotate.setPosition(0.1);
        } else if (action == Outtake.In) {
            servoOuttakeRotate.setPosition(0.8);
        } else if (action == Outtake.PowerUp) {
            motorOuttake.setPower(0.75);
        } else if (action == Outtake.PowerDown) {
            motorOuttake.setPower(-0.5);
        } else if (action == Outtake.Stop) {
            motorOuttake.setPower(0);
        }
    }

    void turnImu(double degrees, double turnPower) {

        double leftPower = 0, rightPower = 0;
        PIDSpeed.setInputRange(0, Math.abs(degrees));
        counts = 0;
        double half = Math.abs(degrees/2);
        power = 1;

        resetAngle();

        if (degrees < 0) {
            while (getAngle() > degrees) {
                counts = Math.abs(getAngle());
                if(counts > half){
                    turnPower = 1 - PIDSpeed.performPID(counts * (-1));
                }else{
                    turnPower = PIDSpeed.performPID(counts * (-1)) + 0.18;
                }
                leftPower = turnPower;
                rightPower = -turnPower;

                motorFrontRight.setPower(rightPower);
                motorFrontLeft.setPower(leftPower);
                motorBackRight.setPower(rightPower);
                motorBackLeft.setPower(leftPower);
            }
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackRight.setPower(0);
            motorBackLeft.setPower(0);

            if(getAngle() != degrees) {
                while (getAngle() < degrees) {
                    motorFrontRight.setPower(0.26);
                    motorFrontLeft.setPower(-0.26);
                    motorBackRight.setPower(0.26);
                    motorBackLeft.setPower(-0.26);
                }
            }
        }
        else if (degrees > 0){
            while (getAngle() < degrees) {
                counts = Math.abs(getAngle());
                if(counts > half){
                    turnPower = 1 - PIDSpeed.performPID(counts * (-1));
                }else{
                    turnPower = PIDSpeed.performPID(counts * (-1)) + 0.18;
                }
                leftPower = -turnPower;
                rightPower = turnPower;

                motorFrontRight.setPower(rightPower);
                motorFrontLeft.setPower(leftPower);
                motorBackRight.setPower(rightPower);
                motorBackLeft.setPower(leftPower);
            }
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackRight.setPower(0);
            motorBackLeft.setPower(0);

            if(getAngle() != degrees) {
                while (getAngle() > degrees) {
                    motorFrontRight.setPower(-0.15);
                    motorFrontLeft.setPower(0.15);
                    motorBackRight.setPower(-0.15);
                    motorBackLeft.setPower(0.15);
                }
            }
        }
        else return;

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

    }

    void turnImuSmall(double degrees, double turnPower) {

        double leftPower = 0, rightPower = 0;

        resetAngle();

        if (degrees < 0) {   // turn left.
            leftPower = turnPower;
            rightPower = -turnPower;
        } else if (degrees > 0) {   // turn right.
            leftPower = -turnPower;
            rightPower = turnPower;
        }
        else return;

        // set power to rotate.
        motorFrontRight.setPower(rightPower);
        motorFrontLeft.setPower(leftPower);
        motorBackRight.setPower(rightPower);
        motorBackLeft.setPower(leftPower);

        if (degrees < 0) {

            while (getAngle() == 0) {}

            while (getAngle() > degrees) {}
        }
        else {   // left turn.
            while (getAngle() < degrees) {}
        }

        // turn the motors off.
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        // wait for rotation to stop.
        //sleep(1000);

        // reset angle tracking on new heading.
        //resetAngle();
    }

    void turnImuBasic(double degrees, double turnPower) {

        double leftPower = 0, rightPower = 0;

        resetAngle();

        if (degrees < 0) {   // turn left.
            leftPower = turnPower;
            rightPower = -turnPower;
        } else if (degrees > 0) {   // turn right.
            leftPower = -turnPower;
            rightPower = turnPower;
        }
        else return;

        // set power to rotate.
        motorFrontRight.setPower(rightPower);
        motorFrontLeft.setPower(leftPower);
        motorBackRight.setPower(rightPower);
        motorBackLeft.setPower(leftPower);

        // rotate until turn is completed.
        if (degrees < 0) { // right turn
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {}

            while (getAngle() > degrees + 25) {}
        } else {   // left turn.
            while (getAngle() < degrees - 25) {
            }
        }

        //motorFrontRight.setPower(0.2 * (rightPower / turnPower));
        //motorFrontLeft.setPower(0.2 * (leftPower / turnPower));
        //motorBackRight.setPower(0.2 * (rightPower / turnPower));
        //motorBackLeft.setPower(0.2 * (leftPower / turnPower));

        if (degrees < 0) {
            while (getAngle() > degrees) {}
        }
        else {   // left turn.
            while (getAngle() < degrees) {}
        }

        // turn the motors off.
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        // wait for rotation to stop.
        //sleep(1000);

        // reset angle tracking on new heading.
        //resetAngle();
    }

    void turnToZero() {
        turnImuBasic(-getAngle(), 0.16);
    }

    void turnToZeroFast(double power) {
        turnImuBasic(-getAngle(), power);
    }

    void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    String CalStatus() {
        return imu.getCalibrationStatus().toString();
    }

    double returnFirstAngle() {
        return lastAngles.firstAngle;
    }

    double returnGlobalAngle() {
        return globalAngle;
    }

    double returnCorrection() {
        return correction;
    }

    double returnAngle() { return getAngle(); }


    @Override
    public void runOpMode() throws InterruptedException {}
}
