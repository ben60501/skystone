package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.PIDController;

@TeleOp(name = "HolonomicPID", group = "TeleOp tests")
public class HolonomicPID extends OpMode {

    private DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorOuttake, motorIntakeRight, motorIntakeLeft, motorIntakeMiddle;
    private Servo servoOuttakeRotate, servoOuttakeClaw, servoFoundationLeft, servoFoundationRight, servoCapstone;
    private double powerFR, powerFL, powerBR, powerBL;
    private double leftX, rightX, leftY, rightY, leftX2, leftY2;

    private double leftBumperTime = 0, rightBumperTime = 0;

    private boolean leftBumper = false, rightBumper = false;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction, rotation;
    PIDController pidStrafe;

    @Override
    public void init() {

        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorOuttake = hardwareMap.get(DcMotor.class, "motorOuttake");
        motorIntakeLeft = hardwareMap.get(DcMotor.class, "motorIntakeLeft");
        motorIntakeRight = hardwareMap.get(DcMotor.class, "motorIntakeRight");
        motorIntakeMiddle = hardwareMap.get(DcMotor.class, "motorIntakeMiddle");

        servoCapstone = hardwareMap.get(Servo.class, "servoCapstone");
        servoOuttakeClaw = hardwareMap.get(Servo.class, "servoOuttakeClaw");
        servoOuttakeRotate = hardwareMap.get(Servo.class, "servoOuttakeRotate");
        servoFoundationLeft = hardwareMap.get(Servo.class, "servoFoundationLeft");
        servoFoundationRight = hardwareMap.get(Servo.class, "servoFoundationRight");

        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorOuttake.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeRight.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeLeft.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeMiddle.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorOuttake.setPower(0);
        motorIntakeRight.setPower(0);
        motorIntakeLeft.setPower(0);
        motorIntakeMiddle.setPower(0);

        servoCapstone.setPosition(0.2);

        motorOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        pidStrafe = new PIDController(.003, .00003, 0);

        pidStrafe.setSetpoint(0);
        pidStrafe.setOutputRange(0, power);
        pidStrafe.setInputRange(-90, 90);
        pidStrafe.enable();

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        resetStartTime();
    }

    @Override
    public void loop() {

        telemetry.addData("Mode", "running");
        telemetry.update();

        leftX2 = gamepad2.left_stick_x;
        leftY2 = -gamepad2.left_stick_y;

        double ry = -gamepad1.right_stick_y;
        double rx = gamepad1.right_stick_x;
        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x;

        double angleRad = Math.abs(Math.atan(ry/rx));
        double angleDeg = Math.toDegrees(angleRad);
        angleDeg = normalizeAngle(angleDeg);

        double anglePow = ((angleDeg-45)/45);
        double distance = Math.sqrt(Math.pow(ry,2)+Math.pow(rx,2));
        double distancePow = distance;

        if(Math.abs(lx)>0.1){
            powerFR = -lx;
            powerFL = lx;
            powerBR = -lx;
            powerBL = lx;
        } else if(Math.abs(rx)>0.1 || Math.abs(ry)>0.1){
            if(Math.abs(rx) > 0 && Math.abs(ry) < 0.1){
                correction = pidStrafe.performPID(getAngle());

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 turn rotation", rotation);
                telemetry.update();

                if(getAngle()<0){
                    if(rx>0) {
                        powerFR = (anglePow * distancePow) - Math.abs(correction); //+
                        powerFL = (distancePow) - Math.abs(correction);
                        powerBR = (distancePow) + Math.abs(correction); //+
                        powerBL = (anglePow * distancePow) + Math.abs(correction);
                    }else{
                        powerFR = (distancePow) - Math.abs(correction); //+
                        powerFL = (anglePow * distancePow) - Math.abs(correction);
                        powerBR = (anglePow * distancePow) + Math.abs(correction); //+
                        powerBL = (distancePow) + Math.abs(correction);
                    }
                }else if(getAngle()>0){
                    if(rx>0) {
                        powerFR = (anglePow * distancePow) + Math.abs(correction); //+
                        powerFL = (distancePow) + Math.abs(correction);
                        powerBR = (distancePow) - Math.abs(correction); //+
                        powerBL = (anglePow * distancePow) - Math.abs(correction);
                    }else{
                        powerFR = (distancePow) + Math.abs(correction); //+
                        powerFL = (anglePow * distancePow) + Math.abs(correction);
                        powerBR = (anglePow * distancePow) - Math.abs(correction); //+
                        powerBL = (distancePow) - Math.abs(correction);
                    }
                }else{
                    if(rx>0) {
                        powerFR = (anglePow * distancePow); //+
                        powerFL = (distancePow);
                        powerBR = (distancePow); //+
                        powerBL = (anglePow * distancePow);
                    }else{
                        powerFR = (distancePow); //+
                        powerFL = (anglePow * distancePow);
                        powerBR = (anglePow * distancePow); //+
                        powerBL = (distancePow);
                    }
                }
            }else if (rx > 0 && ry >= 0) {
                //quad 1
                powerFR = anglePow * distancePow;
                powerFL = distancePow;
                powerBR = distancePow;
                powerBL = anglePow * distancePow;
            } else if (rx < 0 && ry >= 0) {
                //quad 2
                powerFR = distancePow;
                powerFL = anglePow * distancePow;
                powerBR = anglePow * distancePow;
                powerBL = distancePow;
            } else if (rx < 0 && ry < 0) {
                //quad 3
                powerFR = -anglePow * distancePow;
                powerFL = -distancePow;
                powerBR = -distancePow;
                powerBL = -anglePow * distancePow;
            } else if (rx > 0 && ry < 0) {
                //quad 4
                powerFR = -distancePow;
                powerFL = -anglePow * distancePow;
                powerBR = -anglePow * distancePow;
                powerBL = -distancePow;
            } else if (rx == 0) {
                powerFR = ry;
                powerFL = ry;
                powerBR = ry;
                powerBL = ry;
            }
        }else{
            powerFR = 0;
            powerFL = 0;
            powerBR = 0;
            powerBL = 0;
        }

        if(!gamepad1.right_bumper) {
            motorFrontRight.setPower(powerFR);
            motorFrontLeft.setPower(powerFL);
            motorBackRight.setPower(powerBR);
            motorBackLeft.setPower(powerBL);
        }else if(gamepad1.right_bumper){
            motorFrontRight.setPower(powerFR*0.7);
            motorFrontLeft.setPower(powerFL*0.7);
            motorBackRight.setPower(powerBR*0.7);
            motorBackLeft.setPower(powerBL*0.7);
        }

        //Intake (a intake (and stop) b outtake) d pad down is down; up is up, left or right to make horizontal
        if (gamepad2.a) {
            motorIntakeRight.setPower(0.35);
            motorIntakeLeft.setPower(-0.35);
            motorIntakeMiddle.setPower(1);
        } else if (gamepad2.b) {
            motorIntakeRight.setPower(-0.35);
            motorIntakeLeft.setPower(0.35);
            motorIntakeMiddle.setPower(-1);
        } else if (gamepad2.left_trigger > 0.5) {
            motorIntakeLeft.setPower(1);
            motorIntakeRight.setPower(1);
        } else {
            motorIntakeRight.setPower(0);
            motorIntakeLeft.setPower(0);
            motorIntakeMiddle.setPower(0);
        }

        //Outtake
        if (Math.abs(leftY2) > 0.05) {
            motorOuttake.setPower(leftY2);
        } else {
            if (motorOuttake.getCurrentPosition() < -100) {
                motorOuttake.setPower(0.1);
            } else {
                motorOuttake.setPower(0);
            }
        }

        telemetry.addData("Position: ", motorOuttake.getCurrentPosition());
        telemetry.update();

        //Check if bumper was actually pressed and not held
        if(gamepad2.left_bumper) {
            if(leftBumperTime == 0 || getRuntime() - leftBumperTime > 0.5) {
                leftBumper = !leftBumper;
            }
            leftBumperTime = getRuntime();
        }

        //Action depending on state of left bumper
        if (leftBumper) {
            servoOuttakeRotate.setPosition(0.1);

        } else {
            servoOuttakeRotate.setPosition(0.8);
        }

        if (gamepad2.x) {
            servoOuttakeClaw.setPosition(0.45);
        } else if (gamepad2.y) {
            servoOuttakeClaw.setPosition(0);
        }

        if (gamepad2.dpad_up) {
            servoCapstone.setPosition(0.02);
        } else if (gamepad2.dpad_down) {
            servoCapstone.setPosition(0.52);
        }

        //Check if bumper was actually pressed and not held
        if(gamepad2.right_bumper) {
            if(rightBumperTime == 0 || getRuntime() - rightBumperTime > 0.5) {
                rightBumper = !rightBumper;
            }
            rightBumperTime = getRuntime();
        }

        //Action depending on state of left bumper
        if (rightBumper) {
            servoFoundationRight.setPosition(0.85);
            servoFoundationLeft.setPosition(0.8);
        } else {
            servoFoundationRight.setPosition(1);
            servoFoundationLeft.setPosition(1);
        }

        telemetry.addData("angleDeg: ", angleDeg);
        telemetry.addData("anglePow: ", anglePow);
        telemetry.addData("rx: ", rx);
        telemetry.addData("ry: ", ry);

        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        if (angle < 2.5) {
            angle = 0;
        } else if (angle > 42.5 && angle < 47.5) {
            angle = 45;
        } else if (angle > 87.5) {
            angle = 90;
        }
        return angle;
    }

    private double getAngle()
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

}