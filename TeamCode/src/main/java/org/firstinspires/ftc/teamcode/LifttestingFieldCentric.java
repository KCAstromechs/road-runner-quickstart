//  THIS IS MARK 1 CODE... NO TOUCHY FOR NOW


package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "LifttestingFieldCentric (Java)")
public class LifttestingFieldCentric extends LinearOpMode {

    // Drive Motors
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;

    // IMU
    private IMU imu;

    // Lift Motors
    private DcMotor lift1;

    // Grabber
    private Servo grabber;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        // IMU stuff
        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;

        // CONSTANTS
        double yawAngle;
        double Speed_percentage;

        // Drive Motors
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");

        lift1 = hardwareMap.get(DcMotor.class, "lift");

        grabber = hardwareMap.get(Servo.class, "grabber");


        // Motor Init shenanigans
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Speed_percentage = 0.6;
        yawAngle = 0;
        // Initialize the IMU.
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        // Prompt user to press start button.
        telemetry.addData("IMU Example", "Press start to continue...");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                telemetry.addData("Yaw/Rotation Angle", "Press Circle or B on Gamepad to reset.");
                // Check to see if reset yaw is requested.
                if (gamepad1.circle || gamepad1.b)   {
                    imu.resetYaw();
                }
                orientation = imu.getRobotYawPitchRollAngles();

                yawAngle = orientation.getYaw(AngleUnit.RADIANS);

                // Uncomment this line when possible
                double theta = yawAngle;
                // PI / 2; = 90 degrees (in terms of radians)

                double _inputX = gamepad1.left_stick_x;
                double _inputY = gamepad1.left_stick_y;

                // Changing vectors of joystick input
                double robotInputY = ((_inputX * Math.sin(theta)) + (_inputY * Math.sin(theta + (PI / 2)))); // *1.4
                double robotInputX = (_inputX * Math.cos(theta)) + (_inputY * Math.cos(theta + (PI / 2)));


                // Robot-centric drive base code (with edits to robotInputY and robotInputX turn this into Field-centric drive)
                double rightBackPower = (robotInputY + -robotInputX + gamepad1.right_stick_x) * Speed_percentage;
                double leftBackPower = (robotInputY + robotInputX + -gamepad1.right_stick_x) * Speed_percentage;
                double rightFrontPower = (robotInputY + robotInputX + gamepad1.right_stick_x) * Speed_percentage;
                double leftFrontPower = (robotInputY + -robotInputX + -gamepad1.right_stick_x) * Speed_percentage;


                // TODO ADD LIMITERS HERE??
                // Lift stuff
                double lift_normal_speed_limit = .7; // Normal lift1 speed will be 50% of max power
                double lift_speed_limit = lift_normal_speed_limit;

                double lift1_power = -gamepad2.right_stick_y;
                // Lift optimization
                if (lift1.getCurrentPosition() < -50 && lift1_power < 0) {
                    lift_speed_limit = 0;
                } else if (lift1_power < 0) {
                    lift_speed_limit = lift_normal_speed_limit * .3;
                } else {
                    lift_speed_limit = lift_normal_speed_limit;
                }


                // Telemetry
                // Drive motor telemetry
                telemetry.addData("Power of leftBack", leftBack.getPower());
                telemetry.addData("Power of rightBack", rightBack.getPower());
                telemetry.addData("Power of leftFront", leftFront.getPower());
                telemetry.addData("Power of rightFront", rightFront.getPower());

                // Lift telemetry
                telemetry.addData("lift_power", lift1.getPower());
                telemetry.addData("lift_position", lift1.getCurrentPosition());

                // TODO change the comment right below this
                /*
                For lift1:
                Up is negative encoders
                Down is positive encoders
                */


                // ------------------IF statements------------------

                // grabber
                if (gamepad2.a || gamepad2.left_trigger >= .5 || gamepad2.right_trigger >= .5 || gamepad2.left_bumper || gamepad2.right_bumper) {
                    grabber.setPosition(0.5); // close
                } else {
                    grabber.setPosition(0.25); // open
                }

                // BOOSTER BUTTON!!!!!
                if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    Speed_percentage = 1;
                } else {
                    Speed_percentage = 0.6;
                }

                lift1.setPower(lift1_power * lift_speed_limit);

                // FIELD CENTRIC CRAP

                /* highestPower is the highest value out of all of the absolute values of
                rightBackPower, leftBackPower, rightFrontPower, and leftFrontPower. */
                double highestPower = Math.max(Math.max(Math.abs(rightBackPower), Math.abs(leftBackPower)),
                        Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)));


                // Normalizing powers (the powers will never go above 1)
                if (highestPower > 1) {
                    leftBackPower = leftBackPower / highestPower;
                    rightBackPower = rightBackPower / highestPower;
                    leftFrontPower = leftFrontPower / highestPower;
                    rightFrontPower = rightFrontPower / highestPower;
                }

                // directional driving based on robot position
//                rightBack.setPower((gamepad1.right_stick_y + gamepad1.left_trigger + -gamepad1.right_trigger) * speedPercentage);
//                leftBack.setPower((gamepad1.left_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger) * speedPercentage);
//                rightFront.setPower((gamepad1.right_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger) * speedPercentage);
//                leftFront.setPower((gamepad1.left_stick_y + gamepad1.left_trigger + -gamepad1.right_trigger) * speedPercentage);

                rightBack.setPower((rightBackPower));
                leftBack.setPower((leftBackPower));
                rightFront.setPower((rightFrontPower));
                leftFront.setPower((leftFrontPower));
                telemetry.update();
                // Get the orientation and angular velocity.

                orientation = imu.getRobotYawPitchRollAngles();
                angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                telemetry.addData("Yaw Angle", JavaUtil.formatNumber(yawAngle, 2));
//                telemetry.addData("stick_y", gamepad1.left_stick_y);
//
//                telemetry.addData("robotInputY", robotInputY);
//                telemetry.addData("leftYCos", gamepad1.left_stick_y * Math.cos(theta));
//                telemetry.addData("leftXSin", gamepad1.left_stick_x * Math.sin(theta));

//                telemetry.addData("stick_x", gamepad1.left_stick_x);
//
//                telemetry.addData("robotInputX", robotInputX);
//                telemetry.addData("leftYSin", gamepad1.left_stick_y * Math.sin(theta));
//                telemetry.addData("LeftXCos", gamepad1.left_stick_x * Math.cos(theta));

//
//                telemetry.addData("rightStickY", gamepad1.right_stick_y);
//                telemetry.addData("rightStickX", gamepad1.right_stick_x);
//
//                telemetry.addData("Yaw Angle", yawAngle);
                telemetry.update();
            }
        }
    }
}