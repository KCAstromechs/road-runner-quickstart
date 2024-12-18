
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Vector;

@Config
@Autonomous(name = "AutoTest2 (Java)", group = "Autonomous")
public class AutoTest1 extends LinearOpMode {

    @Override
    public void runOpMode() { //throws InterruptedException
        // instantiate your MecanumDrive at a particular pose.
        double initialX = -13;
        double initialY = 63.5;
        double initialHeading = Math.toRadians(180);
        String auto_type = "park"; // change "park" to "get samples" depending on auto goal, vice versa

        Pose2d initialPose = new Pose2d(initialX, initialY, initialHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


//      make a Grabber instance
        class Grabber {
            private Servo grabber;

            public Grabber(HardwareMap hardwareMap) {
                grabber = hardwareMap.get(Servo.class, "grabber");
            }

            public Action close_grabber() {
                return new Action() {
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (!initialized) {
                            // RUN CODE HERE
                            grabber.setPosition(0.5);
                            initialized = true;
                        }

                        double pos = grabber.getPosition();
                        packet.put("Grabber Pos", pos);
                        grabber_update(pos);
                        return false; // if return true, bad bc stops other Actions from working
                    }
                };
            }

            public Action open_grabber() {
                return new Action() {
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (!initialized) {
                            // RUN CODE HERE
                            grabber.setPosition(0.25);
                            initialized = true;
                        }

                        double pos = grabber.getPosition();
                        packet.put("Grabber Pos", pos);
                        grabber_update(pos);
                        return false; // if return true, bad bc stops other Actions from working
                    }
                };
            }
        }

        // make a Lift instance
        class Lift {
            private DcMotor lift;

            public Lift(HardwareMap hardwareMap) {
                // Init the lift
                lift = hardwareMap.get(DcMotor.class, "lift");
                lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            public Action lower_lift() {
                return new Action() {
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (!initialized) {
                            // RUN CODE HERE
                            // TODO change 0 to 'low' lift position
                            if (lift.getCurrentPosition() > 0) {
                                while (lift.getCurrentPosition() > 0) {
                                    lift.setPower(-0.5);
                                }
                                lift.setPower(0);
                            } else if (lift.getCurrentPosition() < 0) {
                                while (lift.getCurrentPosition() < 0) {
                                    lift.setPower(0.5);
                                }
                                lift.setPower(0);
                            }
                            initialized = true;
                        }

                        double pos = lift.getCurrentPosition();
                        packet.put("Lift Pos", pos);
                        return false; // if return true, bad bc stops other Actions from working
                    }
                };
            }

            public Action raise_lift() {
                return new Action() {
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (!initialized) {
                            // RUN CODE HERE
                            // TODO change 0 to 'raised' lift position
                            if (lift.getCurrentPosition() > 4200) {
                                while (lift.getCurrentPosition() > 4200) {
                                    lift.setPower(-0.5);
                                    lift_update(lift.getCurrentPosition());
                                }
                                lift.setPower(0);
                            } else if (lift.getCurrentPosition() < 4200) {
                                while (lift.getCurrentPosition() < 4200) {
                                    lift.setPower(0.5);
                                    lift_update(lift.getCurrentPosition());
                                }
                                lift.setPower(0);
                            }
                            initialized = true;
                        }

                        double pos = lift.getCurrentPosition();
                        packet.put("Lift Pos", pos);
                        return false; // if return true, bad bc stops other Actions from working
                    }
                };
            }
        }


        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                // Whatever the heck we want to happen goes directly below
                .strafeTo(new Vector2d(-37, 37))
                .strafeTo(new Vector2d(-37, 5))
                .strafeTo(new Vector2d(-47, 5))
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-47, 47))
                .strafeTo(new Vector2d(-47, 5))
                .strafeTo(new Vector2d(-59, 5))
                .strafeTo(new Vector2d(-59, 47))
                .strafeTo(new Vector2d(-59, 20))
                .turnTo(Math.toRadians(180))
                .strafeTo(new Vector2d(-59, 47))
                .waitSeconds(2);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-33, 63.5))
                .waitSeconds(2);
        Action trajectoryActionCloseOut = tab1.fresh()
//                .strafeTo(new Vector2d(48, 12))
//                .turn(Math.toRadians(360))
                .strafeTo(new Vector2d(initialX, initialY))
                .waitSeconds(1)
                .build();

        Grabber grabber = new Grabber(hardwareMap);

        Lift lift = new Lift(hardwareMap);

        // actions that need to happen on init; for instance, a claw tightening.
//        Actions.runBlocking(claw.closeClaw());

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;

        if (auto_type.equals("get samples")) {
            trajectoryActionChosen = tab1.build();
        } else if (auto_type.equals("park")) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab2.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        lift.raise_lift(),
                        new SleepAction(5)
                        //,
//                        trajectoryActionCloseOut
                )
        );

    }
    private void pose_update(Pose2d pose) {
        telemetry.addData("Current Pose: ", pose);
        telemetry.update();
    }

    private void grabber_update(double pos) {
        telemetry.addData("Grabber Actual Pos: ", pos);
        telemetry.update();
    }

    private void lift_update(double pos) {
        telemetry.addData("Lift Actual Pos: ", pos);
        telemetry.update();
    }
}
