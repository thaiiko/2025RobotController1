package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "BaseBotTele", group = "teleop")
public class Teleop_Basebot extends LinearOpMode {
    double direction_x, direction_y, pivot, heading;
    Project1Hardware robot;
    Gamepad gamepad;

    static final double WHEEL_CIRCUMFERENCE_MM  = 300.0 * 3.14;
    static final double COUNTS_PER_MOTOR_REV   = 28.0;
    static final double DRIVE_GEAR_REDUCTION    = 30.24;

    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_MM           = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;
    Gamepad lastGamepad;
    public boolean closeZone = true, farZone;

    @Override
    public void runOpMode() {

        robot = new Project1Hardware();
        robot.init(hardwareMap);

        MecanumDrive drive = new MecanumDrive(robot);


        gamepad = new Gamepad();
        lastGamepad = new Gamepad();

        // long lastTime = System.currentTimeMillis();
        // int leftTarget = (int)(610 * COUNTS_PER_MM);
        // int rightTarget = (int)(610 * COUNTS_PER_MM);
        // double TPS = ((double) 145 / 60) * COUNTS_PER_WHEEL_REV;

        waitForStart();
        robot.imu.resetYaw();

        // robot.lShooter.setTargetPosition(leftTarget);
        // robot.rShooter.setTargetPosition(rightTarget);

        // robot.lShooter.setVelocity(TPS);
        // robot.lShooter.setVelocity(TPS);

        while (opModeIsActive()) {

            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);


            direction_x = gamepad.left_stick_x;
            direction_y = gamepad.left_stick_y;
            pivot       = gamepad.right_stick_x * 0.8;
            heading     = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            if (gamepad.touchpad && !lastGamepad.touchpad) {
                robot.imu.resetYaw();
            }


            double shootCurVel = robot.getShooterVel();

            if (gamepad.y && !lastGamepad.y) {
                closeZone = true; farZone = false;
            } else if (gamepad.a && !lastGamepad.a) {
                farZone = true; closeZone = false;
            }

            // drive.remote(direction_y, direction_x, pivot, heading);
            // double frontLeftPower = Math.max(Math.abs(direction_y) + Math.abs(direction_x), 1);
            // double backLeftPower = (direction_y + direction_x + pivot);
            // double frontRightPower = (direction_y - direction_x - pivot);
            // double backRightPower = (direction_y + direction_x - pivot);

            // robot.frontLeft.setPower(frontLeftPower);
            // robot.backLeft.setPower(backLeftPower);
            // robot.backRight.setPower(backRightPower);
            // robot.frontRight.setPower(frontRightPower);

            if (gamepad.dpad_left && !lastGamepad.dpad_left) {
                robot.setIntake(1);
            } else if (gamepad.dpad_right && !lastGamepad.dpad_right) {
                robot.intakeOff();
            }

            if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                if (closeZone) {
                    robot.setShooter(0.7);
                } else if (farZone) {
                    robot.setShooter(0.75);
                }
            } else if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                robot.shooterOff();
            }

            // --- Index Control ---
            if (gamepad.x && !lastGamepad.x) {
                robot.setIndexPos(robot.getIndexPos() + 280);
            } else if (gamepad.b && !lastGamepad.b) {
                robot.setIndexPos(robot.getIndexPos() - 280);
            }

            telemetry.addData("shooterVel", robot.getShooterVel());
            telemetry.addData("closeZone", closeZone);
            telemetry.addData("farZone", farZone);
            telemetry.addData("heading", heading);
            telemetry.update();
        }
    }
}
