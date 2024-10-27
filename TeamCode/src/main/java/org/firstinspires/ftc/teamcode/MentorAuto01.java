package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivetrains.MecanumWithGyroscope;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Autonomous(name="Mentor - Auto 01", group="MentorBotAuto")
public class MentorAuto01 extends LinearOpMode {
    /**********  Timer  **********/
    private ElapsedTime runtime = new ElapsedTime();

    private double forwardDirection = -1.0;
    private double rightDirection = 1.0;

    /**********  IMU  **********/
    /*
     * IMU: This contains 3 different sensors in FTC on the Control Hub/Expansian Hub
     *
     * - Accelerameter: Used to determine the acceleration of the robot
     * - Gyroscope: Used to determine the robot's orientation from it's start position (0 deg is
     *              the initial start position when the robot is first turned on)
     * - Magnetometer: Used to measure magnetic forces, especially earth's magnetism.
     */
    private BNO055IMU imu;

    private Vision vision;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*
         * IMU: This is retrieving the IC2 configuration from the Control Hub/Expansion Hub
         * for the three sensors (accelerometer, gyroscope, and magnetometer).
         */
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        /*
         * IMU: Used to configure Gyroscope and Accelerometer
         */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode             = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit        = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled   = false;

        // IMU: Initialize the IMU sensors with the desired settings.
        imu.initialize(parameters);

        MecanumWithGyroscope driveTrain = new MecanumWithGyroscope(hardwareMap, imu, telemetry);

        Vision vision = new Vision(hardwareMap, telemetry);
        vision.initAprilTag();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            vision.telemetryAprilTag();

            driveTrain.drive(
                    0.0 * rightDirection, // Strafe right (pos) or left (neg)
                    0.0 * forwardDirection,        // Move forward (pos) or backward (neg)
                    0.6 * rightDirection           // Rotate clockwise (pos) or counterclockwise (neg)
            ); // Move forward at full speed

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
