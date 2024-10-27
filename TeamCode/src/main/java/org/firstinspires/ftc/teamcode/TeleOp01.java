package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.drivetrains.MecanumWithGyroscope;

@TeleOp(name="", group="MentorBotTeleOp")
public class TeleOp01 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    /*
     * IMU: This contains 3 different sensors in FTC on the Control Hub/Expansian Hub
     *
     * - Accelerameter: Used to determine the acceleration of the robot
     * - Gyroscope: Used to determine the robot's orientation from it's start position (0 deg is
     *              the initial start position when the robot is first turned on)
     * - Magnetometer: Used to measure magnetic forces, especially earth's magnetism.
     */
    private BNO055IMU imu;

    // Gyroscope: This is used to reset the gyroscope's 0 position in field centric mode
    private double resetAngle = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*
         * Used to connect the hardware wheel confirmation to the software wheel variables.
         *
         * hardwareMap is a variable defined in LinearOpMode which is why it can be used without
         * defining it in this class.    It is used to retrieve the configuration for the
         * requested value from the Control Hub/Expansion Hub.
         *
         * This code was written with the following ports in mind for each wheel
         * 0 = backRightMotor
         * 1 = backLeftMotor
         * 2 = frontRightMotor
         * 3 = frontLeftMotor
         */
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotorO");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotorR");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotorB");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotorG");

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

        MecanumWithGyroscope driveTrain = new MecanumWithGyroscope(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, imu, telemetry);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driveTrain.drive(gamepad1);

            driveTrain.resetAngle(gamepad1);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
