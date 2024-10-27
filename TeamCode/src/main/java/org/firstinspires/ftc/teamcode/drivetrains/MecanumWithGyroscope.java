package org.firstinspires.ftc.teamcode.drivetrains;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Constants;

public class MecanumWithGyroscope {
    private HardwareMap hardwareMap;
    private BNO055IMU imu;
    private Telemetry telemetry;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private double resetAngle = 0;

    public MecanumWithGyroscope(
            HardwareMap hardwareMap,
            BNO055IMU imu,
            Telemetry telemetry
    ) {
        this.hardwareMap = hardwareMap;

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
        this.frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotorO");
        this.frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotorR");
        this.backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotorB");
        this.backRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotorG");

        this.imu = imu;

        this.telemetry = telemetry;

        /*
         * By default all wheels are set to forward direction.   Above is defined the
         * leftDirection and rightDirection variables.   This is used to make sure the
         * left wheels are moving in the same direction and the right wheels are moving
         * in the same direction.
         */
        this.frontLeftMotor.setDirection(Constants.LEFTDIRECTION);
        this.backLeftMotor.setDirection(Constants.LEFTDIRECTION);
        this.frontRightMotor.setDirection(Constants.RIGHTDIRECTION);
        this.backRightMotor.setDirection(Constants.RIGHTDIRECTION);
        /*
         * Used as the default state when no power is being supplied to the motors from
         * the user.
         */
        this.frontLeftMotor.setZeroPowerBehavior(Constants.ZEROPOWERBEHAVIOR);
        this.backLeftMotor.setZeroPowerBehavior(Constants.ZEROPOWERBEHAVIOR);
        this.frontRightMotor.setZeroPowerBehavior(Constants.ZEROPOWERBEHAVIOR);
        this.backRightMotor.setZeroPowerBehavior(Constants.ZEROPOWERBEHAVIOR);
    }

    public void drive(Gamepad gamepad1) {
        drive(
            gamepad1.left_stick_x,
            gamepad1.left_stick_y,
            gamepad1.right_stick_x,
            gamepad1.right_bumper,
            gamepad1.dpad_right,
            gamepad1.dpad_left,
            gamepad1.dpad_up,
            gamepad1.dpad_down
        );
    }

    public void drive( // Overloaded method for testing
            double leftStickX,
            double leftStickY,
            double rightStickX
    ) {
        drive(
                leftStickX,
                leftStickY,
                rightStickX,
                false,
                false,
                false,
                false,
                false
        );

    }

    private void drive(
            double leftStickX,
            double leftStickY,
            double rightStickX,
            boolean rightBumper,
            boolean dpadRight,
            boolean dpadLeft,
            boolean dpadUp,
            boolean dpadDown
    ) {
        /*
         * protate is used to adjust the speed of the robot's rotation.
         *
         * This can be divided by any number but we chose 4 as it was the smoothest for rotation.
         *
         * The range of this when divided by 4 is -0.25 <= protate <= 0.25 radians
         */
        double protate = rightStickX / 4;

        /*
         * The total power cap should be 1, as that's the limitation of the DcMotor.  The
         * power cap of linear motion is 1 - protate.  The maximum magnitude will happen when
         * stick_x = stick_y = 1
         *
         * This formula is based on the pythagorean theorem (a^2 + b^2 = c^2).
         *
         * In this case, we know that the c^2, the hypotenuse, is c^2 = 1 - protate
         *
         * Since we know the maximum magnitude of stick_x = stick_y = 1, we can change the formula to
         *
         * 1 - protate = Math.sqrt(stick_x^2 + stick_y^2) = Math.sqrt(2 * stick_x^2) = Math.sqrt(2 * stick_y^2)
         *
         * Solve for stick_x.
         * NOTE: We could have used stick_y, but since we are calculating magnitude and they both
         * have a meximum of 1, thus being equal, it doesn't really matter.  So, I chose stick_x.
         *
         * 1 - protate = Math.sqrt(2 * stick_x^2)
         *
         * (1 - protate)^2 = 2 * stick_x^2
         *
         * (1 - protate)^2 / 2 = stick_x^2
         *
         * Math.sqrt((1 - protate)^2 / 2) = stick_x
         *      OR
         * stick_x = Math.sqrt((1 - protate)^2 / 2)
         *
         * The range of this is 0.5303 <= processed protate <= 0.707 radians
         *
         * This value relates to the turn.
         * If controller at 0, then power turn adjustment is 0.5303 radians.
         * If controller at 1, then power turn adjustment is 0.707 radians.
         */
        double processedProtate = Math.sqrt( Math.pow( 1 - Math.abs( protate ), 2 ) / 2 );

        /*
         * Analog stick * (magnitude value between 0.5303 - 0.707)
         */
        double stickX = leftStickX * processedProtate;
        double stickY = leftStickY * processedProtate;

        // Is the angle of the left analog stick in radians.
        double theta    = 0;

        // Is the magnitude/hypotenuse of the analog stick.
        double cValue   = 0;

        // Calculated magnitude to be sent to the DcMotor.
        double pX       = 0;
        double pY       = 0;

        // 1/2 PI = 1.5707 radians = 90 degrees
        double halfPi = Math.PI / 2;

        // 1/4 PI = 0.7853 radians = 45 degrees
        double quarterPi = Math.PI / 4;

        /*
         * Converts Gyroscope angle (degrees) into radians
         *
         * 1 radian = Pi / 180 = 0.01745
         *
         * getHeading() converts 360 to -180 to 180.   The reason for this is to denote if the robot
         * is turned left (-1 to -180 deg) or right (1 to 180 deg).
         *
         * gyroAngle = (left or right degrees) * (1 radian = 0.0.1745)
         *
         * Range -3.141 radians (-180 deg) <= gyroAngle <= 3.141 radians (180 deg)
         *
         * I'm assuming we're rotating the final angle halfPi for the same reason we will rotate it
         * in theta.   The axis for the controller is:
         *
         *        -1
         *         |
         *   -1 <--+--> 1
         *         |
         *         1
         *
         * and we want the Cartesian plane to be the correct orientation.
         */
        double gyroAngle = getHeading() * Math.PI / 180;

        if ( gyroAngle <= 0 ) {
            // if gyroAngle between 0.000 to -3.141 radians

            // gyroAngle = (0.000 to -3.141) + 1.5707
            // Range -1.5707 to 1.5707
            gyroAngle = gyroAngle + halfPi;
        } else if ( 0 < gyroAngle  && gyroAngle < halfPi ) {
            // if 0 radians < gyroAngle < 1.5707 radians

            // gyroAngle = (0.001 to 1.5706 radians) + 1.5707 radians
            // Range 1.5708 to 3.141 radians
            gyroAngle = gyroAngle + halfPi;
        } else if ( halfPi <= gyroAngle ) {
            // if 1.5707 radians <= gyroAngle (1.5707 to 3.141 radians

            // gyroAngle = (1.5707 to 3.141) - (3 * 1.5707)
            // gyroAngle = (1.5707 to 3.141) - 4.7121

            // Range -3.141 to -1.5707
            gyroAngle = gyroAngle - (3 * halfPi);
        }

        gyroAngle = -1 * gyroAngle;

        //Disables gyro, sets to -Math.PI/2 so front is defined correctly
        if( rightBumper ) {
            gyroAngle = -halfPi;
        }

        // Fixed power linear directions in case you want to do straight lines
        if (dpadRight) {
            stickX = 0.5;
        } else if ( dpadLeft){
            stickX = -0.5;
        }

        if (dpadUp) {
            stickY = 0.5;
        } else if ( dpadDown ) {
            stickY = -0.5;
        }

        //MOVEMENT
        /**
         * the range of motion on a joystick:
         *       -1.0
         *        |
         *  -1.0 -+- 1.0`
         *        |
         *       1.0
         * using an inverse tangent you can calculate the angle at the center is the circle from the X plane to the point
         * defined by the joystick's X and Y coordinates
         *
         * gyroangle is the angle the gyroscope is from the starting angle to its current position
         *
         * halfPi is 90 degrees or splitting the circle into quadrants
         *
         * theta calculates the angle of the joysticks adjusted by the position(angle) of the gyroscope in relation to
         * 1/4 of the circle (90 degrees). this angle is used to help determine the power values for left/right and forward/back
         *
         * Math.sin finds the length of the side in relation to the angle from the center of the circle increased/reduced by
         * 1/8 of the circle or 45 degree sections
         * NOTE: Math.sin is most likely adjusting the position of the coordinates on the joystick/robot with the angle of the wheels
         *
         * x^2 + y^2 * sine(calculated center angle +/- the angle of the wheels) = power
         *
         *    Omniwheels 45 deg         Mecanum in X
         *      pX /---\ pY             pY \---/ pX
         *         |   |                    | |
         *      pY \---/ pX             pX /---\ pY
         *
         *  halfPi = 1.5707
         *  gyroAngle =
         */
        theta = Math.atan2(stickY, stickX) - gyroAngle - halfPi;

        /**
         * this formula is calculating the value of c in the Pythagorean theorem, but using
         * stickX for 'a' and stickY for 'b'
         *
         * C being the length of the radial line from the center
         *
         * then to take the radial line and rotate based on the calculated amount based on calculated
         * angle and the angle of the robot
         */
        cValue = Math.sqrt(Math.pow(stickX, 2) + Math.pow(stickY, 2));

        // We're rotating values by 45 deg because of the angle of the wheels
        pX = cValue * (Math.sin(theta + quarterPi));
        pY = cValue * (Math.sin(theta - quarterPi));

        telemetry.addData("stickX", stickX);
        telemetry.addData("stickY", stickY);
        telemetry.addData("Magnitude", cValue);

        // Mecanum wheels in an X formation
        telemetry.addData("Front Left", pY - protate);
        telemetry.addData("Back Left", pX - protate);
        telemetry.addData("Back Right", pY + protate);
        telemetry.addData("Front Right", pX + protate);

        // Mecanum
        frontLeftMotor.setPower(pY - protate);
        backLeftMotor.setPower(pX - protate);
        frontRightMotor.setPower(pX + protate);
        backRightMotor.setPower(pY + protate);

        telemetry.update();
    }

    /**
     * Used to set the gyroscope starting angle.
     * the initial starting angle is based on when the robot is turned on
     * if the robot is moved after it is on, it needs to have a starting position
     *
     * EXTRA: maybe this can be automatically
     */
    public void resetAngle(Gamepad gamepad1) {
        if (gamepad1.a){
            resetAngle = getHeading() + resetAngle;
        }
    }

    public double getHeading(){
        // Axesreference.INTRINSIC = intrinsic rotations, where the axes move with the object that is rotating.
        // AxesOrder.ZYX = the order of the axes are returned.
        // AngleUnit.DEGREES = Returns the angle in DEGREES.
        Orientation angles = imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        );

        /*
         * Retrieves the first axis' value in degrees
         */
        double heading = angles.firstAngle;
        if ( heading < -180 ) {
            heading = heading + 360;
        } else if ( heading > 180 ) {
            heading = heading - 360;
        }

        heading = heading - resetAngle;

        return heading;
    }
}
