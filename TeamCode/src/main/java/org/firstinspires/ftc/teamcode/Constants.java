package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {
    public static final DcMotor.Direction FORWARD = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction REVERSE = DcMotor.Direction.REVERSE;

    public static final DcMotorSimple.Direction LEFTDIRECTION = FORWARD;
    public static final DcMotorSimple.Direction RIGHTDIRECTION = REVERSE;

    public static final DcMotor.ZeroPowerBehavior BRAKE = DcMotor.ZeroPowerBehavior.BRAKE;
    public static final DcMotor.ZeroPowerBehavior ZEROPOWERBEHAVIOR = BRAKE;
}
