package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by mathnerd(Jacob) on 9/28/16.
 */

public class Team_10363_TeleOp_Decs {
    //motors
    private DcMotor v_motor_left_drive;
    private DcMotor v_motor_right_drive;
    private DcMotor v_motor_intake;
    private DcMotor v_motor_lift;
    private DcMotor v_motor_cap_left;
    private DcMotor v_motor_cap_right;
    private DcMotor v_motor_ball_shooter;

    //servos
    private Servo v_servo_left_beacon;
    private Servo v_servo_right_beacon;
    private CRServo v_right_beacon;
    private CRServo v_left_beacon;


    //sensors
    private GyroSensor SensorGyro;

    public void init(HardwareMap ahwMap) {
        /*Try to add the left drive motor, stop it if it's moving, set zero power behavior
        * to "brake", and turn encoders off. */
        try{
            v_motor_left_drive=ahwMap.dcMotor.get("left_drive");
            v_motor_left_drive.setDirection(DcMotor.Direction.FORWARD);
            v_motor_left_drive.setPower(0);
            v_motor_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        //If it doesn't work, set the motor to null and add record the problem in the Debug log.
        catch (Exception p_exception){
            v_motor_left_drive=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        //Same as above for the right motor, but reversed
        try {
            v_motor_right_drive=ahwMap.dcMotor.get("right_drive");
            v_motor_right_drive.setDirection(DcMotor.Direction.REVERSE);
            v_motor_right_drive.setPower(0);
            v_motor_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception p_exception){
            v_motor_right_drive=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        try {
            v_motor_cap_right=ahwMap.dcMotor.get("cap-right");
            v_motor_cap_right.setDirection(DcMotor.Direction.FORWARD);
            v_motor_cap_right.setPower(0);
            v_motor_cap_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_cap_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception p_exception){
            v_motor_cap_right=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        try {
            v_motor_cap_left=ahwMap.dcMotor.get("cap-left");
            v_motor_cap_left.setDirection(DcMotor.Direction.REVERSE);
            v_motor_cap_left.setPower(0);
            v_motor_cap_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_cap_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception p_exception){
            v_motor_cap_left=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        try{
            v_motor_intake=ahwMap.dcMotor.get("intake");
            v_motor_intake.setDirection(DcMotor.Direction.FORWARD);
            v_motor_intake.setPower(0);
            v_motor_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //If it doesn't work, set the motor to null and add record the problem in the Debug log.
        catch (Exception p_exception){
            v_motor_intake=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        try{
            v_motor_ball_shooter=ahwMap.dcMotor.get("ball_shooter");
            v_motor_ball_shooter.setDirection(DcMotor.Direction.REVERSE);
            v_motor_ball_shooter.setPower(0);
            v_motor_ball_shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            v_motor_ball_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        //If it doesn't work, set the motor to null and add record the problem in the Debug log.
        catch (Exception p_exception){
            v_motor_ball_shooter=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }

        try{
            v_servo_left_beacon=ahwMap.servo.get("left beacon");
            v_servo_left_beacon.setDirection(Servo.Direction.FORWARD);
            v_servo_left_beacon.setPosition(.3);

        }
        catch (Exception p_exception){
            v_servo_left_beacon=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        try{
            v_servo_right_beacon=ahwMap.servo.get("right beacon");
            v_servo_right_beacon.setDirection(Servo.Direction.REVERSE);
            v_servo_right_beacon.setPosition(.3);
        }
        catch (Exception p_exception){
            v_servo_right_beacon=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }

        try{
            v_right_beacon=ahwMap.crservo.get("right_beacon_sweep");
            v_right_beacon.setDirection(CRServo.Direction.FORWARD);
            v_right_beacon.setPower(0);
        }
        catch (Exception p_exception){
            v_right_beacon=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        try{
            v_left_beacon=ahwMap.crservo.get("left_beacon_sweep");
            v_left_beacon.setDirection(CRServo.Direction.REVERSE);
            v_left_beacon.setPower(0);
        }
        catch (Exception p_exception){
            v_left_beacon=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        try{
            v_motor_lift=ahwMap.dcMotor.get("lift");
            v_motor_lift.setDirection(DcMotor.Direction.FORWARD);
            v_motor_lift.setPower(0);
            v_motor_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //If it doesn't work, set the motor to null and add record the problem in the Debug log.
        catch (Exception p_exception){
            v_motor_lift=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        //Try to add the gyro
        try {
            SensorGyro=ahwMap.gyroSensor.get("gyro");
        }
        catch (Exception p_exception){
            DbgLog.msg(p_exception.getLocalizedMessage());
            SensorGyro=null;
        }
        //Calibrate the gyro. Drive team- don't hit start until the light starts blinking!
        if (SensorGyro!=null){
            SensorGyro.calibrate();
            while(SensorGyro.isCalibrating()){
                try {
                    Thread.sleep(50);
                }
                catch (Exception p_exception){}
            }
        }



    }
    //Modifies the left drive motor's power
    public void m_left_drive_power(float power){
        if (v_motor_left_drive!=null){
            float sendpower= Range.clip(power,-.8f,.8f);
            v_motor_left_drive.setPower(sendpower);
        }

    }
    //Same as above, but for the right drive motor
    public void m_right_drive_power(float power){
        if (v_motor_right_drive!=null){
            float sendpower=Range.clip(power,-.8f,.8f);
            v_motor_right_drive.setPower(sendpower);
        }
    }
    //Sets drive motors' power. Same as in Auto.
    public void setDrivePower(float left_power, float right_power){
        m_left_drive_power(left_power);
        m_right_drive_power(right_power);

    }
    //Same as above, but sets both to average if both are close.
    public void setDrivePowerWithCorrection(float left_power, float right_power){
        if (Math.abs(left_power-right_power)<=.05){
            setDrivePower((left_power+right_power)/2,(right_power+left_power)/2);
        }
        else {
            setDrivePower(left_power,right_power);
        }
    }
    /* Does the calculations for single stick driving for the left drive motor.
    *  To test the algorithm, use single_stick_drive_test*/

    public double single_stick_drive_left(double joystick_x, double joystick_y){

        int quadrant;
        double theta=0;
        double thetaAdjusted;
        double xAdjustedForAngle;
        double yAdjustedForAngle;
        double xAdjustedForAngleAnd1_1;
        double r;
        double scale;
        //find out what quadrant you're in
        if (joystick_x>=0&&joystick_y>=0){quadrant=1;}
        else if (joystick_x<=0&&joystick_y>=0){quadrant=2;}
        else if (joystick_x<=0&&joystick_y<=0){quadrant=3;}
        else {quadrant=4;}
        if (joystick_x==0&&joystick_y>=0){theta=90;}
        else if (joystick_x==0){theta=270;}
        else if (quadrant==1) {
            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x));
        }
        else if (quadrant==2){

            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x))+180;
        }
        else if (quadrant==3){
            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x))+180;
        }
        else if (quadrant==4){
            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x))+360;
        }
        //find r
        r=Math.sqrt((joystick_x*joystick_x)+(joystick_y*joystick_y));
        //find adjusted theta and x and y coordinates
        thetaAdjusted=theta-45;
        xAdjustedForAngle=r*(Math.cos(Math.toRadians(thetaAdjusted)));
        yAdjustedForAngle=r*(Math.sin(Math.toRadians(thetaAdjusted)));
        //scale power to max of (1,1)
        if (xAdjustedForAngle==0 && yAdjustedForAngle==0){scale=0;}
        else if (Math.abs(xAdjustedForAngle)>Math.abs(yAdjustedForAngle)){scale=Math.abs(1/Math.cos(Math.toRadians(thetaAdjusted)));}
        else {scale=Math.abs(1/Math.sin(Math.toRadians(thetaAdjusted)));}
        xAdjustedForAngleAnd1_1=xAdjustedForAngle*scale;
        return xAdjustedForAngleAnd1_1;

    }
    //Same as above, but for the right drive motor.
    public double single_stick_drive_right(double joystick_x, double joystick_y){

        int quadrant;
        double theta=0;
        double thetaAdjusted;
        double xAdjustedForAngle;
        double yAdjustedForAngle;
        double yAdjustedForAngleAnd1_1;
        double r;
        double scale;
        //find out what quadrant you're in
        if (joystick_x>=0&&joystick_y>=0){quadrant=1;}
        else if (joystick_x<=0&&joystick_y>=0){quadrant=2;}
        else if (joystick_x<=0&&joystick_y<=0){quadrant=3;}
        else {quadrant=4;}
        if (joystick_x==0&&joystick_y>=0){theta=90;}
        else if (joystick_x==0){theta=270;}
        else if (quadrant==1) {
            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x));
        }
        else if (quadrant==2){

            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x))+180;
        }
        else if (quadrant==3){
            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x))+180;
        }
        else if (quadrant==4){
            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x))+360;
        }
        //find r
        r=Math.sqrt((joystick_x*joystick_x)+(joystick_y*joystick_y));
        //find adjusted theta and x and y coordinates
        thetaAdjusted=theta-45;
        xAdjustedForAngle=r*(Math.cos(Math.toRadians(thetaAdjusted)));
        yAdjustedForAngle=r*(Math.sin(Math.toRadians(thetaAdjusted)));
        //scale power to max of (1,1)
        if (xAdjustedForAngle==0 && yAdjustedForAngle==0){scale=0;}
        else if (Math.abs(xAdjustedForAngle)>Math.abs(yAdjustedForAngle)){scale=Math.abs(1/Math.cos(Math.toRadians(thetaAdjusted)));}
        else {scale=Math.abs(1/Math.sin(Math.toRadians(thetaAdjusted)));}
        yAdjustedForAngleAnd1_1=yAdjustedForAngle*scale;
        return yAdjustedForAngleAnd1_1;

    }
    /* accesses the gyro's heading. If we can't find it, return an incorrect value that is divisible
    *  by 360. Remember that the gyro returns a clockwise heading. */
    public int a_gyro_heading(){
        if (SensorGyro!=null){
            return SensorGyro.getHeading();
        }
        else {return -36000;}
    }
    //calculates adjspeed, the speed correction factor in straightDrive.
    public double adjspeed(double speedModifier, int deltaAngle){
        return speedModifier*Math.sin(Math.toRadians(deltaAngle));
    }
    public void m_lift_power(float power){
        if (v_motor_lift!=null){
            float sendpower= Range.clip(power,-1,1);
            v_motor_lift.setPower(sendpower);
        }
        

    }
    public void m_cap_power(float power){
        if(v_motor_cap_left!=null&&v_motor_cap_right!=null){
            float sendpower=Range.clip(power,-1,1);
            v_motor_cap_left.setPower(sendpower);
            v_motor_cap_right.setPower(sendpower);
        }
    }
    public void m_intake_power(float power){
        if (v_motor_intake!=null){
            float sendpower=Range.clip(power,-1,1);
            v_motor_intake.setPower(sendpower);
        }
    }
    public void m_ball_shooting_power(float power){
        if (v_motor_ball_shooter!=null){
            float sendpower=(float)Range.clip(power,0,1);
            v_motor_ball_shooter.setPower(sendpower);
        }
    }
     void press_or_reset_beacons(boolean press){
        if (press){
            v_servo_left_beacon.setPosition(.6);
            v_servo_right_beacon.setPosition(.6);
        }
         else {
            v_servo_left_beacon.setPosition(.35);
            v_servo_right_beacon.setPosition(.35);
        }
    }
    public void beacon_extend(boolean beacons1) {
        if (beacons1) {
            v_left_beacon.setPower(-.4);
            v_right_beacon.setPower(-.4);
        }
    }
        public void beacon_retract(boolean beacons1){
            if(beacons1=false){
                v_left_beacon.setPower(.4);
                v_right_beacon.setPower(.4);
            }
    }
}
