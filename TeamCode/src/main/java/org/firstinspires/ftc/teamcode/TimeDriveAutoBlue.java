package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Lego5 on 11/23/2016.
 */
@Autonomous(name = "blue auto both beacons")
public class TimeDriveAutoBlue extends LinearOpMode {
    //Officially, our robot's name is Shartzmugel (votes for: @Lukcio, @Wonnie123, votes against: @jlevine18)
    /* RIP Robert the Robot 2015-2016. May his Res-Q skills be remembered by his 2 children,
        Shartzmugel and JVBot (a temp name) and his lifelong friend 9924Bot.  */
    //time
    private ElapsedTime runtime = new ElapsedTime();
    //ints
    int left_encoder;
    int right_encoder;
    //motors
    DcMotor v_motor_left_drive;
    DcMotor v_motor_right_drive;
    DcMotor v_motor_intake;
    DcMotor v_motor_ball_shooter;
    DcMotor v_motor_lift;
    //servos
    Servo v_servo_left_beacon;
    Servo v_servo_right_beacon;
    //sensors
    GyroSensor SensorGyro;
    ColorSensor GroundColor;
    ColorSensor SideColor;
    ColorSensor FrontColor;
    OpticalDistanceSensor ODS;

    @Override

    public void runOpMode() throws InterruptedException {

        try {
            v_motor_left_drive = hardwareMap.dcMotor.get("right_drive");
            v_motor_left_drive.setDirection(DcMotor.Direction.REVERSE);
            v_motor_left_drive.setPower(0);
            v_motor_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //If it doesn't work, set the motor to null and add record the problem in the Debug log.
        catch (Exception p_exception) {
            v_motor_left_drive = null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        //Same as above for the right motor, but reversed
        try {
            v_motor_right_drive = hardwareMap.dcMotor.get("left_drive");
            v_motor_right_drive.setDirection(DcMotor.Direction.FORWARD);
            v_motor_right_drive.setPower(0);
            v_motor_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception p_exception) {
            v_motor_right_drive = null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        try {
            v_motor_intake = hardwareMap.dcMotor.get("intake");
            v_motor_intake.setDirection(DcMotor.Direction.FORWARD);
            v_motor_intake.setPower(0);
            v_motor_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //If it doesn't work, set the motor to null and add record the problem in the Debug log.
        catch (Exception p_exception) {
            v_motor_intake = null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }

        //Try to add the beacon pushing servos. The right one is in reverse.
        try {
            v_servo_left_beacon = hardwareMap.servo.get("left beacon");
            v_servo_left_beacon.setDirection(Servo.Direction.FORWARD);
            v_servo_left_beacon.setPosition(.3);

        } catch (Exception p_exception) {
            v_servo_left_beacon = null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        try {
            v_servo_right_beacon = hardwareMap.servo.get("right beacon");
            v_servo_right_beacon.setDirection(Servo.Direction.REVERSE);
            v_servo_right_beacon.setPosition(.3);
        } catch (Exception p_exception) {
            v_servo_right_beacon = null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        //Try to add the gyro
        try {
            SensorGyro = hardwareMap.gyroSensor.get("gyro");
        } catch (Exception p_exception) {
            DbgLog.msg(p_exception.getLocalizedMessage());
            SensorGyro = null;
        }
        //Calibrate the gyro. Drive team- don't hit start until the light starts blinking!
        if (SensorGyro != null) {
            SensorGyro.calibrate();
            while (SensorGyro.isCalibrating()) {
                Thread.sleep(50);
            }
        }
        //Try to add the color sensors
        try {
            GroundColor = hardwareMap.colorSensor.get("ground");
            GroundColor.setI2cAddress(I2cAddr.create8bit(0x3C));
            GroundColor.enableLed(true);
        } catch (Exception p_exception) {
            DbgLog.msg(p_exception.getLocalizedMessage());
            GroundColor = null;
        }
        try{
            v_motor_ball_shooter=hardwareMap.dcMotor.get("ball_shooter");
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
            v_motor_lift=hardwareMap.dcMotor.get("lift");
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
        try {
            SideColor = hardwareMap.colorSensor.get("side-color");
            SideColor.setI2cAddress(I2cAddr.create8bit(0x42));
            SideColor.enableLed(false);
        } catch (Exception p_exception) {
            DbgLog.msg(p_exception.getLocalizedMessage());
            SideColor = null;
        }
        try {
            FrontColor = hardwareMap.colorSensor.get("front-color");
            FrontColor.setI2cAddress(I2cAddr.create8bit(0x46));
            FrontColor.enableLed(false);
        } catch (Exception p_exception) {
            DbgLog.msg(p_exception.getLocalizedMessage());
            FrontColor = null;
        }

        left_encoder = a_left_encoder_pos();
        right_encoder = a_right_encoder_pos();

        while (!isStarted()) {
            telemetry.addData("4: Heading", a_gyro_heading());
            telemetry.addData("6: Ground Color (Alpha)", a_ground_alpha());
            if (FrontColor!=null){
                telemetry.addData("8: beacon color",FrontColor.green());
            }else{
                telemetry.addData("front color not inited",0);
            }
            telemetry.addData("this is the newest version",0);

            telemetry.update();
            idle();
        }
        v_motor_right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        v_motor_left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timedrive(1000, .5f, .5f, -5);
        gyroturn(45, 11);
        gyrohold(1000,45);
        setDrivePower(0, 0);
        runtime.reset();
        while (a_ground_alpha() < 7 && opModeIsActive() && runtime.seconds() < 2.5) {
            telemetry.addData("-1: time driving", runtime.milliseconds());
            double adjspeed = (.5 + .5) * Math.sin(((2 * Math.PI) / 360) * (a_gyro_heading() - 45));
            telemetry.addData("2: adjspeed: ", adjspeed);
            v_motor_left_drive.setPower(Range.clip(.3 - adjspeed, -1, 1));
            v_motor_right_drive.setPower(Range.clip(.3 + adjspeed, -1, 1));
            telemetry.addData("5: Heading ", a_gyro_heading());
            telemetry.addData("6: Ground Color (Blue) ", a_ground_blue());
            telemetry.addData("7: Ground Color (Alpha) ", a_ground_alpha());
            telemetry.addData("8: Beacon Red ", a_left_red());
            telemetry.addData("9: Beacon Blue ", a_left_blue());
            telemetry.addData("10: last state left ", left_encoder);
            telemetry.addData("11: last state right ", right_encoder);
            telemetry.addData("12: actual left power ", actual_left_power());
            telemetry.update();
            idle();

        }
        timedrive(500,.5,.5,45);
        gyroturn(270, 20);
        timedrive(400,.5,.5,270);
        gyroturn(270,6);
        gyrohold(1000,270);
        setDrivePower(0, 0);
        if (FrontColor!=null) {
            timedrive(2700, -.2, -.2, 90);
            setDrivePower(0, 0);
            runtime.reset();
            while (runtime.seconds() < 1) {
            }
            if (FrontColor.red() > 2) {
                timedrive(500, .2, .2, 90);
                runtime.reset();
                gyrohold(5000,90);
                timedrive(1000, -.2, -.2, 90);
                setDrivePower(0,0);
                timedrive(2000, .2, .2, 90);
                setDrivePower(0,0);
            } else if (FrontColor.blue()>2){
                timedrive(2000, .2, .2, 90);
                while (runtime.seconds() < 1.5) {
                    telemetry.addData("blue", 2);
                }
            } else {
                timedrive(500,.2,.2,90);
                setDrivePower(0,0);
                runtime.reset();
                while (runtime.seconds()<1){
                    telemetry.addData("neither",0);
                    telemetry.update();
                }
                runtime.reset();
                while (opModeIsActive()&&FrontColor.red()<2&&FrontColor.blue()<2&&runtime.seconds()<1.2){
                    setDrivePower(.3f,-.3f);
                }
                setDrivePower(0,0);
                if (FrontColor.red()>2){
                    timedrive(700,-.2,-.2,90);
                    timedrive(2000,.2,.2,90);
                }else {
                    timedrive(1500,.2,.2,90);
                }
            }
            gyroturn(180, 10);
            setDrivePower(0,0);
            runtime.reset();
            timedrive(3000,.3,.3,180);
        }
        /*runtime.reset();
        while (a_ground_alpha() < 7 && opModeIsActive() && runtime.seconds() < 2.5) {
            telemetry.addData("-1: time driving", runtime.milliseconds());
            double adjspeed = (.5 + .5) * Math.sin(((2 * Math.PI) / 360) * (a_gyro_heading() - 45));
            telemetry.addData("2: adjspeed: ", adjspeed);
            v_motor_left_drive.setPower(Range.clip(.3 - adjspeed, -1, 1));
            v_motor_right_drive.setPower(Range.clip(.3 + adjspeed, -1, 1));
            telemetry.addData("5: Heading ", a_gyro_heading());
            telemetry.addData("6: Ground Color (Blue) ", a_ground_blue());
            telemetry.addData("7: Ground Color (Alpha) ", a_ground_alpha());
            telemetry.addData("8: Beacon Red ", a_left_red());
            telemetry.addData("9: Beacon Blue ", a_left_blue());
            telemetry.addData("10: last state left ", left_encoder);
            telemetry.addData("11: last state right ", right_encoder);
            telemetry.addData("12: actual left power ", actual_left_power());
            telemetry.update();
            idle();

        }
        timedrive(1000,.5,.5,15);
        gyroturn(270, 20);
        timedrive(400,.5,.5,270);
        gyroturn(270,5);
        gyrohold(1000,270);
        setDrivePower(0, 0);
        if (FrontColor!=null) {
            timedrive(2700, -.2, -.2, 90);
            setDrivePower(0, 0);
            runtime.reset();
            while (runtime.seconds() < 1) {
            }
            if (FrontColor.red() > 2) {
                timedrive(2000, .2, .2, 90);
                runtime.reset();
                while (runtime.seconds() < 1.5) {
                    telemetry.addData("red", 2);
                }
                timedrive(2000, -.2, -.2, 90);
                timedrive(2000, .2, .2, 90);
            } else {
                timedrive(2000, .2, .2, 90);
                while (runtime.seconds() < 1.5) {
                    telemetry.addData("blue", 2);
                }
            }
            gyroturn(15, 10);

            runtime.reset();
        }
  /*          while (a_ground_alpha() < 7 && opModeIsActive() && runtime.seconds() < 2.5) {
                telemetry.addData("-1: time driving", runtime.milliseconds());
                double adjspeed = (.5 + .5) * Math.sin(((2 * Math.PI) / 360) * (a_gyro_heading()));
                telemetry.addData("2: adjspeed: ", adjspeed);
                v_motor_left_drive.setPower(Range.clip(.3 - adjspeed, -1, 1));
                v_motor_right_drive.setPower(Range.clip(.3 + adjspeed, -1, 1));
                telemetry.addData("5: Heading ", a_gyro_heading());
                telemetry.addData("6: Ground Color (Blue) ", a_ground_blue());
                telemetry.addData("7: Ground Color (Alpha) ", a_ground_alpha());
                telemetry.addData("8: Beacon Red ", a_left_red());
                telemetry.addData("9: Beacon Blue ", a_left_blue());
                telemetry.addData("10: last state left ", left_encoder);
                telemetry.addData("11: last state right ", right_encoder);
                telemetry.addData("12: actual left power ", actual_left_power());
                telemetry.update();
                idle();

            }
            gyroturn(270,5);
            gyrohold(1000,270);
            runtime.reset();

            while ((!(FrontColor.blue() > 2 && FrontColor.red() < 2) || !(FrontColor.blue() > 2 && FrontColor.red() < 2)) && opModeIsActive()) {
                setDrivePower(-.3f, -.3f);
                telemetry.addData("5: Heading ", a_gyro_heading());
                telemetry.addData("6: Ground Color (Blue) ", a_ground_blue());
                telemetry.addData("7: Ground Color (Alpha) ", a_ground_alpha());
                telemetry.addData("8: Beacon Red ", a_left_red());
                telemetry.addData("9: Beacon Blue ", a_left_blue());
                telemetry.addData("10: last state left ", left_encoder);
                telemetry.addData("11: last state right ", right_encoder);
                telemetry.addData("12: actual left power ", actual_left_power());
                idle();
            }
            if (FrontColor.blue() > 2 && FrontColor.red() < 2) {
                gyroturn(60, 5);
                timedrive(1000, -.3f, -.3f, 60);
                timedrive(1000, 0, 0, 60);
                timedrive(1500, .3f, .3f, 60);
                gyroturn(0, 5);
            } else {
                gyroturn(120, 5);
                timedrive(1000, -.3f, -.3f, 120);
                timedrive(1000, 0, 0, 120);
                timedrive(1500, .3f, .3f, 120);
                gyroturn(0, 5);
            }
            gyroturn(45, 5);
            gyrohold(1000, 45);
            if (v_motor_ball_shooter != null && v_motor_lift != null) {
                v_motor_ball_shooter.setPower(1);
                timedrive(2000, -.5f, -.5f, 45);
                gyrohold(2000, 45);
                v_motor_lift.setPower(1);
                gyrohold(100, 45);
                v_motor_lift.setPower(0);
                v_motor_ball_shooter.setPower(1);
            }
            timedrive(500, -.3f, -.3f, 315);
    */

        /*


        double millel=0;
        while (millel<3000){
            runtime.reset();
            boolean addTime=true;
           telemetry.addData("0: target heading",0);
            telemetry.addData("1: actual heading",a_gyro_heading());
            double adjspeed=(1.75)*Math.sin(((2*Math.PI)/360)*(a_gyro_heading()-0));
            telemetry.addData("2: adjspeed: " ,adjspeed);
            if (v_motor_left_drive!=null&&v_motor_right_drive!=null){
                v_motor_left_drive.setPower(Range.clip(.3-adjspeed,-1,1));
                v_motor_right_drive.setPower(Range.clip(.3+adjspeed,-1,1));
            }
            if (sees blue){
                setDrivePower(0,0);
                addTime=false;
                press beacons;

            }
            if(addTime){
                millel=millel+runtime.milliseconds();
            }

        }
        gyroturn(315,5);
        gyrohold(1000,315);
        if (v_motor_ball_shooter!=null&&v_motor_lift!=null){
            v_motor_ball_shooter.setPower(1);
            timedrive(2000, -.5,-.5,315)
            gyrohold(2000,315);
            v_motor_lift.setPower(1);
            gyrohold(100,315);
            v_motor_lift.setPower(0);
            v_motor_ball_shooter.setPower(1);
        }
        timedrive(500,-.3,-.3,315);
         */


    }
































    public void gyrohold(int mills, int targetheading)throws InterruptedException{
        runtime.reset();
        while (runtime.milliseconds()<mills&&opModeIsActive()){
            telemetry.addData("0: target heading",targetheading);
            telemetry.addData("1: actual heading",a_gyro_heading());
            double adjspeed=(1.65)*Math.sin(((2*Math.PI)/360)*(a_gyro_heading()-targetheading));
            telemetry.addData("2: adjspeed: " ,adjspeed);
            if (v_motor_left_drive!=null&&v_motor_right_drive!=null){
                v_motor_left_drive.setPower(Range.clip(-adjspeed,-1,1));
                v_motor_right_drive.setPower(Range.clip(adjspeed,-1,1));
            }
            telemetry.addData("3: time passed (ms)", runtime.milliseconds());
            telemetry.update();
            // Allow time for other processes to run.

            idle();
        }
    }
    public void gyroturn(int targetheading, int error)throws InterruptedException{
        runtime.reset();
        while((a_gyro_heading()<targetheading-error||a_gyro_heading()>targetheading+error)&&opModeIsActive()){
            double adjspeed=(2.2)*Math.sin(((2*Math.PI)/360)*(a_gyro_heading()-targetheading));
            telemetry.addData("2: adjspeed: " ,adjspeed);
            if (v_motor_left_drive!=null&&v_motor_right_drive!=null){
                v_motor_left_drive.setPower(Range.clip(-adjspeed,-1,1));
                v_motor_right_drive.setPower(Range.clip(adjspeed,-1,1));
            }
            telemetry.addData("0: target heading",targetheading);
            telemetry.addData("1: actual heading",a_gyro_heading());
            telemetry.addData("3: time passed (ms)", runtime.milliseconds());
            telemetry.addData("4: error range", error);
            telemetry.update();
            // Allow time for other processes to run.

            idle();
        }
    }

    public void timedrive(int mills, double speedleft, double speedright, int targetheading) throws InterruptedException{
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds()<mills) {
                telemetry.addData("-1: time driving",runtime.milliseconds());
                if (targetheading>=0&&targetheading<=360){
                double adjspeed=(speedleft+speedright)*Math.sin(((2*Math.PI)/360)*(a_gyro_heading()-targetheading));
                telemetry.addData("2: adjspeed: " ,adjspeed);
                v_motor_left_drive.setPower(speedleft-adjspeed);
                v_motor_right_drive.setPower(speedright+adjspeed);
            }
            else{
                v_motor_left_drive.setPower(speedleft);
                v_motor_right_drive.setPower(speedright);
            }

            telemetry.addData("5: Heading ", a_gyro_heading());
            telemetry.addData("6: Ground Color (Blue) ", a_ground_blue());
            telemetry.addData("7: Ground Color (Alpha) ", a_ground_alpha());
            telemetry.addData("8: Beacon Red ", a_left_red());
            telemetry.addData("9: Beacon Blue ", a_left_blue());
            telemetry.addData("10: last state left ", left_encoder);
            telemetry.addData("11: last state right ", right_encoder);
            telemetry.addData("12: actual left power ", actual_left_power());
            telemetry.update();

            // Allow time for other processes to run.
            idle();
        }
        setDrivePower(0,0);
    }

    //methods
    //Modifies the left drive motor's power
    public void m_left_drive_power(float power){
        if (v_motor_left_drive!=null){
            float sendpower= Range.clip(power,-.6f,.6f);
            v_motor_left_drive.setPower(sendpower);
        }
    }
    public void m_intake_power(float power){
        if (v_motor_intake!=null){
            float sendpower= Range.clip(power,-1,1);
            v_motor_intake.setPower(sendpower);
        }
    }

    //Same as above, but for the right drive motor
    public void m_right_drive_power(float power){
        if (v_motor_right_drive!=null){
            float sendpower=Range.clip(power,-.6f,.6f);
            v_motor_right_drive.setPower(sendpower);
        }
    }

    // Sets both drive motors at once. In teleOp, will have correction if close.
    public void setDrivePower(float left_power, float right_power){
        m_left_drive_power(left_power);
        m_right_drive_power(right_power);

    }
    //calculates adjspeed, the speed correction factor in straightDrive.
    public double adjspeed(double speedModifier, int deltaAngle){
        return speedModifier*Math.sin(Math.toRadians(deltaAngle));
    }
    /* accesses the gyro's heading. If we can't find it, return an incorrect value that is divisible
    *  by 360. Remember that the gyro returns a clockwise heading. */
    public int a_gyro_heading(){
        if (SensorGyro!=null){
            return SensorGyro.getHeading();
        }
        else {return -36000;}
    }
    public double actual_left_power(){
        if (v_motor_left_drive!=null){
            return v_motor_left_drive.getPower();
        } else return 0;
    }
    public double actual_right_power(){
        if (v_motor_right_drive!=null){
            return v_motor_right_drive.getPower();
        } else return 0;
    }

    //Resets the left drive wheel encoder.
    public void reset_left_drive_encoder ()

    {
        if (v_motor_left_drive != null)
        {
            v_motor_left_drive.setMode
                    ( DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    );
        }

    }
    //Resets the right drive wheel encoder.
    public void reset_right_drive_encoder ()

    {
        if (v_motor_right_drive != null)
        {
            v_motor_right_drive.setMode
                    ( DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    );
        }

    }
    //Resets both encoders
    public void reset_drive_encoders(){
        reset_left_drive_encoder();
        reset_right_drive_encoder();
    }
    //Gets the left drive motor's encoder pos
    public int a_left_encoder_pos(){
        if (v_motor_left_drive!=null){
            return v_motor_left_drive.getCurrentPosition();
        }
        else {
            return 0;
        }
    }
    //Same as above, but for the right drive motor
    public int a_right_encoder_pos(){
        if (v_motor_right_drive!=null){
            return v_motor_right_drive.getCurrentPosition();
        }
        else {
            return 0;
        }
    }
    //Have the drive encoders reached a certain value?
    public boolean have_drive_encoders_reached(int left, int right, boolean forwards){
        if (forwards){
            return a_left_encoder_pos()>=left&&a_right_encoder_pos()>=right;
        }
        else {
            return a_left_encoder_pos()<=left&&a_right_encoder_pos()<=right;
        }
    }
    public void push_left_beacon(){
        if (v_servo_left_beacon!=null){v_servo_left_beacon.setPosition(.6);}
    }
    public void move_left_beacon_to_read(){
        if (v_servo_left_beacon!=null){v_servo_left_beacon.setPosition(.45);}
    }
    public void reset_left_beacon_servo(){
        if (v_servo_left_beacon!=null) {v_servo_left_beacon.setPosition(.3);}
    }
    public void push_right_beacon(){
        if (v_servo_right_beacon!=null) {v_servo_right_beacon.setPosition(.6);}
    }
    public void reset_right_beacon_servo(){
        if (v_servo_right_beacon!=null) {v_servo_right_beacon.setPosition(.3);}
    }
    public double a_ground_blue(){
        if (GroundColor!=null) {
            return GroundColor.blue();
        }
        else return -1;
    }
    public double a_ground_alpha(){
        return GroundColor.alpha();

    }
    public double a_left_blue(){
        double returnthis = -1;
        if (SideColor != null) {
            returnthis = SideColor.blue();
        }
        return returnthis;
    }
    public double a_left_red(){
        double returnthis = -1;
        if (SideColor != null) {
            returnthis = SideColor.red();
        }
        return returnthis;
    }
}

