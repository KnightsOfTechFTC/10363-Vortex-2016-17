package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Lego5 on 11/11/2016.
 */
@Autonomous(name="10363 Competition Autonomous on Blue Alliance (no beacon)")
public class Team10363AutoBlueNeither extends OpMode {
    Team_10363_Auto_Decs RobII = new Team_10363_Auto_Decs();
    DcMotor v_motor_ball_shooter;
    DcMotor v_motor_lift;

    @Override
    public void init() {

        RobII.init(hardwareMap);
        left_encoder = RobII.a_left_encoder_pos();
        right_encoder = RobII.a_right_encoder_pos();
        telemetry.addData("4: Heading", RobII.a_gyro_heading());
        telemetry.addData("6: Ground Color (Alpha)", RobII.a_ground_alpha());
        telemetry.update();
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
            v_motor_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        //If it doesn't work, set the motor to null and add record the problem in the Debug log.
        catch (Exception p_exception){
            v_motor_lift=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
    }

    @Override
    public void loop() {
        switch (v_state) {
            case 0:
                RobII.setDrivePower((float) (.5 - RobII.adjspeed(1, RobII.a_gyro_heading())), (float) (.5 + RobII.adjspeed(1, RobII.a_gyro_heading())));
                v_motor_ball_shooter.setPower(1);
                if (RobII.have_drive_encoders_reached(left_encoder - 4220, right_encoder - 4220, false)) {
                    RobII.setDrivePower(0, 0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();

                    v_state++;
                }
                break;
            case 1:
                RobII.setDrivePower((float) .4, 0);
                if (RobII.a_gyro_heading() >= 35 && RobII.a_gyro_heading() < 90) {
                    RobII.setDrivePower(0, 0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;
                    v_motor_lift.setPower(1);
                }
                break;

            case 2:
                RobII.setDrivePower((float) (.3 - RobII.adjspeed(1, RobII.a_gyro_heading() - 45)), (float) (.3 + RobII.adjspeed(1, RobII.a_gyro_heading() - 45)));
                if (RobII.have_drive_encoders_reached(left_encoder - 5000, right_encoder - 5000, false)) {
                    RobII.setDrivePower(0, 0);
                    v_motor_lift.setPower(0);
                    v_motor_ball_shooter.setPower(0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;
                }
                break;

        }

    }
    int v_state = 0;
    int left_encoder = 0;
    int right_encoder = 0;
    int encoderLeft;
    int encoderRight;
    double bblue = 0;
    double bred = 0;
    int count = 0;
    int leftEncoderProblems = 0;
    int rightEncoderProblems = 0;
}