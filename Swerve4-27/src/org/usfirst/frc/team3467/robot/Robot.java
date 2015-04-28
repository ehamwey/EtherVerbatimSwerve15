package org.usfirst.frc.team3467.robot;


import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.SampleRobot;

/**
 * This is a short sample program demonstrating how to use the Talon SRX over
 * CAN to run a closed-loop PID controller with an analog potentiometer.
 */
public class Robot extends SampleRobot {

  CANTalon FLdrive, FLsteer, RLdrive, RLsteer, FRdrive, FRsteer, RRdrive, RRsteer;
  Joystick driveJoy;
  Gyro gyro;

  public Robot() {
      FLdrive = new CANTalon(1); // Initialize the CanTalonSRX on device 1.
      FLsteer = new CANTalon(2); 
      RLdrive = new CANTalon(3); 
      RLsteer = new CANTalon(4);
      FRdrive = new CANTalon(5);
      FRsteer = new CANTalon(6);
      RRdrive = new CANTalon(7);
      RRsteer = new CANTalon(8);
      driveJoy = new Joystick(1);

      FLdrive.changeControlMode(CANTalon.ControlMode.Voltage);
      RLdrive.changeControlMode(CANTalon.ControlMode.Voltage);
      FRdrive.changeControlMode(CANTalon.ControlMode.Voltage);
      RRdrive.changeControlMode(CANTalon.ControlMode.Voltage);
      FLsteer.changeControlMode(CANTalon.ControlMode.Position);
      RLsteer.changeControlMode(CANTalon.ControlMode.Position);
      FRsteer.changeControlMode(CANTalon.ControlMode.Position);
      RRsteer.changeControlMode(CANTalon.ControlMode.Position);

      FLsteer.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
      RLsteer.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
      FRsteer.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
      RRsteer.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
      FLdrive.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
      RLdrive.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
      FRdrive.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
      RRdrive.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);

      FLsteer.setPID(1, 0, 0);
      RLsteer.setPID(1, 0, 0);
      FRsteer.setPID(1, 0, 0);
      RRsteer.setPID(1, 0, 0);
  }

  public void operatorControl() {
    while (isOperatorControl() && isEnabled()) {

    	double joyMagnitude;
    	double moddedGyroAngle;
    	double moddedJoyMagnitude;
    /*	if((gyro.getAngle() > 360) || (gyro.getAngle() < 0)){
    		gyro.reset();
    	}
    	joyMagnitude = driveJoy.getMagnitude();
    	moddedGyroAngle = (gyro.getAngle() + 180) * (1024/360);
    	moddedJoyMagnitude = ((joyMagnitude * 1023) + 512);
    	
    	trueSwerveMode = SmartDashboard.getBoolean("Swerve Mode Enabled", true);
    	if(trueSwerveMode = false){
    	FLsteer.set(moddedJoyMagnitude);
    	RLsteer.set(moddedJoyMagnitude);
    	FRsteer.set(moddedJoyMagnitude);
    	RRsteer.set(moddedJoyMagnitude);
*/
    	double fwd;
    	double str;
    	double rcw;
    	double temp;
    	double fwd2;
    	double str2;
    	double wheelbase;
    	double trackwidth;
    	double r;
    	double a;
    	double b;
    	double c;
    	double d;
    	double frs, fls, rls, rrs; //Front Right, Front Left, Rear Left, Rear Right Wheel Speeds, respectively
    	double fra, fla, rla, rra; //Wheel Angles
    	double max;
    	fwd = driveJoy.getY();
    	str = driveJoy.getX();
    	rcw = driveJoy.getZ();
    	
    	temp = (fwd*Math.cos(gyro.getAngle())) + str*Math.sin(gyro.getAngle());
    	str2 = (-fwd*Math.sin(gyro.getAngle())) + str*Math.cos(gyro.getAngle());
    	fwd2 = temp;
    	
    	wheelbase = 30; //length of drivebase
    	trackwidth = 24; //width of drivebase
    	r = Math.sqrt((wheelbase*wheelbase) + (trackwidth*trackwidth));
    	
    	a = str2 - rcw * (wheelbase/r);
    	b = str2 + rcw * (wheelbase/r);
    	c = fwd2 - rcw * (trackwidth/r);
    	d = fwd2 + rcw * (trackwidth/r);
    	
    	frs = Math.sqrt(b*b + c*c);
    	fls = Math.sqrt(b*b + d*d);
    	rls = Math.sqrt(a*a + d*d);
    	rrs = Math.sqrt(a*a + c*c);
    	
    	fra = Math.atan2(b,c) * 180/Math.PI;
    	fla = Math.atan2(b,d) * 180/Math.PI;
    	rra = Math.atan2(a,d) * 180/Math.PI;
    	rla = Math.atan2(a,c) * 180/Math.PI;
    	
    	//Because the CANTalon position control uses values from 0 - 1023 
    	//for potentiometer ranges, we must modify the wheel angles to 
    	//compenstate for this. To do this add 180 to the wheel angle to 
    	//get a 0-360 range then multiply by 1023/360 which equals 2.8444...
    	
    	FRsteer.set((fra+180) * (1023/360));
    	FLsteer.set((fla+180) * (1023/360));
    	RLsteer.set((rla+180) * (1023/360));
    	RRsteer.set((rra+180) * (1023/360));
    	
    	//Normalize wheel speeds as stated in Ether's document
    	max = frs;
    	if(fls>max){
    		max = fls;
    	}
    	if(rls>max){
    		max = rls;
    	}
    	if(rrs>max){
    		max = rrs;
    	}
    	if(max>1){
    		frs/=max;
    		fls/=max;
    		rrs/=max;
    		rls/=max;
    	}
    	//Wheel speeds are now 0-1. Not -1 to +1.
    	//Multiply wheel speeds by 12 for voltage control mode
    	FRdrive.set(frs*12);
    	FLdrive.set(fls*12);
    	RRdrive.set(rrs*12);
    	RLdrive.set(rls*12);
    }
    }
  }

