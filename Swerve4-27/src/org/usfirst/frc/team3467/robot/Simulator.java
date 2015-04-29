package org.usfirst.frc.team3467.robot;

import javax.swing.JOptionPane;

import de.hardcode.jxinput.Axis;
import de.hardcode.jxinput.JXInputDevice;
import de.hardcode.jxinput.directinput.DirectInputDevice;
import de.hardcode.jxinput.test.AxisListener;

public class Simulator {
/**
 * This program was made by Emile Hamwey, FRC Team 3467 and
 * is based off of Ether's swerve drive kinematics and algorithms.
 * I take no credit for the algorithms and math used in this code.
 * @param args
 * @author Emile Hamwey
 */
	static JXInputDevice joystick;
	static DirectInputDevice logiJoystick; 
	
	public Simulator(){
	logiJoystick = new DirectInputDevice(0); 
	}
	public static void main(String[] args) {
	   	double fwd;
    	double str;
    	double rcw;
    	double getAngle;
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
    	String getfwd = JOptionPane.showInputDialog("Please Input negated Y joystick value");

    	String getstr = JOptionPane.showInputDialog("Please Input X joystick value (-1 to 1)");
    	String getrcw = JOptionPane.showInputDialog("Please Input Rotational Joy Value (-1 to 1)");
    	String getgyro = JOptionPane.showInputDialog("Please Input Gyro Angle (0-360)");
    	fwd = Double.parseDouble(getfwd);
    	str = Double.parseDouble(getstr);
    	rcw = Double.parseDouble(getrcw);
    	getAngle = Double.parseDouble(getgyro);
    	
    	temp = (fwd*Math.cos(getAngle)) + str*Math.sin(getAngle);
    	str2 = (-fwd*Math.sin(getAngle)) + str*Math.cos(getAngle);
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
    	
    	// we must modify the wheel angles to 
    	//compenstate for this. To do this add 180 to the wheel angle to 
    	//get a 0-360 range 
    	
    	System.out.println("Front Right Wheel Angle" + ((fra)));
    	System.out.println("Front Left Wheel Angle"  + ((fla)));
    	System.out.println("Rear Right Wheel Angle"  + ((rla)));
    	System.out.println("Rear Left Wheel Angle"   + ((rra)));
    	
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
    	System.out.println("Front Right Wheel Speed"+ (frs*12));
    	System.out.println("Front Left Wheel Speed" + (fls*12));
    	System.out.println("Rear Right Wheel Speed" + (rrs*12));
    	System.out.println("Rear Left Wheel Speed"  + (rls*12));
	}

}
