/*For use on Aerogeddon, a rocket capable of self-guidance and vertical touchdown under its own propulsion.
This code was intended to provide computation for guidance control surfaces and engine gimballing to maintain an upright orientation on
ascent and descent. However, it can be easily modified to provide many uses such as maintaining the ignition fluid tank's pressure to ensure
a smooth ignition.*/

package com.stormbots;
public class MiniPID{
	private double P=0;
	private double I=0;
	private double D=0;
	private double F=0;
	private double maxIOutput=0;
	private double maxError=0;
	private double errorSum=0;
	private double maxOutput=0; 
	private double minOutput=0;
	private double setpoint=0;
	private double lastActual=0;
	private boolean firstRun=true;
	private boolean reversed=false;
	private double outputRampRate=0;
	private double lastOutput=0;
	private double outputFilter=0;
	private double setpointRange=0;
	public MiniPID(double p, double i, double d){
		P=p; I=i; D=d;
		checkSigns();
		}
	public MiniPID(double p, double i, double d, double f){
		P=p; I=i; D=d; F=f;
		checkSigns();
		}
	public void setP(double p){
		P=p;
		checkSigns();
	public void setI(double i){
		if(I!=0){
			errorSum=errorSum*I/i;
			}
		if(maxIOutput!=0){
			maxError=maxIOutput/i;
		}
		I=i;
		checkSigns();
	}
	public void setD(double d){
		D=d;
		checkSigns();
		}
	public void setF(double f){
		F=f;
		checkSigns();
	}
	public void setPID(double p, double i, double d){
		P=p;D=d;
		setI(i);
		checkSigns();
	}
	public void setPID(double p, double i, double d,double f){
		P=p;D=d;F=f;
		setI(i);
		checkSigns();
	}
	public void setMaxIOutput(double maximum){
		maxIOutput=maximum;
		if(I!=0){
			maxError=maxIOutput/I;
		}
	}
	public void setOutputLimits(double output){
		setOutputLimits(-output,output);
	}
	public void setOutputLimits(double minimum,double maximum){
		if(maximum<minimum)return;
		maxOutput=maximum;
		minOutput=minimum;
		if(maxIOutput==0 || maxIOutput>(maximum-minimum) ){
			setMaxIOutput(maximum-minimum);
		}
	}
	public void  setDirection(boolean reversed){
		this.reversed=reversed;
	}
	public void setSetpoint(double setpoint){
		this.setpoint=setpoint;
	}
	public double getOutput(double actual, double setpoint){
		double output;
		double Poutput;
		double Ioutput;
		double Doutput;
		double Foutput;
		this.setpoint=setpoint;
		if(setpointRange!=0){
			setpoint=constrain(setpoint,actual-setpointRange,actual+setpointRange);
		}
		double error=setpoint-actual;
		Foutput=F*setpoint;
		Poutput=P*error;   
		if(firstRun){
			lastActual=actual;
			lastOutput=Poutput+Foutput;
			firstRun=false;
		}
		Doutput= -D*(actual-lastActual);
		lastActual=actual;
		Ioutput=I*errorSum;
		if(maxIOutput!=0){
			Ioutput=constrain(Ioutput,-maxIOutput,maxIOutput); 
		}    
		output=Foutput + Poutput + Ioutput + Doutput;
		if(minOutput!=maxOutput && !bounded(output, minOutput,maxOutput) ){
			errorSum=error; 
		}
		else if(outputRampRate!=0 && !bounded(output, lastOutput-outputRampRate,lastOutput+outputRampRate) ){
			errorSum=error; 
		}
		else if(maxIOutput!=0){
			errorSum=constrain(errorSum+error,-maxError,maxError);
		}
		else{
			errorSum+=error;
		}

		if(outputRampRate!=0){
			output=constrain(output, lastOutput-outputRampRate,lastOutput+outputRampRate);
		}
		if(minOutput!=maxOutput){ 
			output=constrain(output, minOutput,maxOutput);
			}
		if(outputFilter!=0){
			output=lastOutput*outputFilter+output*(1-outputFilter);
		}

		lastOutput=output;
		return output;
	}
	public double getOutput(){
		return getOutput(lastActual,setpoint);
	}
	public double getOutput(double actual){
		return getOutput(actual,setpoint);
	}
	public void reset(){
		firstRun=true;
		errorSum=0;
	}
	public void setOutputRampRate(double rate){
		outputRampRate=rate;
	}
	public void setSetpointRange(double range){
		setpointRange=range;
	}
	public void setOutputFilter(double strength){
		if(strength==0 || bounded(strength,0,1)){
		outputFilter=strength;
		}
	}
	private double constrain(double value, double min, double max){
		if(value > max){ return max;}
		if(value < min){ return min;}
		return value;
	}  
	private boolean bounded(double value, double min, double max){
		return (min<value) && (value<max);
	}
	private void checkSigns(){
		if(reversed){
			if(P>0) P*=-1;
			if(I>0) I*=-1;
			if(D>0) D*=-1;
			if(F>0) F*=-1;
		}
		else{
			if(P<0) P*=-1;
			if(I<0) I*=-1;
			if(D<0) D*=-1;
			if(F<0) F*=-1;
		}
	}
}