AF_DCMotor motor(2, MOTOR12_2KHZ);
private int rpmcount = 0;
private int rpm = 0;
private int lastmillis = 0;
private int pid_speed = 0;
private float kp = 2.25F;
private float ki = .01;
private float kd = 0.005F;
private int prev_error = 0;
private double pwm_sig = 0;
private int error = 0;
private float desired_rpm = 80F;
private int diff = 0;
private void setup()
SerialBegin(9600);
attachInterrupt(0, rpm_fan, FALLING);
motorRun(FORWARD);

private void loop()
{
  if (millis() - lastmillis == 1000)
  {
	detachInterrupt(0);
	rpm = (rpmcount * 3) / 6;
	Serial.print("RPM =\t");
	Serial.print(rpm);
	rpmcount = 0;
	lastmillis = millis();
	attachInterrupt(0, rpm_fan, FALLING);
  }
  error = (int)desired_rpm - rpm;
  diff += error;
  pid_speed = (int)kp * error;
  + ki * diff;
  pid_speed = (int)pid_speed;
  Serial.print(pid_speed);
  Serial.print("       ");
  if (pid_speed > 0)
  {
	if (pid_speed < 255)
	{
	  motor.setSpeed(255);
	}
	else
	{
	  motor.setSpeed(pid_speed);
	}
  }
  if (pid_speed < 0)
  {
	if (pid_speed < -255)
	{
	  motor.setSpeed(0);
	}
	else
	{
	  pid_speed = Math.abs(pid_speed);
	  motor.setSpeed(255 - pid_speed);
	}
  }
  Serial.print("      ");
  Serial.println(rpm);
}
private void rpm_fan()
	{
	rpmcount++;
	}
}