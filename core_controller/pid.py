import time


def PID(roll, pitch, yaw, f, thrust, roll_setpt, pitch_setpt, yaw_setpt):

	global kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw, prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, flag, setpoint, sampleTime

	kp_roll = 70
	ki_roll = 0.0002
	kd_roll = 89

	kp_pitch = kp_roll
	ki_pitch = ki_roll
	kd_pitch = kd_roll
	kp_yaw = 0.8
	ki_yaw = 0.0002
	kd_yaw = 10
	flag = 0

	sampleTime = 5
	setpoint = 0
	err_pitch = float(pitch)*(180 / 3.141592653) - pitch_setpt
 	err_roll = float(roll)*(180 / 3.141592653) - roll_setpt
	err_yaw = float(yaw)*(180/3.14159263) - yaw_setpt

	currTime = time.time()
	#Reset the following variables during the first run only.
	if flag == 0:
		prevTime = 0
		prevErr_roll = 0
		prevErr_pitch = 0
		prevErr_yaw = 0
		pMem_roll = 0
		pMem_pitch = 0
		pMem_yaw = 0
		iMem_roll = 0
		iMem_pitch = 0
		iMem_yaw = 0
		dMem_roll = 0
		dMem_pitch = 0
		dMem_yaw = 0
		flag += 1

	dTime = currTime - prevTime
	dErr_pitch = err_pitch - prevErr_pitch
	dErr_roll = err_roll - prevErr_roll
	dErr_yaw = err_yaw - prevErr_yaw
	

	if(dTime >= sampleTime):
		#propotional
		pMem_roll = kp_roll * err_roll
		pMem_pitch = kp_pitch * err_pitch
		pMem_yaw = kp_yaw * err_yaw
		
		#integral
		iMem_roll += err_pitch * dTime
		iMem_pitch += err_roll * dTime
		iMem_yaw += err_yaw * dTime

		if(iMem_roll > 400): iMem_roll = 400
		if(iMem_roll < -400): iMem_roll = -400
		if(iMem_pitch > 400): iMem_pitch = 400
		if(iMem_pitch < -400): iMem_pitch = -400
		if(iMem_yaw > 400): iMem_yaw = 400
		if(iMem_yaw < -400): iMem_yaw = 400
		
		#derivative
		dMem_roll = dErr_roll / dTime
		dMem_pitch = dErr_pitch / dTime
		dMem_yaw = dErr_yaw / dTime
	
	#Store the current variables into previous variables for the next iteration.
	prevTime = currTime
	prevErr_roll = err_roll
	prevErr_pitch = err_pitch
	prevErr_yaw = err_yaw
	
	#output = proportional + integral + derivative
	output_roll = pMem_roll + ki_roll * iMem_roll + kd_roll * dMem_roll
	output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch
	output_yaw = pMem_yaw + ki_yaw * iMem_yaw + kd_yaw * dMem_yaw 

	

	#br: Back Right
	#bl: Back Left
	#fl: Front Left
	#fr: Front Right
	#Calculate the ESC pulses (1000us - 2000us PWM signal) for each of the motor.
	
	esc_br = 1500 + output_roll + output_pitch - output_yaw
	esc_bl = 1500 + output_roll - output_pitch + output_yaw
	esc_fl = 1500 - output_roll - output_pitch - output_yaw
	esc_fr = 1500 - output_roll + output_pitch + output_yaw
	
	#Limit the ESC pulses to upper limit and lower limit, in case the PID algorithm goes fucking shitstorm.
	if(esc_br > 4000): esc_br = 2000
	if(esc_bl > 4000): esc_bl = 2000
	if(esc_fr > 4000): esc_fr = 2000
	if(esc_fl > 4000): esc_fl = 2000
	
	if(esc_br < 800): esc_br = 1100
	if(esc_bl < 800): esc_bl = 1100
	if(esc_fr < 800): esc_fr = 1100
	if(esc_fl < 800): esc_fl = 1100
	
	#Map the esc values to motor values
	br_motor_vel = ((esc_br - 1500)/25) + thrust
	bl_motor_vel = ((esc_bl - 1500)/25) + thrust
	fr_motor_vel = ((esc_fr - 1500)/25) + thrust
	fl_motor_vel = ((esc_fl - 1500)/25) + thrust

	'''
	if(fl_motor_vel > 70): fl_motor_vel = 70
	if(fr_motor_vel > 70): fr_motor_vel = 70
	if(bl_motor_vel > 70): bl_motor_vel = 70
	if(br_motor_vel > 70): br_motor_vel = 70
	
	
	if(err_roll > 0 && err_pitch > 0):
		fl_motor_vel = 51
		fr_motor_vel = 45
		bl_motor_vel = 51
		br_motor_vel = 51
	elif(err_roll > 0 && err_pitch < 0):
		fl_motor_vel = 45
		fr_motor_vel = 51
		bl_motor_vel = 51
		br_motor_vel = 51
	elif(err_roll < 0 && err_pitch > 0):
	'''	
	f.data = [fr_motor_vel ,-fl_motor_vel ,bl_motor_vel, -br_motor_vel]
	
	return f, err_roll, err_pitch, err_yaw
