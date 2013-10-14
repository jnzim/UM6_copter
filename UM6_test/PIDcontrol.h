/*
 * PIDcontrol.h
 *
 * Created: 6/13/2013 7:11:19 PM
 *  Author: Justin
 */ 


#ifndef PIDCONTROL_H_
#define PIDCONTROL_H_


#define PID_MAX_GAIN 500
#define PID_MIN_GAIN -500
#define MAX_TOTAL_ERROR 32766			
#define MIN_TOTAL_ERROR -32767

//  prototypes
//void pid( PID_data_t * pid_data);
int16_t Limit_value_signed ( int16_t, int16_t );
//  global variables
 int16_t slope, pid_total, p_term, i_term, d_term;
typedef struct PID_data
{
	int16_t Kp_rate, Ki_rate, Kd_rate;
	int16_t Kp, Ki, Kd;
	int16_t attitude_feedback;
	int16_t rate;
	int16_t attitude_command;
	int16_t attitude_pid_out;
	int16_t rate_pid_out;
	int16_t attitude_error;
	int16_t rate_error;
	int16_t previousRateError0;
	int16_t windupGuard;
	int16_t attitude_total_error;
	int16_t rate_total_error;
	int16_t previousError0;
	int16_t previousError1;
	int16_t previousError2;
	int16_t previousPosition0;
	int16_t previousPosition1;
	int16_t previousPosition2;
	
} PID_data_t ;

void pid_rate(PID_data_t * pid_data) 
{

	// save the last error calculating the integral and derivative term
	pid_data->previousRateError0 = pid_data->rate_error;
	// the rate error is the output of the attitude pid  minus the rate which is calculated by taking the
	//  derivative of the attitude.  This is strange, the units are not consistent
	pid_data->rate_error = (pid_data->attitude_pid_out - pid_data->rate);
	
	//  calculate the integral of the rate,  this is just position so we should really use the IMU data, duh
	pid_data->rate_total_error = pid_data->previousRateError0 + pid_data->rate_error;
	
	p_term = (pid_data->rate_error * pid_data->Kp_rate)/10;
	
	i_term = (pid_data->rate_total_error  * pid_data->Ki_rate)/10;
	
	d_term = (pid_data->rate_error * pid_data->Kd)/10;
	
	pid_data->rate_pid_out = p_term + i_term + d_term;
}

//  pid  position control loop
void pid_attitude(PID_data_t * pid_data) 
{
	////  save the last error calculation so we can calculate the derivative
	//pid_data->previousError0 = pid_data->previousError1;
	//pid_data->previousError1 = pid_data->previousError2;
	//pid_data->previousError2 = pid_data->error;
	//pid_data->previousError0 = pid_data->error;
	////  calculate the new error
	////10 - 23 = -13
	
	pid_data->attitude_error = (pid_data->attitude_command - pid_data->attitude_feedback);
	////  calculate the slope (dt = 1)
	//slope = (pid_data->error - pid_data->previousError2)  + (pid_data->previousError2 - pid_data->previousError1)
	//+ (pid_data->previousError1 - pid_data->previousError0);
	
	//sum the error for the current and last sample (dt = 1)
	//pid_data->total_error = (pid_data->previousError0 + pid_data->previousError1 + pid_data->previousError2 + pid_data->error);
	//pid_data->total_error = Limit_value_signed(pid_data->total_error);
	
	//calculate proportional term
	// still rolls over at small angles if gain is large
	//(360*105)/100
	//p_term = Limit_value_signed(((pid_data->error  *  pid_data->Kp)/100), pid_data->error);
	p_term = (pid_data->attitude_error  *  pid_data->Kp);	
	
	//calculate integral term
	//i_term = Limit_value_signed((pid_data->total_error  * pid_data->Ki)/100);
	
	//calculate derivative  term
	d_term = (pid_data->rate * pid_data->Kd);
	
	// calculate the pid output
	pid_total = p_term + d_term;
	
	
	pid_data->attitude_pid_out =  d_term + p_term;

	
}

void CalculateRate(PID_data_t * pid_data)
{
	//  this is a rolling set of attitude errors
	pid_data->previousPosition0 = pid_data->previousPosition1;
	pid_data->previousPosition1 = pid_data->previousPosition2;
	pid_data->previousPosition2 = pid_data->attitude_feedback;


	//  calculate the rate, the 
	pid_data->rate = (pid_data->attitude_feedback - pid_data->previousPosition2)  + (pid_data->previousPosition2 - pid_data->previousPosition1)
	+ (pid_data->previousPosition1 - pid_data->previousPosition0);
	

}

//Limit_value_signed: limit value of signed Variable between Min_Value and Max_Value
//Supply variable Address, Min and Max value within she has to be limited
//Function return new value if limit take place
int16_t Limit_value_signed ( int16_t Variable,int16_t error)
{
	if (error > 0  && Variable < 0)
	{
		Variable = MAX_TOTAL_ERROR;
	}

	if (error < 0  && Variable >0)
	{
		Variable = MIN_TOTAL_ERROR;
	}
	//if (error < 0  && Variable < MIN_TOTAL_ERROR)
	//{
		//Variable = MAX_TOTAL_ERROR;
	//}
	//if (error > 0  && Variable < MIN_TOTAL_ERROR)
	//{
		//Variable = MAX_TOTAL_ERROR;
	//}
	return Variable;
}


////Limit_value_unsigned: limit value of unsigned Variable between Min_Value and Max_Value
////Supply variable Address, Min and Max value within she has to be limited
////Function return new value if limit take place
//int unsigned Limit_value_unsigned ( int Variable,int Min_Value,int Max_Value)
//{
	//if (Variable < Min_Value)
	//{
		//Variable = Min_Value;
	//}
	//if (Variable > Max_Value)
	//{
		//Variable = Max_Value;
	//}
	//return Variable;
//}
//
////GetPError: Calculate proportional Term for Pid algorithm
////Limiting value between Local_Limit_Low and Local_Limit_High
//void  GetPError (int Local_Error, int Local_Limit_Low, int Local_Limit_High)
//{
	//PTerm = (Kp * Local_Error)/10;
	//if (Local_Error > 0)
	//{    
		//if (PTerm < 0) 
		//{ 
			//PTerm = 0x7FFF;
		//}    
	//}
	//if (Local_Error < 0)
	//{   
		 //if (PTerm > 0) 
		 //{ 
			 //PTerm = 0x8000;
		//}    
	//}
	//PTerm = Limit_value_signed (PTerm, Local_Limit_Low , Local_Limit_High);
//}
//
//
 //
 ////GetIError: Calculate Integral Term for Pid algorythm
 ////Limiting value between Local_Limit_Low and Local_Limit_High
	//void GetIError (int Local_Error, int Local_Limit_Low, int Local_Limit_High)
	//{
		//IInstant = (Ki * Local_Error)/10;
		//if (Local_Error > 0)
		//{
			//if (IInstant < 0)
			//{
				//IInstant = 0x7FFF;
			//}
		//}
		//if (Local_Error < 0)
		//{
			//if (IInstant > 0)
			//{
				//IInstant = 0x8000;
			//}
		//}
		//ITerm = IInstant + ITerm;
		//ITerm = Limit_value_signed (ITerm, Local_Limit_Low, Local_Limit_High);
	//}
 //
 ////GetPError: Calculate Derivative Term for Pid algorythm
 ////Limiting value between Local_Limit_Low and Local_Limit_High
//void GetDError (int Local_Error, int Local_Limit_Low, int Local_Limit_High)
		//{
			//DTerm = ((Local_Error - OldError)*Kd)/10;
			//if ((Local_Error - OldError) > 0)
			//{
				//if (DTerm < 0)
				//{
					//DTerm = 0x7FFF;
				//}
			//}
			//if ((Local_Error - OldError) < 0)
			//{    if (DTerm > 0) { DTerm = 0x8000;}    }
			//OldError = Error;
			//DTerm = Limit_value_signed (DTerm, Local_Limit_Low, Local_Limit_High);
 //}
//
//
//void Pid_Function    (void)
//{
	//if (PIR1bits.TMR1IF == 1)                                //Monitor for 1ms timebase IRQ (TMR1IRQ)
	//{
		//PIR1bits.TMR1IF=0;                                        //Reset IRQ Flag
		//WriteTimer1 (K_TMR1);                                    //Recharge TMR1
		//
		////Update Output-----------
		//
		//
		//
		//Pid_Out = PTerm + ITerm + DTerm;                        //Update out at once to avoid jitter
		//Pid_Out = Limit_value_signed (Pid_Out, b, a);            //Limit Output
		//PutOutput(Pid_Out);                                        //Call output function
		//
		////Update Input, get Error--------
		//Error = Reference - GetFeedback();                            //get Error
		//
		//if (PidCounter == 0)                                    //PID Period Counter Run out?
		//{                                                    //If yes, do Proportional calc
			//GetPError ( Error, -PLimit , PLimit);                //Call function, supplying data
			//PidCounter = PidPeriod;                                //Recharge Period counter
			//if (IPeriod > 0)                                        //Is IPeriod > 0 (ITerm required?)
			//{                                                    //If yes do I calculation
				//if (ICounter == 0)                                        //I Period Counter Run out?
				//{                                                    //If yes, do Integrative calc
					//GetIError ( Error, b , ILimit);                        //Call funtion, supplying data
					//ICounter = IPeriod;                                    //Recharge period counter
				//}
				//else                                                        //If not, decrease I counter
				//{
					//ICounter--;
				//}
			//}
			//else                                                    //If not, then ITerm=0
			//{
				//ITerm = 0;
			//}
			//if (DPeriod > 0)                                        //Is DPeriod > 0 (DTerm required?)
			//{                                                    //If yes do D calculation
				//if (DCounter == 0)                                        //D Period Counter Run out?
				//{                                                    //If yes, do Derivative calc
					//GetDError ( Error, -DLimit , DLimit);                //Call funtion, supplying data
					//DCounter = DPeriod;                                    //Recharge period counter
				//}
				//else
				//{
					//DCounter--;                                            //If not, decrease D counter
				//}
			//}
			//else
			//{
				//DTerm = 0;                                            //If not, then DTerm=0
			//}
		//}
		//else                                                    //If not, decrease Pid counter
		//{
			//PidCounter--;
		//}
	//}
//}

#endif /* PIDCONTROL_H_ */