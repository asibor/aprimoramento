// Boukhezzar.cpp : Defines the initialization routines for the DLL.
//

#include "stdafx.h"
#include "Vidal.h"
#include "parameters.h"
#include <windows.h>
#undef GetCurrentTime
EXTERN_C IMAGE_DOS_HEADER __ImageBase;
#include "../External Controller/ExternalControllerApi.h"
#pragma comment (lib,"../ExternalControllerApi.lib")
using namespace GHTurbineInterface;


#define NINT(a) ((a) >= 0.0 ? (int)((a)+0.5) : (int)((a)-0.5))

extern "C"{

static double GenSpeedF;	//Filtered HSS (generator) speed, rad/s.
static double IntSpdErr; //Current integral of speed error w.r.t. time, rad.
static double LastGenTrq; //Commanded electrical generator torque the last time the controller was called, N-m.
static double LastTime; //Last time this DLL was called, sec.
static double LastTimePC; //Last time the pitch  controller was called, sec.
static double LastTimeVS; //Last time the torque controller was called, sec.
static double PitCom[3]; //Commanded pitch of each blade the last time the controller was called, rad.
static double LastGenSpeedF;




double sgn(double value)
{
	if (value<0.0) return -1.0;
	else if (value>0.0) return 1.0;

	return 0.0;
}

static CParameters *g_pParameters= 0;

int __declspec( dllexport ) __cdecl CONTROLLER (const turbine turbine_id)
{
	//get the path of the dll
	char   DllPath[MAX_PATH] = {0};
	GetModuleFileNameA((HINSTANCE)&__ImageBase, DllPath, _countof(DllPath));
	char filename[1024];
	if(!g_pParameters)
	{
		sprintf_s(filename,1024, "%s",DllPath);
		int length= strlen(filename);
		filename[length-3]= 't';
		filename[length-2]= 'x';
		filename[length-1]= 't';
		g_pParameters= new CParameters(filename);
	}
   //Local Variables:

	double Alpha;
	double BlPitch[3]; //Current values of the blade pitch angles, rad.
	double ElapTime; //Elapsed time since the last call to the controller, sec.
	double CornerFreq= g_pParameters->getDouble("FILTER_CORNER_FREQ"); //Corner frequency (-3dB point) in the recursive, single-pole, low-pass filter, rad/s. -- chosen to be 1/4 the blade edgewise natural frequency ( 1/4 of approx. 1Hz = 0.25Hz = 1.570796rad/s)
	double GenSpeed= GetMeasuredGeneratorSpeed(turbine_id);		//Current  HSS (generator) speed, rad/s.
	double GenTrq;	//Electrical generator torque, N-m.

	double PC_DT= GetCommunicationInterval(turbine_id);  //Communication interval for pitch  controller, sec.
	double PC_KI= g_pParameters->getDouble("PC_KI"); //Integral gain for pitch controller at rated pitch (zero), (-).
	double PC_KP= g_pParameters->getDouble("PC_KP"); //Proportional gain for pitch controller at rated pitch (zero), sec.

	double PC_MaxPit= GetMaximumPitchAngle(turbine_id,0); //Maximum pitch setting in pitch controller, rad.
	double PC_MinPit= GetMinimumPitchAngle(turbine_id,0);; //Minimum pitch setting in pitch controller, rad.
	double PC_MaxRat= GetMaximumPitchRate(turbine_id,0); //Maximum pitch  rate (in absolute value) in pitch  controller, rad/s.
	double PC_MinRat= GetMinimumPitchRate(turbine_id,0);;
	double PC_RefSpd= GetReferenceGeneratorSpeedAboveRated(turbine_id); //Desired (reference) HSS speed for pitch controller, rad/s.
	//get this from patrameter file
	double PC_RefRotorSpd= g_pParameters->getDouble("PC_RefRotorSpd");//Rated rotor speed for pitch controller, rad/s

	double PitRate[3]; //Pitch rates of each blade based on the current pitch angles and current pitch command, rad/s.
	double R2D= 57.295780; //Factor to convert radians to degrees.
	double RPS2RPM= 9.5492966; //Factor to convert radians per second to revolutions per minute.
	double SpdErr; //Current speed error, rad/s.
	double Time= GHTurbineInterface::GetCurrentTime(turbine_id); //Current simulation time, sec.
	double VS_DT= GetCommunicationInterval(turbine_id); //Communication interval for torque controller, sec.
	double VS_MaxRat= g_pParameters->getDouble("VS_MaxRat"); //Maximum torque rate (in absolute value) in torque controller, N-m/s.
	double VS_MaxTq= g_pParameters->getDouble("VS_MaxTq"); //Maximum generator torque in Region 3 (HSS side), N-m. -- chosen to be 10% above VS_RtTq = 43.09355kNm
	double VS_MinTq= g_pParameters->getDouble("VS_MinTq"); //Maximum generator torque in Region 3 (HSS side), N-m. -- chosen to be 10% above VS_RtTq = 43.09355kNm
	double VS_RtPwr= g_pParameters->getDouble("VS_RtPwr");// get this value from the parameter file
	double VS_MeasuredPwr= GetMeasuredElectricalPowerOutput(turbine_id);
	double RotorSpeed= GetMeasuredRotorSpeed(turbine_id);
	
	int numBl= GetNumberOfBlades(turbine_id); //Number of blades, (-).

	if (GetPitchControl(turbine_id)==0) //collective pitch control?
	{
		BlPitch[0]= GetCollectivePitchAngle(turbine_id);
		BlPitch[1]= BlPitch[0];
		BlPitch[2]= BlPitch[0];
	}
	else
	{
		BlPitch[0]= GetMeasuredPitchAngle (turbine_id,0);
		BlPitch[1]= GetMeasuredPitchAngle (turbine_id,1);
		BlPitch[2]= GetMeasuredPitchAngle (turbine_id,2);
	}


	if ( Time == 0.0 )  //.TRUE. if we're on the first call to the DLL
	{
		//Inform users that we are using this user-defined routine:
		GenSpeedF  = GenSpeed;                       //This will ensure that generator speed filter will use the initial value of the generator speed on the first pass
		for (int i= 0; i<3; i++)
			PitCom[i]= BlPitch[i];   //This will ensure that the variable speed controller picks the correct control region and the pitch controller picks the correct gain on the first call

		IntSpdErr  = 0.0;//PitCom[1 -1]/( GK*PC_KI );          //This will ensure that the pitch angle is unchanged if the initial SpdErr is zero

		LastTime   = Time; //This will ensure that generator speed filter will use the initial value of the generator speed on the first pass
		LastTimePC = Time - PC_DT; //This will ensure that the pitch  controller is called on the first pass 
		LastTimeVS = Time - VS_DT; //This will ensure that the torque controller is called on the first pass 

		LastGenTrq= GetMeasuredGeneratorTorque(turbine_id);

		LastGenSpeedF= GenSpeedF;
	}

	if (g_pParameters->getDouble("FILTER_GEN_SPEED")!=0.0)
	{
		Alpha     = exp( ( LastTime - Time )*CornerFreq );
		GenSpeedF= ( 1.0 - Alpha )*GenSpeed + Alpha*GenSpeedF;
	}
	else
		GenSpeedF = GenSpeed;


	double powerError= VS_MeasuredPwr - VS_RtPwr;

	//Variable-speed torque control:
	ElapTime = Time - LastTimeVS;

	double d_T_g;
	double d_omega_g;

///IT MIGHT BE WORTH CHANGING THESE PARAMETERS IF IT DOESN'T WORK PROPERLY
	double A= g_pParameters->getDouble("A");
	double K_ALPHA=  g_pParameters->getDouble("K_ALPHA");

 //  if ( (ElapTime+0.001) >= VS_DT )
   {
	   //d(Tg)/dt= (-1/omega_g)*(T_g*(a*omega_g+d_omega_g)-a*P_setpoint + K_alpha*sgn(P_e-P_setpoint))

		d_omega_g= (GenSpeedF-LastGenSpeedF)/ElapTime;

		d_T_g= (-1.0/GenSpeedF)*(LastGenTrq*(A*GenSpeedF+d_omega_g)
			-A*VS_RtPwr + K_ALPHA*sgn(powerError));

		d_T_g= min(max(d_T_g,-VS_MaxRat),VS_MaxRat);
			
		GenTrq= LastGenTrq + d_T_g*ElapTime;

		//Saturate the commanded torque using the maximum torque limit:

		GenTrq  = max(VS_MinTq,min( GenTrq, VS_MaxTq ));   //Saturate the command using the maximum torque limit


		//Reset the values of LastTimeVS and LastGenTrq to the current values:

		LastTimeVS = Time;
		LastGenTrq = GenTrq;
		LastGenSpeedF= GenSpeedF;
	}
	


   //Set the generator contactor status, avrSWAP(35), to main (high speed) 
   //  variable-speed generator, the torque override to yes, and command the
   //  generator torque (See Appendix A of Bladed User's Guide):
	SetGeneratorContactor(turbine_id, 1); //Generator contactor status: 1= main
	SetTorqueOverrideStatus(turbine_id, 0); //Torque override: 0=on
	SetDemandedGeneratorTorque(turbine_id, LastGenTrq);


   //Pitch control:

   //Compute the elapsed time since the last call to the controller:

   ElapTime = Time - LastTimePC;


//   if ( (ElapTime+0.001) >= PC_DT )
   {
		//Compute the current speed error and its integral w.r.t. time; saturate the
		//  integral term using the pitch angle limits:

		SpdErr    = RotorSpeed - PC_RefRotorSpd;                                 //Current speed error
		IntSpdErr = IntSpdErr + SpdErr*ElapTime;                           //Current integral of speed error w.r.t. time

	  
		//Saturate the integral term using the pitch angle limits, converted to integral speed error limits
		if (PC_KI!=0.0)
			IntSpdErr = min( max( IntSpdErr, PC_MinPit/PC_KI ), PC_MaxPit/PC_KI);


		double dBeta= //0.5*KP*SpdErr*(max(min(0.5,SpdErr*10.0),-0.5)+1.0); //in the simulink vidal sent me (?)
			PC_KP*SpdErr + PC_KI*IntSpdErr;						//typical PI
			//0.5*KP*SpdErr*(1.0+sgn(SpdErr));//+KI*IntSpdErr; // in the original paper
		dBeta= min( max( dBeta, PC_MinRat ), PC_MaxRat );

		for (int k=0; k<numBl; k++) //Loop through all blades
		{
			PitCom[k]= BlPitch[k] + dBeta*ElapTime;
			PitCom[k]= min(max(PitCom[k],PC_MinPit),PC_MaxPit);
			PitRate[k] = ( PitCom[k] - BlPitch[k ] )/ElapTime; //Pitch rate of blade K (unsaturated)
			PitRate[k] = min( max( PitRate[k], -PC_MaxRat ), PC_MaxRat ); //Saturate the pitch rate of blade K using its maximum absolute value
			PitCom[k] = BlPitch[k] + PitRate[k]*ElapTime; //Saturate the overall command of blade K using the pitch rate limit

			PitCom[k]  = min( max( PitCom[k ], PC_MinPit ), PC_MaxPit ); //Saturate the overall command using the pitch angle limits         
		}

		LastTimePC = Time;
   }   
      
      
   //Set the pitch override to yes and command the pitch demanded from the last
   //  call to the controller (See Appendix A of Bladed User's Guide):
	if (GetPitchControl(turbine_id)==0) //collective pitch control?
	{
		if (GetPitchActuatorType(turbine_id,0)==0)//0=position, 1=rate
			SetDemandedCollectivePitchAngle(turbine_id,PitCom[0]);
		else SetDemandedCollectivePitchRate(turbine_id,PitRate[0]);
	}
	else
	{
		for (int i= 0; i<numBl; i++)
		{
			if (GetPitchActuatorType(turbine_id,i)==0)//0=position, 1=rate
				SetDemandedPitchAngle(turbine_id,i,PitCom[i]);
			else SetDemandedPitchRate(turbine_id,i,PitRate[i]);
		}
	}

   SetPitchOverrideStatus(turbine_id,0); //pitch override: 0= on

   //Reset the value of LastTime to the current value:
   LastTime = Time;


	return GH_DISCON_SUCCESS;
}

}