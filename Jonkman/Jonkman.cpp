// Jonkman.cpp : Defines the initialization routines for the DLL.
//

#include "stdafx.h"
#include "Jonkman.h"
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
static double VS_Slope15;//Torque/speed slope of region 1 1/2 cut-in torque ramp , N-m/(rad/s).
static double VS_Slope25;//Torque/speed slope of region 2 1/2 induction generator, N-m/(rad/s).
static double VS_SySp; //Synchronous speed of region 2 1/2 induction generator, rad/s.
static double VS_TrGnSp; //Transitional generator speed (HSS side) between regions 2 and 2 1/2, rad/s.

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

	double Alpha; //Current coefficient in the recursive, single-pole, low-pass filter, (-).
	double BlPitch[3]; //Current values of the blade pitch angles, rad.
	double ElapTime; //Elapsed time since the last call to the controller, sec.
	double CornerFreq= g_pParameters->getDouble("FILTER_CORNER_FREQ"); //Corner frequency (-3dB point) in the recursive, single-pole, low-pass filter, rad/s. -- chosen to be 1/4 the blade edgewise natural frequency ( 1/4 of approx. 1Hz = 0.25Hz = 1.570796rad/s)
	double GenSpeed= GetMeasuredGeneratorSpeed(turbine_id);		//Current  HSS (generator) speed, rad/s.

	double GenTrq;	//Electrical generator torque, N-m.
	double GK;	//Current value of the gain correction factor, used in the gain scheduling law of the
				//pitch controller, (-).

	double PC_DT= GetCommunicationInterval(turbine_id);  //Communication interval for pitch  controller, sec.
	double PC_KI= g_pParameters->getDouble("PC_KI"); //Integral gain for pitch controller at rated pitch (zero), (-).
	double PC_KK= g_pParameters->getDouble("PC_KK"); //Pitch angle where the the derivative of the aerodynamic power w.r.t. pitch has increased by a factor of two relative to the derivative at rated pitch (zero), rad.
	double PC_KP= g_pParameters->getDouble("PC_KP"); //Proportional gain for pitch controller at rated pitch (zero), sec.

	double PC_MaxPit= GetMaximumPitchAngle(turbine_id,0); //Maximum pitch setting in pitch controller, rad.
	double PC_MinPit= GetMinimumPitchAngle(turbine_id,0);; //Minimum pitch setting in pitch controller, rad.
	double PC_MaxRat= GetMaximumPitchRate(turbine_id,0); //Maximum pitch  rate (in absolute value) in pitch  controller, rad/s.
	double PC_MinRat= GetMinimumPitchRate(turbine_id,0);;
	double PC_RefSpd= GetReferenceGeneratorSpeedAboveRated(turbine_id); //Desired (reference) HSS speed for pitch controller, rad/s.
	double PC_RefRotorSpd= g_pParameters->getDouble("PC_RefRotorSpd");//Rated rotor speed for pitch controller, rad/s

	double PitComI; //Integral term of command pitch, rad.
	double PitComP; //Proportional term of command pitch, rad.
	double PitComT; //Total command pitch based on the sum of the proportional and integral terms, rad.
	double PitRate[3]; //Pitch rates of each blade based on the current pitch angles and current pitch command, rad/s.
	double R2D= 57.295780; //Factor to convert radians to degrees.
	double RPS2RPM= 9.5492966; //Factor to convert radians per second to revolutions per minute.
	double SpdErr; //Current speed error, rad/s.
	double Time= GHTurbineInterface::GetCurrentTime(turbine_id); //Current simulation time, sec.
	double TrqRate; //Torque rate based on the current and last torque commands, N-m/s.
	double VS_CtInSp= g_pParameters->getDouble("VS_CtInSp"); //Transitional generator speed (HSS side) between regions 1 and 1 1/2, rad/s.
	double VS_DT= GetCommunicationInterval(turbine_id);; //Communication interval for torque controller, sec.
	double VS_MaxRat= g_pParameters->getDouble("VS_MaxRat"); //Maximum torque rate (in absolute value) in torque controller, N-m/s.
	double VS_MaxTq= g_pParameters->getDouble("VS_MaxTq"); //Maximum generator torque in Region 3 (HSS side), N-m. -- chosen to be 10% above VS_RtTq = 43.09355kNm
	double VS_MinTq= g_pParameters->getDouble("VS_MinTq"); //Maximum generator torque in Region 3 (HSS side), N-m. -- chosen to be 10% above VS_RtTq = 43.09355kNm
	double VS_Rgn2K= g_pParameters->getDouble("VS_Rgn2K"); //Generator torque constant in Region 2 (HSS side), N-m/(rad/s)^2.
	double VS_Rgn2Sp= g_pParameters->getDouble("VS_Rgn2Sp"); //Transitional generator speed (HSS side) between regions 1 1/2 and 2, rad/s.
	double VS_Rgn3MP= g_pParameters->getDouble("VS_Rgn3MP"); //Minimum pitch angle at which the torque is computed as if we are in region 3 regardless of the generator speed, rad. -- chosen to be 1.0 degree above PC_MinPit
	double VS_RtGnSp= g_pParameters->getDouble("VS_RtGnSp"); //Rated generator speed (HSS side), rad/s. -- chosen to be 99% of PC_RefSpd
	double VS_RtPwr= g_pParameters->getDouble("VS_RtPwr");
	double VS_MeasuredPwr= GetMeasuredElectricalPowerOutput(turbine_id);
	double VS_SlPc= g_pParameters->getDouble("VS_SlPc");//Rated generator slip percentage in Region 2 1/2, %.

	int numBl= GetNumberOfBlades(turbine_id);

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


   //Read any External Controller Parameters specified in the User Interface
   //  and initialize variables:

	if ( Time == 0.0 )  //.TRUE. if we're on the first call to the DLL
	{
		//Inform users that we are using this user-defined routine:

		//Determine some torque control parameters not specified directly:

	   VS_SySp    = VS_RtGnSp/( 1.0 +  0.01*VS_SlPc );
	   VS_Slope15 = ( VS_Rgn2K*VS_Rgn2Sp*VS_Rgn2Sp )/( VS_Rgn2Sp - VS_CtInSp );
	   VS_Slope25 = ( VS_RtPwr/VS_RtGnSp           )/( VS_RtGnSp - VS_SySp   );
	   if ( VS_Rgn2K == 0.0 )  //.TRUE. if the Region 2 torque is flat, and thus, the denominator in the ELSE condition is zero
		  VS_TrGnSp = VS_SySp;
	   else                          //.TRUE. if the Region 2 torque is quadratic with speed
		  VS_TrGnSp = ( VS_Slope25 - sqrt( VS_Slope25*( VS_Slope25 - 4.0*VS_Rgn2K*VS_SySp ) ) )/( 2.0*VS_Rgn2K );
	   


		GenSpeedF  = GenSpeed;                       //This will ensure that generator speed filter will use the initial value of the generator speed on the first pass
		for (int i= 0; i<3; i++) PitCom[i]= BlPitch[i];   //This will ensure that the variable speed controller picks the correct control region and the pitch controller picks the correct gain on the first call
		GK         = 1.0/( 1.0 + PitCom[1 -1]/PC_KK );   //This will ensure that the pitch angle is unchanged if the initial SpdErr is zero
		IntSpdErr  = PitCom[1 -1]/( GK*PC_KI );          //This will ensure that the pitch angle is unchanged if the initial SpdErr is zero

		LastTime   = Time; //This will ensure that generator speed filter will use the initial value of the generator speed on the first pass
		LastTimePC = Time - PC_DT; //This will ensure that the pitch  controller is called on the first pass 
		LastTimeVS = Time - VS_DT; //This will ensure that the torque controller is called on the first pass 

	}


	if (g_pParameters->getDouble("FILTER_GEN_SPEED")!=0.0)
	{
		Alpha     = exp( ( LastTime - Time )*CornerFreq );
		GenSpeedF= ( 1.0 - Alpha )*GenSpeed + Alpha*GenSpeedF;
	}
	else
		GenSpeedF = GenSpeed;
   //Variable-speed torque control:
   //Compute the elapsed time since the last call to the controller:
   ElapTime = Time - LastTimeVS;


   //Only perform the control calculations if the elapsed time is greater than
   //  or equal to the communication interval of the torque controller:
   //NOTE: Time is scaled by OnePlusEps to ensure that the contoller is called
   //      at every time step when VS_DT = DT, even in the presence of
   //      numerical precision errors.

//   if (  (ElapTime+0.001) >= VS_DT )
   {
   //Compute the generator torque, which depends on which region we are in:

	  if ( (   GenSpeedF >= VS_RtGnSp ) || (  PitCom[1 -1] >= VS_Rgn3MP ) )   //We are in region 3 - power is constant
		 GenTrq = VS_RtPwr/GenSpeedF;
	  else if ( GenSpeedF <= VS_CtInSp )                                      //We are in region 1 - torque is zero
		 GenTrq = 0.0;
	  else if ( GenSpeedF <  VS_Rgn2Sp )                                      //We are in region 1 1/2 - linear ramp in torque from zero to optimal
		 GenTrq = VS_Slope15*( GenSpeedF - VS_CtInSp );
	  else if ( GenSpeedF <  VS_TrGnSp )                                      //We are in region 2 - optimal torque is proportional to the square of the generator speed
		 GenTrq = VS_Rgn2K*GenSpeedF*GenSpeedF;
	  else                                                                       //We are in region 2 1/2 - simple induction generator transition region
		 GenTrq = VS_Slope25*( GenSpeedF - VS_SySp   );

		//Saturate the commanded torque using the maximum torque limit:
		GenTrq  = max(VS_MinTq,min( GenTrq, VS_MaxTq ));   //Saturate the command using the maximum torque limit

		//Saturate the commanded torque using the torque rate limit:
		if ( Time == 0.0 )  LastGenTrq = GenTrq; //Initialize the value of LastGenTrq on the first pass only

		TrqRate = ( GenTrq - LastGenTrq )/ElapTime; //Torque rate (unsaturated)
		TrqRate = min( max( TrqRate, -VS_MaxRat ), VS_MaxRat ); //Saturate the torque rate using its maximum absolute value
		GenTrq  = LastGenTrq + TrqRate*ElapTime;                 //Saturate the command using the torque rate limit

		//Reset the values of LastTimeVS and LastGenTrq to the current values:
		LastTimeVS = Time;
		LastGenTrq = GenTrq;
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


//	if ( (ElapTime+0.001) >= PC_DT )
	{
		//Compute the gain scheduling correction factor based on the previously
		//  commanded pitch angle for blade 1:
		GK = 1.0/( 1.0 + PitCom[0]/PC_KK );

		//Compute the current speed error and its integral w.r.t. time; saturate the
		//  integral term using the pitch angle limits:
		SpdErr    = GenSpeedF - PC_RefSpd;                                 //Current speed error
		IntSpdErr = IntSpdErr + SpdErr*ElapTime;                           //Current integral of speed error w.r.t. time
		//Saturate the integral term using the pitch angle limits, converted to integral speed error limits
		IntSpdErr = min( max( IntSpdErr, PC_MinPit/( GK*PC_KI ) ), PC_MaxPit/( GK*PC_KI ));
	  
		//Compute the pitch commands associated with the proportional and integral
		//  gains:
		PitComP   = GK*PC_KP*   SpdErr; //Proportional term
		PitComI   = GK*PC_KI*IntSpdErr; //Integral term (saturated)


		//Superimpose the individual commands to get the total pitch command;
		//  saturate the overall command using the pitch angle limits:
		PitComT   = PitComP + PitComI;                                     //Overall command (unsaturated)
		PitComT   = min( max( PitComT, PC_MinPit ), PC_MaxPit );           //Saturate the overall command using the pitch angle limits


		for (int k=1; k<=numBl; k++) //Loop through all blades
		{
		 PitRate[k -1] = ( PitComT - BlPitch[k -1] )/ElapTime; //Pitch rate of blade K (unsaturated)
		 PitRate[k -1] = min( max( PitRate[k -1], PC_MinRat ), PC_MaxRat ); //Saturate the pitch rate of blade K using its maximum absolute value
		 PitCom [k -1] = BlPitch[k -1] + PitRate[k -1]*ElapTime; //Saturate the overall command of blade K using the pitch rate limit

		 PitCom[k -1]  = min( max( PitCom[k -1], PC_MinPit ), PC_MaxPit ); //Saturate the overall command using the pitch angle limits         
		}

		//Reset the value of LastTimePC to the current value:

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