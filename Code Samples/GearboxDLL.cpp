/*
This is a simple example of an external gearbox DLL written in C++.  It reads the user-defined parameter values
defined in Bladed from the text file created by Bladed.  It uses the gearbox ratio from Bladed, and
includes stiffness and damping in the gearbox.  Shaft brake and loss torques from Bladed are included,
together with stick/slip discontinuity handling for the brake, which is assumed to be on the high speed shaft.
*/

#include <stdio.h>
#include <string.h>
#define ABS(x) ((x) >= 0 ? (x) : (-x))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

//Local procedures defined bellow
int ReadOneParameter(FILE *file, char *Name, double *value, char *Msg);
void EOM (double *dTime, double *dStates, double *dTorques, double dGearRatio, double dStiffness,
		  double dDamping, double dInertiaLSS, double dInertiaHSS, double *dDerivs, double dFricTrq);
void EOM_Stopped (double *dTime, double *dStates, double *dTorques, double dGearRatio, double dStiffness,
		  double dDamping, double dInertiaLSS, double dInertiaHSS, double *dDerivs, double dTheta2);

static double cdGearRatio, cdStiffness, cdDamping, cdInertiaLSS, cdInertiaHSS, cdV0, cdTheta2Stopped, cdVelTol, cdFricTol;
void GetStatics(double *dGearRatio, double *dStiffness, double *dDamping, double *dInertiaLSS, double *dInertiaHSS,
				double *dV0, double *dTheta2Stopped, double *dVelTol, double *dFricTol)
{
	*dGearRatio = cdGearRatio;
	*dStiffness = cdStiffness;
	*dDamping = cdDamping;
	*dInertiaLSS = cdInertiaLSS;
	*dInertiaHSS = cdInertiaHSS;
	*dV0 = cdV0;
	*dTheta2Stopped = cdTheta2Stopped;
	*dVelTol = cdVelTol;
	*dFricTol = cdFricTol;
}
void PutStatics(double dGearRatio, double dStiffness, double dDamping, double dInertiaLSS, double dInertiaHSS,
				double dV0, double dTheta2Stopped, double dVelTol, double dFricTol)
{
	cdGearRatio = dGearRatio;
	cdStiffness = dStiffness;
	cdDamping = dDamping;
	cdInertiaLSS = dInertiaLSS;
	cdInertiaHSS = dInertiaHSS;
	cdV0 = dV0;
	cdTheta2Stopped = dTheta2Stopped;
	cdVelTol = dVelTol;
	cdFricTol = dFricTol;
}

extern "C"
{
	void __declspec(dllexport) __cdecl
	DLL_GBX(int *iArg1, double *dArg2, int *iArg3, char *cArg4,
			 double *dArg5, double *dArg6, double *dArg7, char *cArg8);
}

void __declspec(dllexport) __cdecl
	DLL_GBX(int *iArg1, double *dArg2, int *iArg3, char *cArg4,
			 double *dArg5, double *dArg6, double *dArg7, char *cArg8)
{
	double dGearRatio, dStiffness, dDamping, dInertiaLSS, dInertiaHSS, dV0, dTheta2Stopped, dVelTol, dFricTol;
	static int Nx, Nout;
	int iCallType, ii;
	char *cInFile, *cVerifyFile;
	FILE *file;
	static int iStopped;
	double dLssPos, dLssVel, dHssPos, dHssVel;


	//==============================================================================================
	// Call to initialise the DLL model (using any constants which might come from the Bladed model)
	// and return the number of integrator states and output variables.
	//==============================================================================================
	iCallType = iArg1[1];

	if (iCallType == 1)
	{
		cdGearRatio = dArg5[0];
		Nx = 4;		//No of states
		Nout = 8;	//No of outputs
		iArg3[0] = Nx;
		iArg3[1] = Nout;
		//Open parameter file
		cInFile = cArg4; cVerifyFile = NULL;
		for (ii=0; ii < iArg1[3]; ii++)
		{
			if (cArg4[ii] == ';')
			{
				cArg4[ii] = '\0';
				if (cVerifyFile == NULL) cVerifyFile = &(cArg4[ii+1]);
			}
		}
		if (strlen(cInFile) > 0)
		{
			if ((file=fopen(cInFile,"r")) == NULL)
			{
				strcpy(cArg8,"Could not open parameter file: ");
				cArg8 = strcat(cArg8,cInFile);
				iArg1[8] = -1;
				return;
			}
			{
				if ((iArg1[8] = ReadOneParameter(file, "STIFFNESS", &cdStiffness, cArg8)) < 0) return;
				if ((iArg1[8] = ReadOneParameter(file, "DAMPING", &cdDamping, cArg8)) < 0) return;
				if ((iArg1[8] = ReadOneParameter(file, "INERTIALSS", &cdInertiaLSS, cArg8)) < 0) return;
				if ((iArg1[8] = ReadOneParameter(file, "INERTIAHSS", &cdInertiaHSS, cArg8)) < 0) return;
			}
		}
		cdVelTol = 1.0E-5;
		cdFricTol = 1.0E-5;
		return;
	}

	//==============================================================================================
	// Call to return the names and absolute tolerances of integrator states required by the DLL.
	//==============================================================================================

	if (iCallType == 2)
	{
		//Initialisation flags
		iArg3[1] = 0;
		iArg3[2] = 1;
		iArg3[3] = 1;
		iArg3[4] = 1;
		//State names
		strcpy(cArg4,"Gearbox state 1:N;Gearbox state 2:N;Gearbox state 3:N;Gearbox state 4:N;");
		//State absolute tolerances
		dArg5[0] = 1.0E-4;
		dArg5[1] = 1.0E-4;
		dArg5[2] = 1.0E-4;
		dArg5[3] = 1.0E-4;
		return;
	}

	GetStatics(&dGearRatio, &dStiffness, &dDamping, &dInertiaLSS, &dInertiaHSS, &dV0, &dTheta2Stopped, &dVelTol, &dFricTol);

	//==============================================================================================
	// Call to return the names and units of variables from the DLL to be output in time history
	// output from Bladed (i.e. yout from the second state-space equation yout = C.x + D.u).
	//==============================================================================================

	if (iCallType == 3)
	{
		strcpy(cArg4,"LSSpos:A;LSSvel:A/T;HSSpos:A;HSSvel:A/T;LSStrq:FL;HSStrq:FL;LSS tooth torque:FL;Friction torque:FL;");
	}

	//==============================================================================================
	// Call to execute one step of initial conditions iteration and return the initial values of
	// the states.
	//==============================================================================================

	if (iCallType == 4)
	{
		double dLssTrq, dFricTrq;
		dLssPos = dArg5[0];
		dHssVel = dArg5[1];
		dLssVel = dHssVel/dGearRatio;
		dFricTrq = dArg5[3] + dArg5[4];
		if (dHssVel > 0)
		{
			dLssTrq = (dArg5[2] + dFricTrq)*dGearRatio;		//LSS trq - apply any loss torque at HSS
			iStopped = 0;
		}
		else if (dHssVel < 0)
		{
			dLssTrq = (dArg5[2] - dFricTrq)*dGearRatio;		//LSS trq - apply any loss torque at HSS
			iStopped = 0;
		}
		else
		{
			dLssTrq = dArg6[2];
			if (dLssTrq > dFricTrq*dGearRatio) dLssTrq = dFricTrq*dGearRatio;
			if (dLssTrq < -dFricTrq*dGearRatio) dLssTrq = -dFricTrq*dGearRatio;
			iStopped = 1;
		}
		dHssPos = dGearRatio*(dLssPos - dLssTrq/dStiffness);	//HSS pos
		dArg6[0] = dHssPos;
		dArg6[1] = dLssVel;
		dArg6[2] = dLssTrq;

		if (iArg3[3] == 1)	//Initial values of states
		{
			dArg7[0] = dLssPos;
			dV0 = dLssVel;	//Use the initial velocity as the nominal velocity (see EOM below).
			dArg7[1] = dLssVel - dV0;
			dArg7[2] = dHssPos - dGearRatio*dLssPos;
			dArg7[3] = dHssVel - dGearRatio*dLssVel;
			//States 3 & 4 disabled if stopped:
			if (iStopped == 1)
			{
				iArg3[6] = 0;
				iArg3[7] = 0;
				dTheta2Stopped = dHssPos;
			}
		}
	}

	//==============================================================================================
	// Call to pass to the DLL the state values and any required variables derived from Bladed
	// states, and return the state derivatives (xdot) calculated by the DLL.
	//==============================================================================================

	if (iCallType == 5)
	{
		if (iStopped)
			EOM_Stopped(dArg2, dArg5, dArg6, dGearRatio, dStiffness, dDamping, dInertiaLSS, dInertiaHSS, dArg7, dTheta2Stopped);
		else
		{
			double dFricTrq;
			dFricTrq = dArg6[2] + dArg6[3];
			dHssVel = dArg5[3] + dGearRatio*(dArg5[1] + dV0);
			if (dHssVel > 0) dFricTrq = -dFricTrq;
			EOM(dArg2, dArg5, dArg6, dGearRatio, dStiffness, dDamping, dInertiaLSS, dInertiaHSS, dArg7, dFricTrq);
		}
		dArg7[4] = dArg7[1];	// LSS acceleration
		dArg7[5] = dArg7[3]+ dGearRatio * dArg7[1];	// HSS acceleration
	}

	//==============================================================================================
	// Call to pass to the DLL the state values and any other variables required from Bladed,
	// and return the values of variables (y) required by Bladed from the DLL.
	//==============================================================================================

	if (iCallType == 6)
	{
		dArg7[0] = dArg5[0] + dV0*(*dArg2);
		dArg7[1] = dArg5[1] + dV0;
		dArg7[2] = dArg5[2] + dGearRatio*dArg7[0];
		dArg7[3] = dArg5[3] + dGearRatio*dArg7[1];
	}

	//==============================================================================================
	//==============================================================================================

	if (iCallType == 7 || iCallType == 8 || iCallType == 9)
	{
		double dToothTorque, dFricMoving, dFricStopped;


		dLssPos = dArg5[0] + dV0*(*dArg2);
		dLssVel = dArg5[1] + dV0;
		if (iStopped == 0)
		{
			dHssPos = dArg5[2] + dGearRatio*dLssPos;
			dHssVel = dArg5[3] + dGearRatio*dLssVel;
			dToothTorque = -(dStiffness*dArg5[2] + dDamping*dArg5[3])/dGearRatio;
		}
		else
		{
			dHssPos = dTheta2Stopped;
			dHssVel = 0.0;
			dToothTorque = dStiffness*(dLssPos - dTheta2Stopped/dGearRatio) + dDamping*dLssVel;
		}
		dFricMoving = dArg6[2] + dArg6[3];					//Friction torques applied
		dFricStopped = -dToothTorque/dGearRatio + dArg6[1];	//Reaction torque for zero HSS acceleration
		//Ensure the tolerances are not too small for this system
		dVelTol = MAX(dVelTol, ABS(dHssVel)*1.0E-5);
		dFricTol = MAX(dFricTol, ABS(dFricMoving)*1.0E-5);
		dFricTol = MAX(dFricTol, ABS(dArg6[1])*1.0E-5);

		//==============================================================================================
		// Call to pass to the DLL the state values and any other variables required from Bladed,
		// and return the values of output variables (yout) required for output.
		//==============================================================================================

		if (iCallType == 7)
		{
			dArg7[0] = dLssPos;
			dArg7[1] = dLssVel;
			dArg7[2] = dHssPos;
			dArg7[3] = dHssVel;
			dArg7[4] = dArg6[0];
			dArg7[5] = dArg6[1];
			dArg7[6] = dToothTorque;
			if (iStopped == 1) dArg7[7] = dFricStopped;
			else if (dHssVel > 0) dArg7[7] = -dFricMoving;		//Must oppose motion
			else if (dHssVel < 0) dArg7[7] = dFricMoving;		//Must oppose motion
			else dArg7[7] = 0.0;
		}

		//==============================================================================================
		// Call to check if a discontinuity has been passed
		//==============================================================================================

		if (iCallType == 8)
		{
			if ((iStopped == 0) && (ABS(dHssVel) < 0.5*dVelTol) && (ABS(dFricStopped) < dFricMoving))
			{
				iArg3[2] = -1;
			}
			else if ((iStopped == 1) && (ABS(dFricStopped) > dFricMoving + dFricTol))
			{
				iArg3[2] = -1;
			}
		}

		//==============================================================================================
		// Call to implement the discontinuity
		//==============================================================================================

		if (iCallType == 9)
		{
			if ((iStopped == 0) && (ABS(dHssVel) < dVelTol) && (ABS(dFricStopped) < dFricMoving))
			{
				iStopped = 1;
				dTheta2Stopped = dHssPos;
				//Don't use dV0 any more - it's no longer useful anyway in this case.
				dArg5[0] = dLssPos;
				dArg5[1] = dLssVel;
				dV0 = 0.0;
				//Disable states 3 & 4
				iArg3[4] = 0;
				iArg3[5] = 0;
			}
			else if ((iStopped == 1) && (ABS(dFricStopped) > dFricMoving))
			{
				iStopped = 0;
				//Ensure states 3 & 4 are correct
				dArg5[2] = dHssPos - dGearRatio*dLssPos;
				dArg5[3] = dHssVel - dGearRatio*dLssVel;
				//Enable states 3 & 4
				iArg3[4] = 1;
				iArg3[5] = 1;
			}
		}

	}

	PutStatics(dGearRatio, dStiffness, dDamping, dInertiaLSS, dInertiaHSS, dV0, dTheta2Stopped, dVelTol, dFricTol);
	return;
}

//==============================================================================================
//==============================================================================================

void EOM (double *dTime, double *dStates, double *dTorques, double dGearRatio, double dStiffness,
		  double dDamping, double dInertiaLSS, double dInertiaHSS, double *dDerivs, double dFricTrq)
{
	double I1, I2, G, K, D, Jinv;
	I1 = dInertiaLSS;
	I2 = dInertiaHSS;
	G = dGearRatio;
	K = dStiffness;
	D = dDamping;

// x1 = low speed shaft position, x2 = high speed shaft position
// v1 = x1', v2 = x2' ( ' means derivative w.r.t. time)
// Q1 = low speed chaft input torque, Q2 = high speed shaft output torque

// Accelerations ("):
// x1" = -(K/I1).x1		-(D/I1).x1'		+K/(G*I1).x2	+D/(G*I1).x2'	+Q1/I1
// x2" =  (K/G*I2).x1	+D/(G*I2).x1'	-K/(G*G*I2).x2	-D/(G*G*I2).x2'	-Q2/I2

// Now define: z2 = x2 - G*x1 (Differential position); z2' = x2' - G*x1'
// Using z2 will be numerically more stable than using x2.
// Also define:

	Jinv = 1.0/I1 + 1.0/(G*G*I2);

// Then:
// z2" = -(K*Jinv).z2 -(D*Jinv).z2' -(G/I1).Q1 -(1/I2).Q2

// State space:
// |x1'|   |   0   1    0        0    ||x1 |   |  0     0  |
// |x1"| = |   0   0 K/(G*I1) D/(G*I1)||x1'| + | 1/I1   0  ||Q1|
// |z2'|   |   0   0    0        1    ||z2 |   |  0     0  ||Q2|
// |z2"|   |   0   0 -K*Jinv  -D*Jinv ||z2'|   |-G/I1 -1/I2|

// Further improvement for numerical stability:
// Define z1 = x1 - v0.t where v0 is some nominal velocity, and state z1 is then the deviation from this value
// z1' = x1' - v0
// z1" = x1"
// This does not change the above state-space formulation except that z1, z1', z1" replace x1, x1', x1"

	dDerivs[0] = dStates[1];
	dDerivs[1] = K/(G*I1)*dStates[2] + D/(G*I1)*dStates[3]  + 1.0/I1*dTorques[0];
	dDerivs[2] = dStates[3];
	dDerivs[3] = -K*Jinv*dStates[2] - D*Jinv*dStates[3] - G/I1*dTorques[0] - 1.0/I2*(dTorques[1] - dFricTrq);
}

void EOM_Stopped (double *dTime, double *dStates, double *dTorques, double dGearRatio, double dStiffness,
		  double dDamping, double dInertiaLSS, double dInertiaHSS, double *dDerivs, double dTheta2)
{
	double I1, G, K, D;
	I1 = dInertiaLSS;
	G = dGearRatio;
	K = dStiffness;
	D = dDamping;
	//States are just the low speed shaft position and velocity:
	dDerivs[0] = dStates[1];
	dDerivs[1] = -K/I1*dStates[0] - D/I1*dStates[1] + 1.0/I1*dTorques[0] + K/(G*I1)*dTheta2;
	//Other derivatives not strictly needed as states will be disabled:
	dDerivs[2] = -G*dDerivs[0];
	dDerivs[3] = -G*dDerivs[1];
}

//==============================================================================================
//==============================================================================================

int ReadOneParameter(FILE *file, char *Name, double *value, char *cMsg)
{
	char cLine[64];
	strcpy(cLine,Name);
	strcat(cLine," %lf \n");
	if (fscanf(file, cLine, value) != 1)
	{
		strcpy(cMsg,"Failed to read parameter: ");
		strcat(cMsg,Name);
		return(-1);
	}
	else return(0);
}
