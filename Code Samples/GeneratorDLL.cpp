#include <stdio.h>
#include <string.h>
#define ABS(x) ((x) >= 0 ? (x) : (-x))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

void EOM (double *dTime, double *dStates, double *dDerivs, double dHSSTorque, double TimeConst);

int ReadOneParameter(FILE *file, char *Name, double *value, char *Msg);

static double cdTimeConst, cdEfficiency;

void GetStatics(double *dTimeConst, double *dEfficiency)
{
	*dTimeConst = cdTimeConst;
	*dEfficiency = cdEfficiency;
}

void PutStatics(double dTimeConst, double dEfficiency)
{
	cdTimeConst = dTimeConst;
	cdEfficiency = dEfficiency;
}

extern "C"
{
	void __declspec(dllexport) __cdecl
	DLL_GENER(int *iArg1, double *dArg2, int *iArg3, char *cArg4,
			 double *dArg5, double *dArg6, double *dArg7, char *cArg8);
}

void __declspec(dllexport) __cdecl
	DLL_GENER(int *iArg1, double *dArg2, int *iArg3, char *cArg4,
			 double *dArg5, double *dArg6, double *dArg7, char *cArg8)
{
	double dTimeConst, dEfficiency;
	static int Nx, Nout;
	int iCallType, ii;
	char *cInFile, *cVerifyFile;
	FILE *file;
	double dGenTorque, dGenSpeed, dGenPower;
	double rFreqDisturbanceFactor, rVoltDisturbanceFactor;

	//==============================================================================================
	// Call to initialise the DLL model (using any constants which might come from the Bladed model)
	// and return the number of integrator states and output variables.
	//==============================================================================================

	iCallType = iArg1[1];

	if (iCallType == 1)
	{
		Nx = 1;		//No of states
		Nout = 2;	//No of outputs
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
				if ((iArg1[8] = ReadOneParameter(file, "TIMECONSTANT", &cdTimeConst, cArg8)) < 0) return;
				if ((iArg1[8] = ReadOneParameter(file, "EFFICIENCY", &cdEfficiency, cArg8)) < 0) return;
			}
		}
		return;
	}

	//==============================================================================================
	// Call to return the names and absolute tolerances of integrator states required by the DLL.
	//==============================================================================================

	if (iCallType == 2)
	{
		iArg3[1] = 0;
		strcpy(cArg4,"Generator Torque state 1:N");

		//State absolute tolerances
		dArg5[0] = 1.0E-4;
		return;
	}

	GetStatics(&dTimeConst, &dEfficiency);

	//==============================================================================================
	// Call to return the names and units of variables from the DLL to be output in time history
	// output from Bladed (i.e. yout from the second state-space equation yout = C.x + D.u).
	//==============================================================================================

	if (iCallType == 3)
	{
		strcpy(cArg4,"Generator Torque:FL;Generator Power:P;");
	}

	//==============================================================================================
	// Call to execute one step of initial conditions iteration and return the initial values of
	// the states.
	//==============================================================================================

	if (iCallType == 4)
	{
		double dGenTorqueRef = dArg5[2];
		dGenSpeed = dArg5[0];
		rVoltDisturbanceFactor = dArg5[3];
		rFreqDisturbanceFactor = dArg5[4];

		dGenTorque = dGenTorqueRef;
		dGenPower = dGenTorque * dGenSpeed * dEfficiency;

		dArg6[0] = dGenTorque;
		dArg6[1] = dGenPower;

		if (iArg3[3] == 1)	//Initial values of states
		{
			dArg7[0] = dGenTorque;
		}
	}

	//==============================================================================================
	// Call to pass to the DLL the state values and any required variables derived from Bladed
	// states, and return the state derivatives (xdot) calculated by the DLL.
	//==============================================================================================

	if (iCallType == 5)
	{
		double dGenTorqueRef;
		dGenTorqueRef = dArg6[2];
		rVoltDisturbanceFactor = dArg6[3];
		rFreqDisturbanceFactor = dArg6[4];

		dGenTorqueRef = dGenTorqueRef * (1 + rVoltDisturbanceFactor);

		EOM(dArg2, dArg5, dArg7, dGenTorqueRef, dTimeConst);
	}

	//==============================================================================================
	// Call to pass to the DLL the state values and any other variables required from Bladed,
	// and return the values of variables (y) required by Bladed from the DLL.
	//==============================================================================================

	if (iCallType == 6)
	{
		dGenTorque = dArg5[0];	// State variable

		dGenSpeed = dArg6[0];	// Non state variable
		dGenPower =  dGenTorque * dGenSpeed * dEfficiency;

		dArg7[0] = dGenTorque;
		dArg7[1] = dGenPower;
	}

	//==============================================================================================
	//==============================================================================================

	if (iCallType == 7 || iCallType == 8 || iCallType == 9)
	{
		dGenTorque = dArg5[0];

		rVoltDisturbanceFactor = dArg6[3];
		rFreqDisturbanceFactor = dArg6[4];

		//==============================================================================================
		// Call to pass to the DLL the state values and any other variables required from Bladed,
		// and return the values of output variables (yout) required for output.
		//==============================================================================================

		if (iCallType == 7)
		{
			dGenSpeed = dArg6[0];	// Non state variable
			dGenPower =  dGenTorque * dGenSpeed * dEfficiency;

			dArg7[0] = dGenTorque;
			dArg7[1] = dGenPower;
		}

		//==============================================================================================
		// Call to check if a discontinuity has been passed
		//==============================================================================================

		if (iCallType == 8)
		{
			return;
		}

		//==============================================================================================
		// Call to implement the discontinuity
		//==============================================================================================

		if (iCallType == 9)
		{
			return;
		}
	}
	PutStatics(dTimeConst, dEfficiency);
	return;
}

//==============================================================================================
//==============================================================================================

void EOM (double *dTime, double *dStates, double *dDerivs, double dHSSTorque, double TimeConst)
{
	if (TimeConst == 0)
	{
		dDerivs[0] = 0;
		return;
	}
	dDerivs[0] = 1/TimeConst * (dHSSTorque - dStates[0]);
}

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
