/* =================================================================================================
This is a very simple example of an external controller DLL written in C++ which simply causes a
step change in pitch position demand.
If the pitch actuator cannot accept a pitch position demand, an error message is returned to Bladed.
A simple example of logging output is also included.
================================================================================================= */

#pragma comment(lib, "ExternalControllerApi.lib")

#include "ExternalControllerApi.h"    // This defines the C functions that can be called, as well as 'turbine'. 

using namespace GHTurbineInterface;


/// <summary>
/// Function to calculate the required pitch demand from the current pitch and wind speed.
/// </summary>
/// <TODO>Write this function.</TODO>
int calcs(const double measured_speed, const double measured_pitch, double pitch_demand)
{ return 0; };

 
extern "C" 
{
  int __declspec( dllexport ) __cdecl CONTROLLER (const turbine turbine_id)
  {    
    int result = 0;
    double measured_pitch, measured_speed, pitch_demand;
    
	measured_pitch = GetMeasuredPitchAngle(turbine_id, 0);
    measured_speed = GetNominalHubFlowSpeed(turbine_id);
    pitch_demand = 0.0;

    // Main calculation (User to supply calcs routine)
    result = calcs(measured_speed, measured_pitch, pitch_demand);
    // ----------------------------------------------

    if (result >= 0)
    {
        result = SetDemandedPitchAngle(turbine_id, 0, pitch_demand);
        if (result == GH_DISCON_SUCCESS) 
        {
            ReportInfoMessage(turbine_id, "Set pitch");
            return GH_DISCON_SUCCESS;
        }
    }

    ReportErrorMessage(turbine_id, "Failed to set pitch");

    return GH_DISCON_ERROR;
  }
}
