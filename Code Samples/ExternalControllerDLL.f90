! ====================================================================================================
! This is a very simple example of an external controller DLL written in FORTRAN which simply causes
! a step change in pitch position demand.
! If the pitch actuator cannot accept a pitch position demand, an error message is returned to Bladed.
! A simple example of logging output is also included.
! ====================================================================================================

  
INTEGER(4) FUNCTION CONTROLLER [C] (turbine_id)

    USE mExternalControllerApi    ! This defines the C function interfaces

    IMPLICIT NONE

    
    ! Compiler specific: Tell the complier that this routine is the entry point
    ! For the DLL. The next line is for the case of the Digital Visual Fortran
    ! compiler.
    !DEC$ ATTRIBUTES DLLEXPORT, ALIAS:'CONTROLLER' :: CONTROLLER

     ! Arguments
    INTEGER(C_SIZE_T), INTENT(IN) :: turbine_id[VALUE]

    ! Variables

    INTEGER(4) RESULT
    REAL*8 dMeasuredPitch, dMeasuredSpeed, dPitchDemand

    RESULT = 0;
    dPitchDemand = GetMeasuredPitchAngle(turbine_id, 0)
    dMeasuredSpeed = GetNominalHubFlowSpeed(turbine_id)
    dPitchDemand = 0.0

    ! Main calculation (User to supply calcs routine)

    !RESULT = calcs(iStatus, dMeasuredSpeed, dMeasuredPitch, dPitchDemand)

    ! ----------------------------------------------
    IF (RESULT.GT.0) THEN

        RESULT = SetDemandedPitchAngle(turbine_id, 0, dPitchDemand) 

        CONTROLLER = RESULT

        IF (RESULT.EQ.GH_DISCON_SUCCESS) THEN
            RESULT = ReportInfoMessage(turbine_id, "Set pitch"//CHAR(0))
            RETURN
        ENDIF
    ENDIF

    RESULT = ReportErrorMessage(turbine_id, "Failed to set pitch"//CHAR(0))
    RETURN
END
