! This is a simple example of an external gearbox DLL written in FORTRAN.  It reads the user-defined parameter values 
! defined in Bladed from the text file created by Bladed.  It uses the gearbox ratio from Bladed, and
! includes stiffness and damping in the gearbox.  Any shaft brake and loss torques from Bladed are ignored.

SUBROUTINE DLL_GBX(iArg1,dArg2,iArg3,cArg4,dArg5,dArg6,dArg7,cArg8)

! Expose subroutine DLL_GBX to users of this DLL
!DEC$ ATTRIBUTES DLLEXPORT :: DLL_GBX
!DEC$ ATTRIBUTES ALIAS: "DLL_GBX" :: DLL_GBX	
IMPLICIT NONE

!Arguments
INTEGER		iArg1(9)	
REAL*8		dArg2	
INTEGER		iArg3(iArg1(3))	
INTEGER*1	cArg4(iArg1(4))	
REAL*8		dArg5(iArg1(5))	
REAL*8		dArg6(iArg1(6))	
REAL*8		dArg7(iArg1(7))
INTEGER*1	cArg8(iArg1(8))	

INTEGER II, iStart, iEnd, iFileHandle
LOGICAL lInUse, lGotLabel

REAL*8, SAVE :: dGearRatio, dStiffness, dDamping, dInertiaLSS, dInertiaHSS, dV0
CHARACTER(LEN=1024) :: cInFile

INTEGER iParameter
CHARACTER(LEN=12) :: cParamNames(4) = (/'STIFFNESS   ','DAMPING     ','INERTIALSS  ','INERTIAHSS  '/)
REAL*8 dParams(4)

!==============================================================================================
! Call to initialise the DLL model (using any constants which might come from the Bladed model) 
! and return the number of integrator states and output variables.
!==============================================================================================
	
IF (iArg1(2).EQ.1) THEN

	dGearRatio = dArg5(1)
	iArg3(1) = 4	!No of states
	iArg3(2) = 6	!No of outputs
	!Open parameter file
	cInFile = ' '
	DO II = 1,MIN(LEN(cInFile),iArg1(4))
		cInFile(II:II) = CHAR(cArg4(II))
		IF (cArg4(II).EQ.';') THEN
			cInFile(II:LEN(cInFile)) = ' '
			EXIT
		ENDIF
	ENDDO
	IF (LEN(cInFile).GT.0) THEN
		DO iFileHandle = 10,999
			INQUIRE(UNIT=iFileHandle, OPENED=lInUse)
			IF (.NOT.lInUse) EXIT
		ENDDO
		OPEN (iFileHandle, FILE=cInFile, STATUS='OLD', IOSTAT=iArg1(9))
		IF (iArg1(9).NE.0) THEN
			iArg1(9) = -1
			CALL CHAR2INT('Could not open DLL parameter file: '//TRIM(cInFile),cArg8)
		ELSE
			DO iParameter = 1,4
				READ (iFileHandle,'(A)') cInFile	!Re-use this string variable for the file data
				iStart = 0
				iEnd = 0
				lGotLabel = .FALSE.
				DO II = 1,LEN_TRIM(cInFile)
					IF (iStart.EQ.0.AND.cInFile(II:II).NE.' '.AND.cInFile(II:II).NE.CHAR(9)) THEN
						iStart = II
					ENDIF
					IF (iStart.GT.0.AND.cInFile(II:II).NE.' '.AND.cInFile(II:II).NE.CHAR(9)) iEnd = II
					IF (cInFile(iStart:iEnd).EQ.TRIM(cParamNames(iParameter))) THEN
						lGotLabel = .TRUE.
						iStart = 0
						iEnd = 0
					ENDIF
				ENDDO
				READ(cInFile(iStart:iEnd),*,IOSTAT=iArg1(9)) dParams(iParameter)
			ENDDO
			dStiffness = dParams(1)
			dDamping = dParams(2)
			dInertiaLSS = dParams(3)
			dInertiaHSS = dParams(4)
			IF (iArg1(9).NE.0) THEN
				iArg1(9) = -1
				CALL CHAR2INT('Failed to read from DLL parameter file',cArg8)
			ENDIF
		ENDIF
	ENDIF

  RETURN
ENDIF

!==============================================================================================
! Call to return the names and absolute tolerances of integrator states required by the DLL.
!==============================================================================================
	
IF (iArg1(2).EQ.2) THEN

	!Initialisation flags
	iArg3(2) = 0
	iArg3(3) = 1
	iArg3(4) = 0
	iArg3(5) = 1
	CALL CHAR2INT('Gearbox state 1:N;Gearbox state 2:N;Gearbox state 3:N;Gearbox state 4:N;',cArg4)
	dArg5(1:4) = 1.0E-4

  RETURN
ENDIF

!==============================================================================================
! Call to return the names and units of variables from the DLL to be output in time history 
! output from Bladed (i.e. yout from the second state-space equation yout = C.x + D.u).
!==============================================================================================

IF (iArg1(2).EQ.3) THEN

	CALL CHAR2INT('LSSpos:A;LSSvel:A/T;HSSpos:A;HSSvel:A/T;LSStrq:FL;HSStrq:FL;',cArg4)

	RETURN
ENDIF  

!==============================================================================================
! Call to execute one step of initial conditions iteration and return the initial values of 
! the states.
!==============================================================================================

IF (iArg1(2).EQ.4) THEN

	dArg6(1) = dGearRatio*(dArg5(1) - dGearRatio*dArg5(3)/dStiffness)
	dArg6(2) = dArg5(2)/dGearRatio
	dArg6(3) = dArg5(3)*dGearRatio 

	IF (iArg3(4).EQ.1) THEN	!Initial values of states
		dArg7(1) = dArg5(1)
		dV0 = dArg6(2)				!Use the initial velocity as the nominal velocity (see EOM below).
		dArg7(2) = dArg6(2) - dV0
		dArg7(3) = dArg6(1) - dGearRatio*dArg5(1)
		dArg7(4) = dArg5(2) - dGearRatio*dV0
	ENDIF
       
  RETURN
ENDIF

!==============================================================================================
! Call to pass to the DLL the state values and any required variables derived from Bladed 
! states, and return the state derivatives (xdot) calculated by the DLL.
!==============================================================================================
	
IF (iArg1(2).EQ.5) THEN

	CALL EOM (dArg2, dArg5, dArg6,dGearRatio, dStiffness, dDamping, dInertiaLSS, dInertiaHSS, dArg7)
	dArg7(5) = dArg7(2);	!LSS acceleration
	dArg7(6) = dArg7(4)+ dGearRatio * dArg7(2);	!HSS acceleration

  RETURN       
ENDIF

!==============================================================================================
! Call to pass to the DLL the state values and any other variables required from Bladed, 
! and return the values of variables (y) required by Bladed from the DLL.
!==============================================================================================
	
IF (iArg1(2).EQ.6) THEN

	dArg7(1) = dArg5(1) + dV0*dArg2
	dArg7(2) = dArg5(2) + dV0
	dArg7(3) = dArg5(3) + dGearRatio*dArg7(1);
	dArg7(4) = dArg5(4) + dGearRatio*dArg7(2);

  RETURN
ENDIF

!==============================================================================================
! Call to pass to the DLL the state values and any other variables required from Bladed, 
! and return the values of output variables (yout) required for output.
!==============================================================================================
	
IF (iArg1(2).EQ.7) THEN

	dArg7(1) = dArg5(1) + dV0*dArg2
	dArg7(2) = dArg5(2) + dV0
	dArg7(3) = dArg5(3) + dGearRatio*dArg7(1);
	dArg7(4) = dArg5(4) + dGearRatio*dArg7(2);
	DO II = 1,2
		dArg7(4+II) = dArg6(II)
	ENDDO
 
  RETURN
ENDIF
               
END SUBROUTINE DLL_GBX

!==============================================================================================
!==============================================================================================

SUBROUTINE EOM (dTime, dStates, dTorques, dGearRatio, dStiffness, dDamping, dInertiaLSS, dInertiaHSS, dDerivs)
IMPLICIT NONE
REAL*8 dTime
REAL*8 dStates(4), dDerivs(4), dTorques(2)
REAL*8 dGearRatio, dStiffness, dDamping, dInertiaLSS, dInertiaHSS

REAL*8 :: I1=6000, I2=60, G=90, K=1.6e8, D=0.25e6, Jinv
I1 = dInertiaLSS
I2 = dInertiaHSS
G = dGearRatio
K = dStiffness
D = dDamping

! x1 = low speed shaft position, x2 = high speed shaft position
! v1 = x1', v2 = x2' ( ' means derivative w.r.t. time)
! Q1 = low speed chaft input torque, Q2 = high speed shaft output torque

! Accelerations ("):
! x1" = -(K/I1).x1		-(D/I1).x1'		-K/(G*I1).x2	-D/(G*I1).x2'	+Q1/I1
! x2" =  (K/G*I2).x1	+D/(G*I2).x1'	-K/(G*G*I2).x2	-D/(G*G*I2).x2'	-Q2/I2	

! Now define: z2 = x2 - G*x1 (Differential position); z2' = x2' - G*x1'
! Using z2 will be numerically more stable than using x2.
! Also define:

Jinv = 1.0/I1 + 1.0/(G*G*I2);

! Then:
! z2" = -(K*Jinv).z2 -(D*Jinv).z2' -(G/I1).Q1 -(1/I2).Q2

! State space:
! |x1'|   |   0   1    0        0    ||x1 |   |  0     0  |
! |x1"| = |   0   0 K/(G*I1) D/(G*I1)||x1'| + | 1/I1   0  ||Q1|
! |z2'|   |   0   0    0        1    ||z2 |   |  0     0  ||Q2|
! |z2"|   |   0   0 -K*Jinv  -D*Jinv ||z2'|   |-G/I1 -1/I2|

! Further improvement for numerical stability:
! Define z1 = x1 - v0.t where v0 is some nominal velocity, and state z1 is then the deviation from this value
! z1' = x1' - v0
! z1" = x1"
! This does not change the above state-space formulation except that z1, z1', z1" replace x1, x1', x1"

dDerivs(1) = dStates(2)
dDerivs(2) = K/(G*I1)*dStates(3) + D/(G*I1)*dStates(4) + 1.0/I1*dTorques(1)
dDerivs(3) = dStates(4)
dDerivs(4) = -K*Jinv*dStates(3) - D*Jinv*dStates(4) - G/I1*dTorques(1) - 1.0/I2*dTorques(2)

RETURN
END

!==============================================================================================
!==============================================================================================

SUBROUTINE CHAR2INT(accString,avi1Int)
IMPLICIT NONE
CHARACTER*(*) accString
INTEGER*1 avi1Int(*)
INTEGER I
DO I = 1,LEN_TRIM(accString)
	avi1Int(I) = ICHAR(accString(I:I))
ENDDO
avi1Int(LEN_TRIM(accString)+1) = 0
RETURN
END
