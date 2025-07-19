wait until ship:unpacked.
clearscreen.

print "This is the right file".

// Open terminal
CORE:PART:GETMODULE("kOSProcessor"):DOEVENT("Open Terminal").
set terminal:height to 8.
set terminal:width to 70.

// Create the GUI

LOCAL my_gui IS GUI(300,150).
set isGuiVisible to true.
set my_gui:x to 1200.
set my_gui:y to 850.

// Flags for the shuttle modes
set isShuttleFlightActive to false.  // true when the shuttle is in liftoff or reentry mode.
set isShuttleLiftoffActive to false. // true when the shuttle is in liftoff mode.
set isReentryActive to false.        // true when the shuttle is in reentry mode.
set isAutolanding to false.
set isCircling to false.
set circleAngleIncrementPerSecond to 0.
set autolandLongitude to -74.662222.
set autolandLatitude to -0.0725.
set autolandAltitude to 0.
set runwayBearing to 270.
set alphaFinal to 2.
set alphaApproach to 10.
set previousTargetHeading to 0.

// Default values
set defaultReentryPitch to 45.
set defaultLiftoffPitch to 90.
set shuttlePitch to 0.

// Add widgets to the GUI

LOCAL airSpeedBox is my_gui:addHBox().

//////////////////////////////////////////////////////////////////   AIRSPEED

LOCAL label IS airSpeedBox:ADDLABEL("Airspeed").
SET label:STYLE:ALIGN TO "LEFT".


local speedDecreaseButton to airSpeedBox:addButton("Slower").
SET speedDecreaseButton:STYLE:ALIGN TO "CENTER".
SET speedDecreaseButton:STYLE:HSTRETCH TO False.
set speedDecreaseButton:style:width to 60.
set speedDecreaseButton:onClick to {
	increaseCruiseSpeed(-5).
	set speedTextField:text to ("" + floor(requestedCruiseSpeed)).
}.

LOCAL speedTextField TO airSpeedBox:ADDTEXTFIELD("").
SET speedTextField:STYLE:ALIGN TO "CENTER".
SET speedTextField:STYLE:HSTRETCH TO True.
set speedTextField:style:width to 60.

local speedIncreaseButton to airSpeedBox:addButton("Faster").
SET speedIncreaseButton:STYLE:ALIGN TO "CENTER".
SET speedIncreaseButton:STYLE:HSTRETCH TO False.
set speedIncreaseButton:style:width to 60.
set speedIncreaseButton:onClick to {
	increaseCruiseSpeed(5).
	set speedTextField:text to ("" + floor(requestedCruiseSpeed)).
}.

set speedTextField:onConfirm to {
	parameter str.
	set convertedVal to str:toNumber(-9999).
	if convertedVal = -9999 {
		print "error in speed conversion, setting speed to " + requestedCruiseSpeed.
		set convertedVal to requestedCruiseSpeed.
	}
	print "speed confirmation: " + convertedVal.
	set requestedCruiseSpeed to convertedVal.
	set speedTextField:text to ("" + floor(requestedCruiseSpeed)).
}.

//////////////////////////////////////////////////////////////////   ALTITUDE

LOCAL altitudeBox is my_gui:addHBox().



LOCAL label IS altitudeBox:ADDLABEL("Altitude").
SET label:STYLE:ALIGN TO "LEFT".



local altitudeDecreaseButton to altitudeBox:addButton("v").
SET altitudeDecreaseButton:STYLE:ALIGN TO "CENTER".
SET altitudeDecreaseButton:STYLE:HSTRETCH TO False.
set altitudeDecreaseButton:style:width to 60.
set altitudeDecreaseButton:onClick to {
	increaseRequestedAltitude(-5).
	set altitudeTextField:text to ("" + floor(requestedAlt)).
}.

LOCAL altitudeTextField TO altitudeBox:ADDTEXTFIELD("").
SET altitudeTextField:STYLE:ALIGN TO "CENTER".
SET altitudeTextField:STYLE:HSTRETCH TO True.
set altitudeTextField:style:width to 60.
set altitudeTextField:onConfirm to {
	parameter str.
	set convertedVal to str:toNumber(-9999).
	if convertedVal = -9999 {
		print "error in altitude conversion, setting altitude to " + requestedAlt.
		set convertedVal to requestedAlt.
	}
	print "altitude confirmation: " + convertedVal.
	set requestedAlt to convertedVal.
	set altitudeTextField:text to "" + requestedAlt.
}.

local altitudeIncreaseButton to altitudeBox:addButton("^").
SET altitudeIncreaseButton:STYLE:ALIGN TO "CENTER".
SET altitudeIncreaseButton:STYLE:HSTRETCH TO False.
set altitudeIncreaseButton:style:width to 60.
set altitudeIncreaseButton:onClick to {
	increaseRequestedAltitude(5).
	set altitudeTextField:text to ("" + floor(requestedAlt)).
}.

//////////////////////////////////////////////////////////////////   HEADING

LOCAL headingBox is my_gui:addHBox().

LOCAL label IS headingBox:ADDLABEL("Heading").
SET label:STYLE:ALIGN TO "LEFT".

local headingLeftButton to headingBox:addButton("Left").
SET headingLeftButton:STYLE:ALIGN TO "LEFT".
SET headingLeftButton:STYLE:HSTRETCH TO False.
set headingLeftButton:style:width to 60.
set headingLeftButton:onClick to {
	increaseHeading(-5).
}.

LOCAL headingTextField TO headingBox:ADDTEXTFIELD("").
SET headingTextField:STYLE:ALIGN TO "CENTER".
SET headingTextField:STYLE:HSTRETCH TO False.
set headingTextField:style:width to 60.

 set headingTextField:onConfirm to {
 	parameter str.
 	set convertedVal to str:toNumber(-9999).
 	if convertedVal = -9999 {
 		print "error in heading conversion, setting heading to " + requestedHeading.
 		set convertedVal to requestedHeading.
 	}
 	print "heading confirmation: " + convertedVal.
 	set requestedHeading to convertedVal.
 	set headingTextField:text to "" + requestedHeading.
 }.

local headingRightButton to headingBox:addButton("Right").
SET headingRightButton:STYLE:ALIGN TO "RIGHT".
SET headingRightButton:STYLE:HSTRETCH TO False.
set headingRightButton:style:width to 60.
set headingRightButton:onClick to {
	increaseHeading(5).
}.



//////////////////////////////////////////////////////////////////   SHUTTLE

//////////////////////////////////////////////////////////////////   SHUTTLE MAIN CONTROLS

LOCAL shuttleBox is my_gui:addHBox().

LOCAL shuttleCheckBox is shuttleBox:addCheckBox("Shuttle", false).

local shuttleLowerPitchButton to shuttleBox:addButton("-").
SET shuttleLowerPitchButton:STYLE:ALIGN TO "LEFT".
SET shuttleLowerPitchButton:STYLE:HSTRETCH TO False.
set shuttleLowerPitchButton:style:width to 60.
set shuttleLowerPitchButton:onClick to {
	set shuttlePitch to shuttlePitch - 5.
	set shuttlePitchTextField:text to ("" + shuttlePitch).
}.
shuttleLowerPitchButton:hide().

LOCAL shuttlePitchTextField TO shuttleBox:ADDTEXTFIELD("").
SET shuttlePitchTextField:STYLE:ALIGN TO "CENTER".
SET shuttlePitchTextField:STYLE:HSTRETCH TO False.
set shuttlePitchTextField:style:width to 60.
set shuttlePitchTextField:text to ("" + shuttlePitch).
set shuttlePitchTextField:onConfirm to {
	parameter str.
	print "shuttle text field changed: " + str.
	
 	set convertedVal to str:toNumber(-9999).
 	if convertedVal = -9999 {
 		print "error in heading conversion, setting shuttle pitch to " + shuttlePitch.
 		set convertedVal to shuttlePitch.
 	}
	print "setting shuttle pitch to " + convertedVal.
	set shuttlePitch to convertedVal.
}.
shuttlePitchTextField:hide().

local shuttleHigherPitchButton to shuttleBox:addButton("+").
SET shuttleHigherPitchButton:STYLE:ALIGN TO "RIGHT".
SET shuttleHigherPitchButton:STYLE:HSTRETCH TO False.
set shuttleHigherPitchButton:style:width to 60.
set shuttleHigherPitchButton:onClick to {
	set shuttlePitch to shuttlePitch + 5.
	set shuttlePitchTextField:text to ("" + shuttlePitch).
}.
shuttleHigherPitchButton:hide().

set shuttleCheckBox:onToggle to {
	parameter val.
	set isShuttleFlightActive to val.
	if isShuttleFlightActive {
		shuttleSecondRow:show().
		shuttleLiftoffCheckBox:show().
		shuttleReentryCheckBox:show().
		shuttleLowerPitchButton:show().
		shuttleHigherPitchButton:show().
		shuttlePitchTextField:show().
	}
	else {
		set isShuttleFlightActive to false.
		set isShuttleLiftoffActive to false.
		set isReentryActive to false.
		set shuttleLiftoffCheckBox:pressed to false.
		set shuttleReentryCheckBox:pressed to false.
		shuttleLiftoffCheckBox:hide().
		shuttleReentryCheckBox:hide().
		shuttleLowerPitchButton:hide().
		shuttleHigherPitchButton:hide().
		shuttlePitchTextField:hide().
		shuttleSecondRow:hide().
	}
}.


//////////////////////////////////////////////////////////////////   LIFTOFF + REENTRY
LOCAL shuttleSecondRow is my_gui:addHBox().
shuttleSecondRow:hide().


//////////////////////////////////////////////////////////////////   SHUTTLE LIFTOFF

LOCAL shuttleLiftoffCheckBox is shuttleSecondRow:addCheckBox("Shuttle Liftoff", false).

set shuttleLiftoffCheckBox:onToggle to {
	parameter val.
	set isShuttleLiftoffActive to val.
	if isShuttleLiftoffActive {
		print "Shuttle liftoff sequence.".
		freeSSMEsGimbal().
		set shuttlePitch to defaultLiftoffPitch.
		set shuttlePitchTextField:text to ("" + shuttlePitch).
		set shuttleReentryCheckBox:pressed to false.
	}
}.

shuttleLiftoffCheckBox:hide().


//////////////////////////////////////////////////////////////////   REENTRY

LOCAL shuttleReentryCheckBox is shuttleSecondRow:addCheckBox("Reentry", false).


set shuttleReentryCheckBox:onToggle to {
	parameter val.
	set isReentryActive to val.
	if isReentryActive {
		print "Shuttle reentry sequence.".
		set shuttlePitch to defaultReentryPitch.
		set shuttlePitchTextField:text to ("" + shuttlePitch).
		set shuttleLiftoffCheckBox:pressed to false.
		shutdownAndLockSSMEs().
	}
}.

shuttleReentryCheckBox:hide().


//////////////////////////////////////////////////////////////////   AUTOLAND AND CIRCLE

LOCAL autolandAndTurnBox is my_gui:addHBox().

LOCAL autolandCheckbox is autolandAndTurnBox:addCheckBox("Autoland").

set autolandCheckbox:onToggle to {
	parameter val.
	set isAutolanding to val.
	print "autolanding: " + val.
}.

LOCAL circleCheckBox is autolandAndTurnBox:addCheckBox("Circle").
LOCAL circleTextField is autolandAndTurnBox:addTextField().
circleTextField:hide().

set circleCheckBox:onToggle to {
	parameter val.
	set isCircling to val.
	if isCircling {
		circleTextField:show().
	}
	else {
		circleTextField:hide().
		set circleAngleIncrementPerSecond to 0.
	}
	print "circling: " + val.
}.


set circleTextField:onConfirm to {
	parameter str.
	set convertedVal to str:toNumber(-9999).
	if convertedVal = -9999 {
		print "error in circle conversion, setting angle to 0".
		set convertedVal to 0.
	}
	else {
		print "setting circle increment to " + convertedVal.
	}
	set circleAngleIncrementPerSecond to convertedVal.
}.

//////////////////////////////////////////////////////////////////   END AUTOLAND AND CIRCLE

//////////////////////////////////////////////////////////////////   END GUI

// Show the GUI.
my_gui:SHOW().


declare function trimDigits {
	parameter value.

	set dotIndex to (value+""):findLast(".").
	if dotIndex > 0 {
		set stringValue to (value+"").
		set trimIndex to min(dotIndex+3, stringValue:length).
		set valueTrimmed to (value+""):substring(0, trimIndex).
	}
	else {
		set valueTrimmed to value.
	}
	return valueTrimmed.
}

// Handle GUI widget interactions.
LOCAL isDone IS FALSE.


set standardMaxVario to 45.
set extendedMaxVario to 70.
set speedForExtendedVario to 200.
set currentMaxVario to 0.
set varioSmoothingRate to 2.


print "Starting.".

// Increase the loading distance.
set ship:LoadDistance:flying:load to 5000.
set ship:LoadDistance:flying:pack to 3000.
wait 0.001.
set ship:LoadDistance:landed:load to 5000.
set ship:LoadDistance:landed:pack to 3000.

//set KUNIVERSE:defaultLoadDistance:FLYING:load to 5000.
//set KUNIVERSE:defaultLoadDistance:LANDED:load to 5000.
//wait 0.001.
//set KUNIVERSE:defaultPackDistance:FLYING:load to 5000.
//set KUNIVERSE:defaultPackDistance:LANDED:load to 5000.


set altitudeIncrement to 10.
set speedIncrement to 10.
set terrainLatitude to -0.08726.
set terrainLongitude to -74.623. // west; east: -74.580
set terrainAltitude to 70.
set cruiseAltitudeMargin to 100.
set isDriving to false.
set isGoingHome to false.

set dHeading to 5.

set phase to "cruise".

sas off.
brakes off.

set targetPitch to 90.

set currentSpeed to 0.

set minPitch to -5.
set targetRoll to 270.

set remainingFuel to 0.

set dt to 0.3.

// this PID controls vertical speed, or vario
set kPvario to 0.5. set kIvario to 0.1. set kDvario to 0.5.

set requestedHeading to -floor(ship:bearing).
set requestedHeading to 5*(floor(requestedHeading/5)).

until (requestedHeading > 0) {
	set requestedHeading to requestedHeading + 360.
} 

set headingTextField:text to "" + requestedHeading.
set requestedCruiseSpeed to 5*(floor(ship:velocity:surface:mag/5)).
set speedTextField:text to "" + requestedCruiseSpeed.
set requestedAlt to 5*floor(ship:altitude/5).
set altitudeTextField:text to "" + requestedAlt.
set requestedVario to 0.

set isFollowingTarget to false.
set distanceToTarget to 0.

set mustPrintPalt to false.

set timeSinceLastDisplay to 0.
set timeBetweenDisplays to 3.

print "Target alt: " + requestedAlt + "; speed: " + requestedCruiseSpeed + "; heading: " + requestedHeading.


// PID to control vario
set Ivario to 0.
set Dvario to 0.
set PvarioPrev to requestedVario - ship:verticalspeed.
set prevVario to 0.
set tuneLimitVario to 100.
set targetPitchTrimmed to 0.
set maxPitch to 70.


// PID for horizontal speed
set kPspeed to 0.035.
set kIspeed to 0.0035.
set kDspeed to 0.12. // 0.07.
set Ispeed to 0.
set Dspeed to 0.
set prevSpeed to 0.
set tuneSpeedLimit to 1.3.


// PID for roll
set kProll to 0.000.
set kIroll to 0.000.
set kDroll to 0.000.
set Iroll to 0.
set Droll to 0.
set prevRoll to 0.


set prevHeading to ship:heading.
set maxLength to 200.

list engines in myEnginesList.
set remainingFlightDuration to 0.
set currentRange to 0.

set goingToNewVario to false.
set previousChangeInTargetVario to 0.

declare function computeSmoothedVario {
	if not isReentryActive {
		parameter currentAltitude.
		parameter requestedAltitude.
		parameter requestedVarioOld.

		set roughVario to computeVario(currentAltitude, requestedAltitude).
		
		set isVarioNew to false.
		
		if roughVario > requestedVarioOld + varioSmoothingRate {
			set varioSmoothed to requestedVarioOld + varioSmoothingRate.
		}
		else if roughVario < requestedVarioOld - varioSmoothingRate {
			set varioSmoothed to requestedVarioOld - varioSmoothingRate.
		}
		else {
			set varioSmoothed to roughVario.
			if roughVario <> requestedVarioOld and abs(roughVario) >= 1{
				print "final vario: " + varioSmoothed.
			}
		}
		
		if abs(varioSmoothed) > 1 and varioSmoothed <> roughVario {
			print "smoothed vario: " + varioSmoothed.
		}
		return varioSmoothed.
	}
	else {
		return 0.
	}
}

declare function computeVario {
	parameter currentAltitude.
	parameter requestedAltitude.
	
	set currentError to currentAltitude - requestedAltitude.

	set resultingVario to 0.
	
	// 	error:		-200	-100	- 50	- 10	+ 10	+ 50	+100	+200
	//	Vario:	+30     +15     +5       +2        0      -2     -5      -15      -30
	set varioA to 1.
	set varioB to 5.
	set varioC to 10.
	set varioD to 15.
	set varioE to 30.
//	set varioF to 70.
	
	
	if ship:velocity:surface:mag > speedForExtendedVario {
		set varioF to extendedMaxVario.
	}
	else {
		set varioF to standardMaxVario.
	}
	
	if currentError < -2000 {
		set resultingVario to varioF.
	}
	else if currentError < -1000 {
		set resultingVario to varioE.
	}
	else if -1000 < currentError and currentError < -400 {
		set resultingVario to varioD.
	}
	else if -400 < currentError and currentError < -200 {
		set resultingVario to varioC.
	}
	else if -200 < currentError and currentError < -50 {
		set resultingVario to varioB.
	}
	else if -50 < currentError and currentError < -10 {
		set resultingVario to varioA.
	}
	else if -10 < currentError and currentError < 10 {
		set resultingVario to -0.1 * currentError.
	}
	else if 10 < currentError and currentError < 50 {
		set resultingVario to -varioA.
	}
	else if 50 < currentError and currentError < 200 {
		set resultingVario to -varioB.
	}
	else if 200 < currentError and currentError < 400 {
		set resultingVario to -varioC.
	}
	else if 400 < currentError and currentError < 1000 {
		set resultingVario to -varioD.
	}
	else if 1000 < currentError and currentError < 2000 {
		set resultingVario to -varioE.
	}
	else{
		set resultingVario to -varioF.
	}
	
	// Requested vario will not excess limit.
	if resultingVario > varioF {
		set resultingVario to varioF.
		print "excess vario (l.198)".
	}
	return resultingVario.
}

declare function increaseHeading {
	parameter dHeading.

	set requestedHeading to requestedHeading + dHeading.
	if requestedHeading < 0 {
		set requestedHeading to requestedHeading + 360.
	}
	if requestedHeading >= 360 {
		set requestedHeading to requestedHeading - 360.
	}
	set requestedHeading to floor(requestedHeading/5) * 5.
	print "New heading: " + requestedHeading.
	set headingTextField:text to "" + requestedHeading.
}

declare function increaseCruiseSpeed {
	parameter dSpeed.

	set requestedCruiseSpeed to requestedCruiseSpeed + dSpeed.
}

declare function increaseRequestedAltitude {
	parameter dAlt.

	set requestedAlt to requestedAlt + dAlt.
	print "requestedAlt: " + requestedAlt.
}
			
declare function getHeadingForTarget {
	// TODO write the doc and set meaningful variable names.

	if hastarget {
		
		set lambdaA to ship:longitude.
		set phiA to ship:latitude.
		set lambdaB to target:longitude.
		set phiB to target:latitude.
		
		set TAX to V(-sin(lambdaA), cos(lambdaA), 0).
		set TAY to V(-sin(phiA)*cos(lambdaA), -sin(phiA)*sin(lambdaA), cos(phiA)).
		
		set Rt to 600000.
		
		set PNx to 0.
		set PNy to Rt*cos(phiA).
		
		set PBx to -Rt*cos(lambdaB)*cos(phiB)*sin(lambdaA) + Rt*sin(lambdaB)*cos(phiB)*cos(lambdaA).
		set PBy to -Rt*cos(lambdaB)*cos(phiB)*sin(phiA)*cos(lambdaA) - Rt*sin(lambdaB)*cos(phiB)*sin(phiA)*sin(lambdaA) + Rt*sin(phiB)*cos(phiA).
		
		
		set K to PBX / sqrt( PBx*PBx + PBy*PBy).

		if PBy > 0 {
			set newTargetHeading to arcsin(K).
		}
		else {
			set newTargetHeading to 180 - arcsin(K).
		}
		
		// Dampen the new target heading
		set dampener to 0.7. // 0: no damping, 1: full damping (and no taking new heading into account)
		set targetHeading to previousTargetHeading + (1-dampener)*(newTargetHeading - previousTargetHeading).
		
		// print "Target heading: " + targetHeading.
		
		set previousTargetHeading to targetHeading.
		return targetHeading.
		
	}
	else {
		print "No target. Ship current heading: " + (-ship:bearing).
		return -ship:bearing.
	}
}

declare function computeSpeedFromTarget {
//	TODO need to compute speed for landed, driving and flying targets.
	set resultSpeed to 0.
	if hastarget {
		set prevDistanceToTarget to distanceToTarget.
		set distanceToTarget to (target:position - ship:position):mag.

		set targetGroundSpeed to target:groundspeed.
		
		print "targetGroundSpeed : " + targetGroundSpeed.
		if targetGroundSpeed < 1 {
			set targetIsGrounded to true.
		}
		else {
			set targetIsGrounded to false.
		}
		
		if ship.status = "LANDED" or targetIsGrounded{
		
			print "follow landed target".
			set resultSpeed to requestedCruiseSpeed.
		}
		else {	// MOVING TARGET or FLYING SHIP
			
			print "follow flying target, d = " + trimDigits(distanceToTarget).
			
			if distanceToTarget < 100 {
				// We now are close to target
				if not targetIsGrounded {
					set resultSpeed to targetGroundSpeed.
				}

				if prevDistanceToTarget > distanceToTarget {
					brakes on.
				}
				else {
					brakes off.
				}
			}
			else if distanceToTarget < 300 {
				set resultSpeed to targetGroundSpeed + 5.
			}
			else {
				set resultSpeed to targetGroundSpeed + 10.
			}
			// print "	target ground speed: " + resultSpeed.
		}
		// print "requested speed: " + resultSpeed.
	}
	else {
		// Default speed requested by user.
		set resultSpeed to requestedCruiseSpeed.
	}
	
	return resultSpeed.
}

declare function next {
	parameter x.
	if x >= 10 {
		if(x >= 10000){
			set newX to x + 1000.
		}else{
			set newX to x*2.
		}
		print "log x: " + log10(x) + ", log(newX): " + log10(newX).
		set fx to floor(log10(x)).
		set fxNext to floor(log10(newX)).
		print "floors: " + fx + ", " + fxNext.
		if(fx <> fxNext) {
			set newX to 10^floor(log10(newX)).
		}
	}
	else {
		set newX to x+1.
	}
	return newX.
}

declare function previous {
	parameter x.
	if x = 0 {
		return x.
	}
	if x >= 10 {
		if(x >= 10000){
			set newX to x - 1000.
		}else{
			set newX to x/2.
		}
		print "log x: " + log10(x) + ", log(newX): " + log10(newX).
		set fx to floor(log10(x)).
		set fxNext to floor(log10(newX)).
		print "floors: " + fx + ", " + fxNext.
		if(fx <> fxNext) {
			set newX to 0.8*x.
		}
	}
	else {
		set newX to x-1.
	}
	return newX.
}

// Set the wheel steer to turn the aircraft toward 'requestedHeading'.
declare function computeSteerFromHeading {
	
	if isFollowingTarget and HASTARGET {
		// print "following target".
		set requiredBearing to target:bearing.
	}
	else {
		set requiredBearing to requestedHeading + ship:bearing.
		if requiredBearing > 180 {
			set requiredBearing to requiredBearing - 360.
		}
	}	

	//print "requested heading: " + requestedHeading.
	//print "currentHeading: " + (-ship:bearing).
	// print "requiredBearing: " + requiredBearing.
	return -requiredBearing / 100.
}


// Determine in which sector the plane currently is, relative to a given runway.
declare function getApproachSector {
	
	return "E".
}

// Free SSMEs gimbals.
declare function freeSSMEsGimbal {

	list engines in engineList.
	set nbEnginesFound to 0.
	for engine in engineList{
		if engine:name:contains("SSME"){
			// This engine is an SSME.
			// Freeing gimbal.
			set moduleGimbal to engine:GetModule("ModuleGimbal").
			set moduleGimbal:limit to 100.
		}
	}
}

// Shutdown SSMEs and lock gimbals.
declare function shutdownAndLockSSMEs {
	
	print "Shutting down SSMEs and locking gimbals.".
	list engines in engineList.
	for engine in engineList{
		if engine:name:contains("SSME"){
			// This engine is an SSME.
			set engineFound to true.
			
			// Shutting down engine.
			engine:shutdown.
						
			// Locking gimbal.
			set moduleGimbal to engine:GetModule("ModuleGimbal").
			set moduleGimbal:limit to 0.
		}
	}
}

set exit to false.
until exit = true{

	set totalFuelFlow to 0.
	for eng in myEnginesList {
		set totalFuelFlow to totalFuelFlow + eng:fuelFlow.
	}
	
	list resources in resourcesList.
	for res in resourcesList {
		if res:name = "LiquidFuel" {
			set remainingFuel to res:amount.
		}
	}
	if totalFuelFlow <> 0 {
		set remainingFlightDuration to remainingFuel / totalFuelFlow.
		set currentRange to remainingFlightDuration * ship:velocity:surface:mag.
	}
	
	set requestedVarioOld to requestedVario.
	if(isShuttleFlightActive){
		set requestedVario to 0. // Unused
	}
	else {
		set requestedVario to computeSmoothedVario(ship:altitude, requestedAlt, requestedVarioOld).
	}
	
	if(isReentryActive){
		// print "shuttle pitch for reentry: " + targetPitch.
		set targetPitch to shuttlePitch.
		set targetThrottle to 0.
	}
	else if (isShuttleLiftoffActive){
		// print "shuttle pitch for liftoff: " + targetPitch.
		set targetPitch to shuttlePitch.
		set targetThrottle to 1.
	}
	else{
		set Pvario to requestedVario - ship:verticalSpeed.
		set Ivario to Ivario + Pvario*dt.
		set Dvario to (Pvario - PvarioPrev)/dt.
		set targetPitch to kPvario*Pvario + kIvario*Ivario + kDvario*Dvario.
		set prevVario to ship:verticalSpeed.
		set PvSpeedPrev to requestedVario - ship:verticalSpeed.

		// print "pitch for normal flight: " + targetPitch.

		// PID for speed
		set PspeedPrev to requestedCruiseSpeed - prevSpeed.
		set Pspeed to requestedCruiseSpeed - ship:velocity:surface:mag.

		set Ispeed to Ispeed + Pspeed*dt.


		set Dspeed to (Pspeed - PspeedPrev)/dt.
		set prevSpeed to ship:velocity:surface:mag.
		if kIspeed * Ispeed > tuneSpeedLimit { set Ispeed to tuneSpeedLimit/kIspeed. } // Limit kI*i to [-1, 1]
		if kIspeed * Ispeed < -tuneSpeedLimit { set Ispeed to -tuneSpeedLimit/kIspeed. }

		set targetThrottle to kPspeed * Pspeed + kIspeed * Ispeed + kDspeed * Dspeed.
	}
	
	set targetPitchTrimmed to targetPitch.
	lock throttle to targetThrottle.


	// PID for roll
	set ProllPrev to targetRoll - prevRoll.
	set Proll to targetRoll - ship:facing:roll.
	set Iroll to Iroll + Proll*dt.
	set Droll to (Proll - ProllPrev)/dt.
	set prevRoll to ship:facing:roll.

	set rollCommand to kProll * Proll + kIroll * Iroll + kDroll * Droll.
	
	if isFollowingTarget {
		set requestedHeading to getHeadingForTarget().
		set requestedCruiseSpeed to computeSpeedFromTarget().
		set requestedAlt to target:altitude.
	}
	else if isAutolanding {
		// Compute heading to runway.
		print "Going to the runway " + autolandLatitude + ", " + autolandLongitude.
		set dLat to autolandLatitude - ship:latitude.
		set dLon to autolandLongitude - ship:longitude.
		//print "dLat: " + trimDigits(dLat) + ", dLon: " + trimDigits(dLon).
		
		//set alpha to arctan(dLat/dLon).
		set alpha to arctan2(dLat, dLon).
		print "alpha: " + alpha.
		
		set beta to alpha - runwayBearing + 90.
		//if beta < -180 {
		//	set beta to beta + 360.
		//}
		
		if abs(beta) <= alphaFinal {
			set requestedHeading to runwayBearing - 0.5*beta.
			print "FINAL APPROACH, requestedHeading is " + requestedHeading.
		}
		else if abs(beta) <= alphaApproach {
			set requestedHeading to runwayBearing - 2*beta.
			print "FIRST APPROACH, requestedHeading is " + requestedHeading.
		}
		else {
			print "autoland; beta: " + beta + ", alphaApproach: " + alphaApproach + ", alphaFinal: " + alphaFinal.
			// Do not change requestedHeading.
		}
	}
	else if isCircling {
		// Increase the requested heading every step.
		set requestedHeading to requestedHeading + circleAngleIncrementPerSecond*dt.
		if requestedHeading >= 360 {
			set requestedHeading to requestedHeading-360.
		}
		else if requestedHeading < 0 {
			set requestedHeading to requestedHeading + 360.
		}
		set headingTextField:text to "" + trimDigits(requestedHeading).
	}
	// Otherwise simply apply requested heading.
	

	// When changing directions on land, we must act on the wheel
	if ship:status = "LANDED" {
		// print "ship must turn on the ground.".
		if ship:groundspeed < 50 {
			// print "current yaw: " + ship:control:yaw.
			set commandSteer to computeSteerFromHeading().
			// print "	command steer: " + commandSteer.
			set ship:control:wheelsteer to commandSteer.
		}
		else {
			set ship:control:wheelsteer to 0.
		}
	}

	if isDriving {
		unlock steering.
	}
	else {
		// print "lock steering to pitch " + targetPitchTrimmed.
		set actualRequestedHeading to requestedHeading.
		if isShuttleLiftoffActive {
			// Shuttle flies inverted, i.e. pitch > 90
			// print "shuttle lifting off".
			set actualRequestedHeading to actualRequestedHeading + 180.
		}
		lock steering to heading(actualRequestedHeading, targetPitchTrimmed).
	}
	
	// Speed and Altitude control:
	// left, right: heading
	// Keyboard 'T': set heading to target
	// up, down: altitude
	// Keypad plus, keypad minus: speed

	// Read input from keyboard
	if terminal:input:haschar {
		set ch to terminal:input:getchar().
		if ch = terminal:input:LEFTCURSORONE {
			increaseHeading(-5).
		}
		if ch = terminal:input:RIGHTCURSORONE {
			set requestedHeading to requestedHeading + dHeading.
			if requestedHeading >= 360 {
				set requestedHeading to requestedHeading - 360.
			}
			set requestedHeading to floor(requestedHeading/5) * 5.
			print "New heading: " + requestedHeading.
			set headingTextField:text to "" + requestedHeading.
		}
		
		if ch = "9" {
			set speedIncrement to 2*speedIncrement.
			print "speed increment: " + speedIncrement.
		}
		if ch = "3" {
			set speedIncrement to speedIncrement/2.
			print "speed increment: " + speedIncrement.
		}
		if ch = "8" {
			set altitudeIncrement to 2*altitudeIncrement.
			print "altitude increment: " + altitudeIncrement.
		}
		if ch = "2" {
			set altitudeIncrement to altitudeIncrement/2.
			print "altitude increment: " + altitudeIncrement.
		}
		if ch = "0" {
			set isDriving to not isDriving.
			if isDriving {
				print "driving mode.".
			}
			else {
				print "flying mode.".
			}
		}
		
		set altiOrSpeedChanged to false.
		if ch = terminal:input:DOWNCURSORONE {
		
			set requestedAlt to requestedAlt - altitudeIncrement.
			set altiOrSpeedChanged to true.
			set altitudeTextField:text to ""+ requestedAlt.
		}
		if ch = terminal:input:UPCURSORONE {

			set requestedAlt to requestedAlt + altitudeIncrement.
			set altiOrSpeedChanged to true.
			set altitudeTextField:text to "" + requestedAlt.
		}
		
		if ch = "+" {
			set requestedCruiseSpeed to requestedCruiseSpeed + speedIncrement.
			set altiOrSpeedChanged to true.
			set speedTextField:text to "" + requestedCruiseSpeed.
		}
		if ch = "-" {
			set requestedCruiseSpeed to requestedCruiseSpeed - speedIncrement.
			set altiOrSpeedChanged to true.
			set speedTextField:text to "" + requestedCruiseSpeed.
		}
		
		
		if altiOrSpeedChanged {
			print "Target altitude : " + requestedAlt + "m; target speed: " + requestedCruiseSpeed + "m/s.".
		}
		
		
		if ch = "v" {
			print "Current vario: " + ship:verticalSpeed + ", target vario: " + requestedVario.
		}
		
		if ch = "t" {
			// Compute custom heading to go straight to target, following a great circle of the planet.
			if HASTARGET and not isFollowingTarget {
				// set requestedHeading to getHeadingForTarget().
				set isFollowingTarget to true.
				print "Follow target: " + target.
			}
			else {
				// Go back to a keypad-controlled heading
				set isFollowingTarget to false.
				set requestedHeading to -ship:bearing.
				set requestedHeading to floor(requestedHeading/5) * 5.
				print "No target. Requested heading : " + requestedHeading.
				// print "current heading : " + requestedHeading.
			}
		}
		
		if ch = "w" {
			// If a waypoint was set as target, follow this waypoint.
			
		}
		
		if ch = "d" {
			print "Estimated flight duration: " + trimDigits(remainingFlightDuration) + "s, expected currentRange: " + trimDigits(currentRange/1000) + "km".
		}
		if ch = "f" {
			set currentSpeed to ship:velocity:surface:mag. // speed in m/s
			set instantFuelConsumption to 0.
			list engines in engineList.
			for engine in engineList{
				set instantFuelConsumption to instantFuelConsumption + engine:fuelFlow.
			}
			if instantFuelConsumption > 0 {
				set fuelEfficiency to (currentSpeed/instantFuelConsumption).
			}
			else {
				set fuelEfficiency to 42000000000.
			}
			// Remove non-significant digits
			if(fuelEfficiency >= 1000) {
				set fuelEfficiency to fuelEfficiency/1000. // (km/L)
				//set dotIndex to (fuelEfficiency+""):findLast(".").
				//set fuelEfficiencyTrimmed to (fuelEfficiency+""):substring(0, dotIndex+3).
				print "Fuel efficiency: " + trimDigits(fuelEfficiency) + " km/L".
			}
			else{
				//set dotIndex to (fuelEfficiency+""):findLast(".").
				//set fuelEfficiencyTrimmed to (fuelEfficiency+""):substring(0, dotIndex+3).
				print "Fuel efficiency: " + trimDigits(fuelEfficiency) + " m/L".
			}
		}
		
		if ch = "h" {
			set isGoingHome to not isGoingHome.
			
			if isGoingHome {
				// Align on the runway facing west in preparation for landing.
				print "going home.".
			}
			else {
				print "Not going home yet.".
			}
		}
		
		if isGoingHome {
			// override altitude and heading commands
			set currentSector to getApproachSector().
			
			if currentSector = "A" {
				set latitudeDifference to ship:latitude - terrainLatitude.
				set targetHeading to 90 + latitudeDifference.
				set requestedAlt to terrainAltitude + 200.
			}
			else if currentSector = "B" {
				if ship:latitude > terrainLatitude {
					// B north
					set targetHeading to 135.
				}
				else {
					// B south
					set targetHeading to 45.
				}
				set requestedAlt to terrainAltitude + 400.
			}
			else if currentSector = "C" {
				set targetHeading to 270.
				set requestedAlt to terrainAltitude + 600.
			}
			else if currentSector = "D" {
				if ship:latitude > terrainLatitude {
					// D north
					set targetHeading to 0.
				}
				else {
					// D south
					set targetHeading to 180.
				}
				set requestedAlt to terrainAltitude + 600.
			}
			else if currentSector = "E" {
				// go for landing
			}
		}
		
		if ch = "l" {
			print "Current latitude: " + ship:latitude.
			print "Current longitude: " + ship:longitude.
		}
		
		if ch = "g" {
			// Toggle GUI
			if isGuiVisible {
				my_gui:HIDE().
			}
			else {
				my_gui:SHOW().
			}
			set isGuiVisible to not isGuiVisible.
		}
		
		if ch = "q" {
			print "Exit.".
			my_gui:HIDE().
			sas on.
			if isShuttleLiftoffActive {
				shutdownAndLockSSMEs().
			}
			set exit to true.
		}
	}
	
	
	// Wait
	set now to time:seconds.
	wait until time:seconds > now + dt.
	
	set timeSinceLastDisplay to timeSinceLastDisplay + dt.
	
	if timeSinceLastDisplay > timeBetweenDisplays {
		set timeSinceLastDisplay to 0.
		// display stuff
		// print "altitude: " + ship:altitude + ", target: " + requestedAlt + ", requested vario: " + requestedVario + ", currentError: " + currentError.
	}
}


set ship:control:wheelsteer to 0.