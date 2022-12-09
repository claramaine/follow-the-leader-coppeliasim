function sysCall_init()
    camera=sim.getObjectHandle('blobDetectionCamera_camera')
    baseObject=sim.getObjectAssociatedWithScript(sim.handle_self)
    res=sim.getVisionSensorResolution(camera)
    lastImageAnalysisTime=0
    lastLowestBlobDetectionPositionY=100
    lastLowestBlobDetectionTime=0
    
    blobSize=0
    blobCoords={0,0}
    firstPass = true
    
    leftJoint=sim.getObjectHandle("dr20_leftWheelJoint_")
    rightJoint=sim.getObjectHandle("dr20_rightWheelJoint_")

    PID_P=15 --because the optimal size for the blob is so small, we need to pump the error up so the velocity will be changed a significant amount
    PID_D=0 --this is changed over time
    
    PID_P_X=0.2 --The error for the x position is multiplied by 0.2 and applied to the wheels 
    PID_D_X=0
    
    PID_I=0.05 --it's useful to keep track of a constant error and and add that to the total error if it's a thing for too long
    pidCumulativeErrorForIntegralParam=0

    velocityLeft=0
    velocityRight=0
    
    targetBlobSize=0.01
    
end

function sysCall_cleanup() 
    first = false
end 

function sysCall_actuation()
--VISION AND BLOB DETECTION
    t=sim.getSimulationTime()
    imageAnalysisFrequency=sim.getScriptSimulationParameter(sim.getScriptAssociatedWithObject(baseObject),'imageAnalysisFrequency')

    if (t-lastImageAnalysisTime>1/imageAnalysisFrequency) then
        lastImageAnalysisTime=t
        sim.handleVisionSensor(camera) -- the image processing camera is handled explicitely, since we do not need to execute that command at each simulation pass
        result,t0,t1=sim.readVisionSensor(camera) -- Here we read the image processing camera!

        if (t1) then -- in t1 we have the blob information
            if t1[3] then
                blobSize = t1[3]
                blobCoords[0] = t1[5] --x
                blobCoords[1] = t1[6] --y
                PID_controller()
            else --NO BLOB FOUND
            
                sim.addStatusbarMessage("blob lost")
                sim.setJointTargetVelocity(leftJoint,0)
                sim.setJointTargetVelocity(rightJoint,0)
                
                --[[--just avoid objects
                sim.addStatusbarMessage("it's free real estate :(")
                objDetected = false

                velocityLeft=0
                velocityRight=0
                if not objDetected then --IF there is no object in front of us, proceed
                    velocityLeft=velUpperLimit*(1+2*velocityLeft)
                    velocityRight=velUpperLimit*(1+2*velocityRight)
                end --otherwise. Halt all velocity
                ]]--
                            
            end
        end
    end   
end 

function PID_controller() 
    --STUFF FOR VELOCITY
    s=sim.getObjectSizeFactor(baseObject) -- make sure that if we scale the robot during simulation, other values are scaled too!
    velUpperLimit=0.4*s
    wheelDiameter=0.085*s
    
    local currentBlobSize = blobSize

    local errorValue = targetBlobSize - currentBlobSize
    sim.addStatusbarMessage("\ncurrentBlobSize: "..tostring(currentBlobSize))
    sim.addStatusbarMessage("blobCoords: ("..tostring(blobCoords[0]).." , "..tostring(blobCoords[1])..")")
    sim.addStatusbarMessage("SizeErrorValue: "..tostring(errorValue))
    
    -- ACCOUNT FOR DISTANCE/SIZE OF BLOB
    -- 1. Proportional part:
    local ctrl=errorValue*PID_P
    
        --2. Integral
        if PID_I~=0 then
            pidCumulativeErrorForIntegralParam=pidCumulativeErrorForIntegralParam+errorValue
        else
            pidCumulativeErrorForIntegralParam=0
        end
        ctrl=ctrl+pidCumulativeErrorForIntegralParam*PID_I
        
        -- 3. Derivative part:
        if not firstPass then
            ctrl=ctrl+(errorValue-pidLastErrorForDerivativeParam)*PID_D
        end
        pidLastErrorForDerivativeParam=errorValue
        
        local ctrlVelocity = ctrl
        
        -- 4. Check that velocity is not exceeding the upper limit   
        if (ctrlVelocity>velUpperLimit) then
            ctrlVelocity=velUpperLimit
        end
        if (ctrlVelocity<-velUpperLimit) then
            ctrlVelocity=-velUpperLimit
        end
    
    --ACCOUNT FOR HORIZONTAL MOVEMENT
    xError = 0.5 - blobCoords[0] --target(center view) - current 
    
        -- 1. Proportional part:
        local xCtrl=xError*PID_P_X
        
        -- 3. Derivative part:
        if not firstPass then
            xCtrl=xCtrl+(xError-pidLastErrorForDerivativeParam)*PID_D_X
        end
        pidLastErrorForDerivativeParam=xError
        
        local wheelVelOffset = xCtrl
        sim.addStatusbarMessage("xErrorValue: "..tostring(wheelVelOffset))
    
    --VELOCITY
    velocityLeft=ctrlVelocity*(1+2*velocityLeft) - wheelVelOffset
    velocityRight=ctrlVelocity*(1+2*velocityRight) + wheelVelOffset
    sim.addStatusbarMessage("velocityL: "..tostring(velocityLeft))
    sim.addStatusbarMessage("velocityR: "..tostring(velocityRight))
    
    firstPass=false

    --Apply the updated velocities
    p=sim.boolOr32(sim.getModelProperty(baseObject),sim.modelproperty_not_dynamic)-sim.modelproperty_not_dynamic
    sim.setModelProperty(baseObject,p)
    sim.setJointTargetVelocity(leftJoint,velocityLeft*2/wheelDiameter)
    sim.setJointTargetVelocity(rightJoint,velocityRight*2/wheelDiameter)   
end
