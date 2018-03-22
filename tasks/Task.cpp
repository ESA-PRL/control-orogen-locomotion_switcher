#include "Task.hpp"

using namespace locomotion_switcher;

//__CONSTRUCTORS
Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}


//__DECONSTRUCTOR
Task::~Task()
{
}


//__CONFIGURE
bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    isModeOverrideEnabled = true;
    locomotionModeOverride = LocomotionMode::DRIVING;
    state = INITIAL;
    stopRover = true;
    window = 0.01;
    std::cout<<"SWITCHER: INITIAL state" <<std::endl;
    motion_command.translation = 0.0;
    motion_command.rotation = 0.0;
    kill_switch = true;
    resetDepJoints = false;
    locomotionMode = LocomotionMode::DRIVING;
    return true;
}


//__START
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}


//__UPDATE
void Task::updateHook()
{
    TaskBase::updateHook();

    bool new_command = false;
    if (_motion_command.read(motion_command) == RTT::NewData)
        new_command = true;

    if (
            (!isModeOverrideEnabled && (_locomotion_mode.read(locomotionMode) == RTT::NewData))|| // new mode while autonomous or
            (_locomotion_mode_override.read(locomotionModeOverride) == RTT::NewData)||            // new override input or
            (new_command)                               // new motion command
       )
    {

        // do we need to override the path planner's locomotion mode?
        if (locomotionModeOverride == LocomotionMode::DONT_CARE)
        {
            isModeOverrideEnabled = false;
        }
        else
        {
            locomotionMode = locomotionModeOverride;
            isModeOverrideEnabled = true;
        }

        // stop command
        if ((motion_command.translation == 0) && (motion_command.rotation == 0))
        {
            // we need to stop the wheel walking component using the kill switch port.
            // the locomotion component (driving and turning) handles (0,0) commands by itself.
            std::cout<<"SWITCHER: stopping rover " << std::endl;
            stopRover = true;
        }
        // rotation command
        else if ((motion_command.translation == 0) && (motion_command.rotation != 0))
        {
            state = LC;
            std::cout<<"SWITCHER: do spot turn" << std::endl;
            stopRover = false;
        }
        // translation (and rotation) command
        else
        {
            if((locomotionMode == LocomotionMode::DRIVING) && (state != LC))
            {
                state = LC;
                std::cout<<"SWITCHER: changing to Locomotion Control" << std::endl;
            }
            else if((locomotionMode == LocomotionMode::WHEEL_WALKING) && (state != WWC))
            {
                state = WWC;
                std::cout<<"SWITCHER: changing to Wheel-walking Control " << std::endl;
            }
            else if((locomotionMode == LocomotionMode::DEPLOYMENT) && (state != DC))
            {
                state = DC;
                std::cout<<"SWITCHER: changing to Deployment Control " << std::endl;
            }
            stopRover = false;
        }
    }

    if(state == WWC)
    {
        if(resetDepJoints)
            _reset_dep_joints.write(resetDepJoints = false);

        if (stopRover)
        {
            if(!kill_switch)
            {
                _kill_switch.write(kill_switch = true);
            }
            // writing the kill switch does not take effect immidiately
            _ww_joints_commands.read(ww_joints_commands);
            _joints_commands.write(ww_joints_commands);
        }
        else if(!isZeroSteering())
        {
            // make sure that the wheels are straight before using wheel walking
            _joints_commands.write(rectifySteering());
        }
        else
        {
            if(kill_switch)
            {
                _kill_switch.write(kill_switch = false);
            }
            _ww_joints_commands.read(ww_joints_commands);
            _joints_commands.write(ww_joints_commands);
        }
    }
    else if (state == LC)
    {
        if(!isZeroWalking())
        {
            if (!kill_switch)
                _kill_switch.write(kill_switch = true);
            if(!resetDepJoints)
                _reset_dep_joints.write(resetDepJoints = true);
            if(_ww_joints_commands.read(ww_joints_commands) == RTT::NewData)
                _joints_commands.write(ww_joints_commands);
        }
        else
        {
            if (new_command)
                _lc_motion_command.write(motion_command);
            if(_lc_joints_commands.read(lc_joints_commands) == RTT::NewData)
                _joints_commands.write(lc_joints_commands);
        }
    }
    else if (state == DC)
    {
        //if(!isZeroSteering())
        //{
        //    // make sure that the wheels are straight before using wheel walking
        //    _joints_commands.write(rectifySteering());
        //}
        if (new_command)
        {
            if (stopRover)
                _lc_motion_command.write(motion_command);
            else
                _bema_command.write(motion_command.translation);
        }
        if(_lc_joints_commands.read(lc_joints_commands) == RTT::NewData)
            _joints_commands.write(lc_joints_commands);
    }
}

//__CHECK_ IF_STEERING_JOINT_POSITIONS_ARE_ZERO
bool Task::isZeroSteering()
{
    _motors_readings.read(motors_readings);
    for (unsigned int i = 6; i < 10; i++)
        if ((fabs(motors_readings[i].position) > 2*window)||(fabs(motors_readings[i].speed) > window))
            return false;
    return true;
}


//__CHECK_IF_WALKING_JOINT_POSITIONS_ARE_ZERO
bool Task::isZeroWalking()
{
    _motors_readings.read(motors_readings);
    for (unsigned int i = 10; i < 16; i++)
        if ((fabs(motors_readings[i].position) > 2*window)||(fabs(motors_readings[i].speed) > window))
            return false;
    return true;
}

bool Task::isZeroSpeeds()
{
    _motors_readings.read(motors_readings);
    for (unsigned int i = 0; i < 16; i++)
        if (fabs(motors_readings[i].speed) > window)
            return false;
    return true;
}


base::commands::Joints Task::rectifySteering()
{
    base::commands::Joints rJoints;
    rJoints.resize(19);
    rJoints.names[0] = "WHEEL_DRIVE_FL";
    rJoints.names[1] = "WHEEL_DRIVE_FR";
    rJoints.names[2] = "WHEEL_DRIVE_CL";
    rJoints.names[3] = "WHEEL_DRIVE_CR";
    rJoints.names[4] = "WHEEL_DRIVE_BL";
    rJoints.names[5] = "WHEEL_DRIVE_BR";

    rJoints.names[6] = "WHEEL_STEER_FL";
    rJoints.names[7] = "WHEEL_STEER_FR";
    rJoints.names[8] = "WHEEL_STEER_BL";
    rJoints.names[9] = "WHEEL_STEER_BR";

    rJoints.names[10] = "WHEEL_WALK_FL";
    rJoints.names[11] = "WHEEL_WALK_FR";
    rJoints.names[12] = "WHEEL_WALK_CL";
    rJoints.names[13] = "WHEEL_WALK_CR";
    rJoints.names[14] = "WHEEL_WALK_BL";
    rJoints.names[15] = "WHEEL_WALK_BR";

    rJoints.names[16] = "WHEEL_DRIVE_GROUP";
    rJoints.names[17] = "WHEEL_STEER_GROUP";
    rJoints.names[18] = "WHEEL_WALK_GROUP";

    for (unsigned int i = 0; i < 16; i++)
    {
        if ((i > 5)&&(i < 10))
        {
            rJoints[i].position = 0;
            rJoints[i].speed = base::NaN<double>();
        }
        else
        {
            rJoints[i].speed = 0;
            rJoints[i].position = base::NaN<double>();
        }
    }
    for(uint i=16; i<19; i++)
    {
	rJoints[i].speed = base::NaN<double>();
	rJoints[i].position = base::NaN<double>();
    }
    //std::cout<<"SWITCHER: rectifying steering joints" <<std::endl;
    return rJoints;
}


void Task::errorHook()
{
    TaskBase::errorHook();
}


void Task::stopHook()
{
    TaskBase::stopHook();
}


void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
