#include "Task.hpp"

using namespace locomotion_switcher;

//__CONSTRUCTORS
Task::Task(std::string const& name)
    : TaskBase(name)
{
	state = INITIAL;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
	state = INITIAL;
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
    return true;
}


//__START
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    state = INITIAL;
    window = 0.01;
    first_iteration = true;
    std::cout<<"SWITCHER: INITIAL state" <<std::endl;
    motion_command.translation = 0.0;
    motion_command.rotation = 0.0;
    kill_switch = true;
    resetDepJoints = false;
    locomotionMode = 0;
    return true;
}


//__UPDATE
void Task::updateHook()
{
    TaskBase::updateHook();

    if ((_motion_command.read(motion_command) == RTT::NewData)||(_locomotionMode.read(locomotionMode)== RTT::NewData))
    {
        //std::cout<<"SWITCHER: new command received: " << motion_command.translation << " " << motion_command.rotation << std::endl;
        _lc_motion_command.write(motion_command);
        if ((motion_command.translation == 0) && (motion_command.rotation == 0))
        {
            if (state != INITIAL)
            {
                state = STOP;
                std::cout<<"SWITCHER: stopping rover " << std::endl;
            }
        }
        else if ((motion_command.translation == 0) && (motion_command.rotation != 0))
        {
            if ((state != LC) && (state != WWC2LC))
            {
                state = WWC2LC;
                std::cout<<"SWITCHER: to LC to do spot turn" << std::endl;
            }
        }
        else
        {
            if((locomotionMode == 0) && (state != LC) && (state != WWC2LC))
            {
                state = WWC2LC;
                std::cout<<"SWITCHER: changing to Locomotion Control" << std::endl;
            }
            if((locomotionMode == 1) && (state != WWC) && (state != LC2WWC))
            {
                state = LC2WWC;
                std::cout<<"SWITCHER: changing to Wheel-walking Control " << std::endl;
            }
        }
    }

    if (state == INITIAL)
    {
    }
    else
    {
        if(state == WWC)
        {
            if(resetDepJoints)
                _resetDepJoints.write(resetDepJoints = false);
            if(isZeroWalking())
            {
                //if(!resetDepJoints) _resetDepJoints.write(resetDepJoints = true);
                if(kill_switch) _kill_switch.write(kill_switch = false);
                _ww_joints_commands.read(ww_joints_commands);
                _joints_commands.write(ww_joints_commands);
            }
            else
            {
                _ww_joints_commands.read(ww_joints_commands);
                _joints_commands.write(ww_joints_commands);
            }
        }
        else
        {
            if(isZeroWalking())
            {
                if(resetDepJoints) _resetDepJoints.write(resetDepJoints = false);
                if(!kill_switch) _kill_switch.write(kill_switch = true);
                if(state == LC)
                {
                    if(isZeroSpeeds())
                        _lc_motion_command.write(motion_command);
                    if(_lc_joints_commands.read(lc_joints_commands) == RTT::NewData)
                        _joints_commands.write(lc_joints_commands);
                }
                else
                {
                    if(isZeroSteering())
                    {
                        if(state == STOP)
                        {
                            if(isZeroSpeeds())
                            {
                                state = INITIAL;
                            }
                            else
                            {
                                motion_command.translation = 0.0;
                                motion_command.rotation = 0.0;
                                _lc_motion_command.write(motion_command);
                                if(_lc_joints_commands.read(lc_joints_commands) == RTT::NewData)
                                    _joints_commands.write(lc_joints_commands);
                            }
                        }
                        else
                        {
                            if(state == WWC2LC)
                                state = LC;
                            if(state == LC2WWC)
                                state = WWC;
                        }
                    }
                    else
                        _joints_commands.write(rectifySteering());
                }
            }
            else
            {
                if(!resetDepJoints)
                    _resetDepJoints.write(resetDepJoints = true);
                if(!kill_switch)
                    _kill_switch.write(kill_switch = true);
                if(_ww_joints_commands.read(ww_joints_commands) == RTT::NewData)
                {
                    //std::cout<<"SWITCHER: RECTIFYING" << std::endl;
                    _joints_commands.write(ww_joints_commands);
                }
                //_joints_commands.write(rectifyWalking());
            }
        }
    }
}


//__CHECK_IF_STEERING_JOINT_POSITIONS_ARE_ZERO
bool Task::isZeroSteering()
{
    _motors_readings.read(motors_readings);
    for (unsigned int i = 0; i < 10; i++)
	if (fabs(motors_readings[i].speed) > window)
	    return false;
    for (unsigned int i = 10; i < 16; i++)
	    if ((fabs(motors_readings[i].position) > 2*window)||(fabs(motors_readings[i].speed) > window))
		return false;
    return true;
}


//__CHECK_IF_WALKING_JOINT_POSITIONS_ARE_ZERO
bool Task::isZeroWalking()
{
    _motors_readings.read(motors_readings);
    /*for (unsigned int i = 0; i < 10; i++)
	if (fabs(motors_readings[i].speed) > window)
	    return false;*/
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

    for (unsigned int i = 0; i < 6; i++)
        rJoints[i].speed = 0;
    for (unsigned int i = 0; i < 16; i++)
	if ((i > 5)&&(i < 10))
	    rJoints[i].position = 0;
	else
	    rJoints[i].speed = 0;
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
