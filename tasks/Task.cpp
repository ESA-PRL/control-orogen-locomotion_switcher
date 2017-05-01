/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace locomotion_switcher;

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

Task::~Task()
{
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    window = 0.01;
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    state = INITIAL;
    new_lc_command = false;
    new_ww_command = false;
    std::cout<<"SWITCHER: waiting for new locomotion mode" <<std::endl;
    current_locomotion_mode = -1;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    if (_locomotion_mode.read(new_locomotion_mode) == RTT::NewData)
        if (new_locomotion_mode != current_locomotion_mode)
	    if (new_locomotion_mode)
		state = D2WW;
	    else
		state = WW2D;

    if (_joints_readings.read(joints_readings) == RTT::NewData)
    {
	
    }

    if (_ww_commands.read(ww_commands) == RTT::NewData)
    {
	new_ww_command = true;
    }

    if (_lc_commands.read(lc_commands) == RTT::NewData)
    {
	new_lc_command = true;
    }

    if (state == D2WW)
    {
	std::cout<<"SWITCHER: transitioning to wheel-walking mode" <<std::endl;
	if (isZeroSteering())
	    state = WHEEL_WALKING;
	else
	    _joints_commands.write(rectifySteering());
	    
    }

    if (state == WW2D)
    {
	std::cout<<"SWITCHER: transitioning to driving mode" <<std::endl;
	if(isZeroWalking())
	{
	    state = DRIVING;
	    std::cout<<"SWITCHER: entering driving mode" <<std::endl;
	}
	else
	    _joints_commands.write(rectifyWalking());
    }

    if ((state == DRIVING)&&(new_lc_command))
    {
	std::cout<<"SWITCHER: new driving command" <<std::endl;
	_joints_commands.write(lc_commands);
	new_lc_command = false;
    }


    if ((state == WHEEL_WALKING)&&(new_ww_command))
    {
	new_ww_command = false;
    }

}

bool Task::isZeroSteering()
{
    for (unsigned int i = 6; i < 10; i++)
	    if (fabs(joints_readings[i].position) > window)
		return false;
    return true;
}

bool Task::isZeroWalking()
{
    for (unsigned int i = 0; i < 6; i++)
	if (fabs(joints_readings[i].speed) > 0)
	    return false;
    for (unsigned int i = 10; i < 16; i++)
    {
	if (fabs(joints_readings[i].position) > 2*window)
	    return false;
	if (fabs(joints_readings[i].speed) > 0)
	    return false;
    }
    return true;
}

base::commands::Joints Task::rectifySteering()
{
    base::commands::Joints rJoints;
    rJoints.resize(16);
    for (unsigned int i = 0; i < 16; i++)
	if ((i > 5)&&(i < 10))
	    rJoints[i].position = 0.0;
	else
	    rJoints[i].speed = 0.0;
    return rJoints;
}

base::commands::Joints Task::rectifyWalking()
{
    base::commands::Joints rJoints;
    rJoints.resize(19);
    double gamma = 0.15;
    double L = 0.1253;
    double R = 0.07;
    for(uint i=0;i<6;i++){
	if(joints_readings[10+i].position > 2*window){
	    rJoints[10+i].speed = -gamma*joints_readings[10+i].position;
	    rJoints[i].speed = -gamma*(1+cos(joints_readings[10+i].position)*L/R)*joints_readings[10+i].position;
	}else if(joints_readings[10+i].position < -2*window){
	    rJoints[10+i].speed = -gamma*joints_readings[10+i].position;
	    rJoints[i].speed = -gamma*(1+cos(joints_readings[10+i].position)*L/R)*joints_readings[10+i].position;
	}else{
	    rJoints[10+i].speed = 0.0;
	    rJoints[i].speed = 0.0;
	}
    }
    for(uint i=16; i<19; i++)
    {
	rJoints[i].speed = base::NaN<double>();
	rJoints[i].position = base::NaN<double>();
    }
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
