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
	
    }

    if (_lc_commands.read(lc_commands) == RTT::NewData)
    {

    }

    if (state == INITIAL)
	std::cout<<"SWITCHER: waiting for new locomotion mode" <<std::endl;

    if (state == D2WW)
    {
	std::cout<<"SWITCHER: transitioning to wheel-walking mode" <<std::endl;
	if (isZeroSteering())
	    state = WHEEL_WALKING;
    }

    if (state == WW2D)
    {
	std::cout<<"SWITCHER: transitioning to driving mode" <<std::endl;
	if(isZeroWalking())
	    state = DRIVING;
    }

    if (state == DRIVING);


    if (state == WHEEL_WALKING);

}

bool Task::isZeroSteering()
{
    for (unsigned int i = 6; i < 10; i++)
	    if (joints_readings[i].position > window)
		return false;
    return true;
}

bool Task::isZeroWalking()
{
    for (unsigned int i = 11; i < 16; i++)
	    if (joints_readings[i].position > window)
		return false;
    return true;
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
