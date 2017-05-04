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


//__CONFIGURE__\\

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    window = 0.01;
    first_iteration = true;
    return true;
}


//__START__\\

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    state = INITIAL;
    new_lc_command = false;
    new_ww_command = false;
    std::cout<<"SWITCHER: INITIAL state" <<std::endl;
    current_locomotion_mode = -1;
    return true;
}


//__UPDATE__\\

void Task::updateHook()
{
    TaskBase::updateHook();
    controldev::RawCommand joystick_command;
    base::commands::Motion2D joystick_motion_command;

    if (_joystick_command.read(joystick_command) == RTT::NewData)
        evaluateJoystickCommands(joystick_command);

    if (_joystick_motion_command.read(joystick_motion_command) == RTT::NewData)
        if (state == LC)
	    _lc_motion_command.write(joystick_motion_command);

    if (_joints_readings.read(joints_readings) == RTT::NewData)
	if (state == LC)
	    _lc_readings.write(joints_readings);
	else if (state == WWC)
	    _ww_readings.write(joints_readings);

    if (_lc_commands.read(lc_commands) == RTT::NewData)
	if (state == LC)
	    _joints_commands.write(lc_commands);

    if (_ww_commands.read(ww_commands) == RTT::NewData)
	if (state == WWC)
	    _joints_commands.write(ww_commands);

    if (_bema_joints.read(bema_joints) == RTT::NewData)
	if (state == LC);
	    // FIX THIS

    if (state == LC2WWC)
	if (isZeroSteering())
	    state = WWC;
	else
	    _joints_commands.write(rectifySteering());

    if (state == WWC2LC)
	if(isZeroWalking())
	    state = LC;
	else
	    _joints_commands.write(rectifyWalking());    
}

bool Task::isZeroSteering()
{
    /*for (unsigned int i = 0; i < 6; i++)
	if (fabs(joints_readings[i].speed) > 0)
	    return false;*/
    for (unsigned int i = 6; i < 10; i++)
	    if (fabs(joints_readings[i].position) > window)
		return false;
    std::cout<<"SWITCHER: WW in control" <<std::endl;
    return true;
}

bool Task::isZeroWalking()
{
    /*for (unsigned int i = 0; i < 6; i++)
	if (fabs(joints_readings[i].speed) > 0)
	    return false;*/
    for (unsigned int i = 10; i < 16; i++)
    {
	if (fabs(joints_readings[i].position) > 2*window)
	    return false;
	/*if (fabs(joints_readings[i].speed) > 0)
	    return false;*/
    }
    std::cout<<"SWITCHER: LC in control" <<std::endl;
    return true;
}

base::commands::Joints Task::rectifySteering()
{
    base::commands::Joints rJoints;
    rJoints.resize(19);
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
    std::cout<<"SWITCHER: rectifying steering joints" <<std::endl;
    return rJoints;
}

base::commands::Joints Task::rectifyWalking()
{
    /*base::commands::Joints rJoints;
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
    return rJoints;*/
    
}


//__JOYSTICK_COMMANDS_EVALUATION__\\

void Task::evaluateJoystickCommands(const controldev::RawCommand joystick_command)
{
    if (first_iteration)
    {
        last_button_values = joystick_command.buttons.elements;
        last_axes_values = joystick_command.axes.elements;
        first_iteration = false;
    }

    if ((joystick_command.buttons[5] == 1 && last_button_values[5] == 0) && 
        (joystick_command.buttons[4] == 1 && last_button_values[4] == 0)) //BTN_Z (right top) + BTN_Y (left top)
        if (state == LC)
	{
	    std::cout<<"SWITCHER: switching to WWC" <<std::endl;
	    state = LC2WWC;
	}
	else if (state == WWC)
	{
	    std::cout<<"SWITCHER: switching to LC" <<std::endl;
	    state = WWC2LC;
	}
        else if (state == INITIAL)
        {
	    std::cout<<"SWITCHER: switching to LC" <<std::endl;
	    state = WWC2LC;
        }

    if (state == WWC)
	_ww_joystick_command.write(joystick_command);

    last_button_values = joystick_command.buttons.elements;
    last_axes_values = joystick_command.axes.elements;
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
