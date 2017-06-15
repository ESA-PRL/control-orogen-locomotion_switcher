/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

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
    current_locomotion_mode = -1;
    joystick_motion_command.translation = 0.0;
    joystick_motion_command.rotation = 0.0;
    return true;
}


//__UPDATE

void Task::updateHook()
{
    TaskBase::updateHook();
    controldev::RawCommand joystick_command;
    for(int i = 0; i < 12; i++)
    {
        joystick_command.buttons.elements.push_back(0);
    }
    for(int i = 0; i < 8; i++)
    {
        joystick_command.axes.elements.push_back(0);
    }
    if (state == INITIAL)
        _ww_joystick_command.write(joystick_command);

    
    std::vector< int16_t > locomotionVector;
    base::samples::RigidBodyState pose;
    base::Waypoint currentWaypoint;
    int current_segment;

    if (_joystick_command.read(joystick_command) == RTT::NewData)
        evaluateJoystickCommands(joystick_command);

    if (_pose.read(pose) == RTT::NewData);

    if (_currentWaypoint.read(currentWaypoint) == RTT::NewData);

    if (_trajectory.read(trajectory) == RTT::NewData);

    if (_locomotionVector.read(locomotionVector) == RTT::NewData)
	if (state == INITIAL)
        {
            std::cout<<"SWITCHER: Locomotion Vector Received, going to Locomotion Control State" <<std::endl;
            state = LC;
        }
	    

    if (_current_segment.read(current_segment) == RTT::NewData)
    {
        //std::cout<<"SWITCHER: current segment: " << current_segment << " with mode " << locomotionVector[current_segment] <<std::endl;
        if (state != INITIAL)
        {
            if ((locomotionVector[current_segment] == 0) && (state == WWC))
                state = WWC2LC;
            if ((locomotionVector[current_segment] == 1) && (state == LC))
            {
                std::cout<<"SWITCHER: waypoint heading = " << trajectory[current_segment].heading <<std::endl;
                std::cout<<"SWITCHER: Yaw = " << pose.getYaw() <<std::endl;
                std::cout<<"SWITCHER: resultado = " << fabs(pose.getYaw() - currentWaypoint.heading) <<std::endl;
                state = ALIGNING;
            }
        }
    }

    if (_joystick_motion_command.read(joystick_motion_command) == RTT::NewData);

    if (state == LC)
	_lc_motion_command.write(joystick_motion_command);


    if (_joints_readings.read(joints_readings) == RTT::NewData)
	if (state == WWC)
        {
	    _ww_readings.write(joints_readings);
        }

    if (_motors_readings.read(motors_readings) == RTT::NewData)
	if ((state == LC)||(state == ALIGNING))
	    _lc_readings.write(motors_readings);

    if (_lc_commands.read(lc_commands) == RTT::NewData)
	if ((state == LC)||(state == ALIGNING))
	    _joints_commands.write(lc_commands);

    if (_ww_commands.read(ww_commands) == RTT::NewData)
	if (state == WWC)
        {
	    _joints_commands.write(ww_commands);
        }

    if (_bema_joints.read(bema_joints) == RTT::NewData)
	if (state == LC);
	    // FIX THIS TO INCLUDE BEMA JOINTS FUNCTIONALITY

  // Transition to Wheelwalking control

    if (state == ALIGNING)
    {
        if (fabs(pose.getYaw() - trajectory[current_segment].heading) < 0.01)
        {
            state = LC2WWC;
        }
        else
        {
            joystick_motion_command.translation = 0.0;
            if ((pose.getYaw() - trajectory[current_segment].heading) < 0.0)
            {
                joystick_motion_command.rotation = 0.01 + trajectory[current_segment].heading - pose.getYaw();
            }
            else
            {
                joystick_motion_command.rotation = -0.01 + trajectory[current_segment].heading - pose.getYaw();
            }
	    _lc_motion_command.write(joystick_motion_command);
        }
    }

    if (state == LC2WWC)
    {
        
	if (isZeroSteering())
	{
	    std::cout<<"SWITCHER: steering joints are rectified" <<std::endl;
            initWW(joystick_command);
	    state = WWC;
            std::cout<<"SWITCHER: WW in control" <<std::endl;
	}
	else
	    _joints_commands.write(rectifySteering());
    }

  // Transition to Locomotion Control
    if (state == WWC2LC)
    {
	if(isZeroWalking())
        {
	    state = LC;
            std::cout<<"SWITCHER: LC in control" <<std::endl;
        }
	else
	    _joints_commands.write(rectifyWalking());
    }    
}


//__CHECK_IF_STEERING_JOINT_POSITIONS_ARE_ZERO

bool Task::isZeroSteering()
{
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
    for (unsigned int i = 0; i < 10; i++)
	if (fabs(motors_readings[i].speed) > window)
	    return false;
    for (unsigned int i = 10; i < 16; i++)
	if ((fabs(motors_readings[i].position) > 2*window)||(fabs(motors_readings[i].speed) > window))
	    return false;
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
    //std::cout<<"SWITCHER: rectifying steering joints" <<std::endl;
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
	if(motors_readings[10+i].position > 2*window){
	    rJoints[10+i].speed = -gamma*motors_readings[10+i].position;
	    rJoints[i].speed = gamma*(1+cos(motors_readings[10+i].position)*L/R)*motors_readings[10+i].position;
	}else if(motors_readings[10+i].position < -2*window){
	    rJoints[10+i].speed = -gamma*motors_readings[10+i].position;
	    rJoints[i].speed = gamma*(1+cos(motors_readings[10+i].position)*L/R)*motors_readings[10+i].position;
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

//__JOYSTICK_COMMANDS_EVALUATION

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
    {
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
    }

    if (state == WWC)
	_ww_joystick_command.write(joystick_command);

    last_button_values = joystick_command.buttons.elements;
    last_axes_values = joystick_command.axes.elements;
}


//__REQUIRED_INITIALIZATION_IN_WHEELWALKING_CONTROL

void Task::initWW(const controldev::RawCommand joystick_command)
{
    controldev::RawCommand joystick_rectifier = joystick_command;
    //joystick_rectifier.buttons[6] = 1; //Engage Kill Switch
    //joystick_rectifier.buttons[1] = 1; //Side-By-Side
    //joystick_rectifier.buttons[5] = 1;
    //joystick_rectifier.buttons[7] = 1; //Disengage Kill Switch
    //joystick_rectifier.buttons[8] = 1; //Return to Initial Configuration
    //joystick_rectifier.axes[5] = 1;
    //_ww_joystick_command.write(joystick_rectifier);
    _kill_switch.write(false);
    std::cout<<"SWITCHER: deactivating kill switch on wheelwalking_control" <<std::endl;
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
