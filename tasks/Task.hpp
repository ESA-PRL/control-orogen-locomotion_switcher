/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef LOCOMOTION_SWITCHER_TASK_TASK_HPP
#define LOCOMOTION_SWITCHER_TASK_TASK_HPP

#include "locomotion_switcher/TaskBase.hpp"
#include <base/samples/Joints.hpp>
#include <base/commands/Joints.hpp>
#include <base/commands/Motion2D.hpp>
#include <vector>

namespace locomotion_switcher {

    enum SwitcherState {INITIAL, LC, WWC, ALIGNING, LC2WWC, WWC2LC, STOP};

    class Task : public TaskBase
    {
	friend class TaskBase;
        protected:
	    bool first_iteration;
	    double window;
	    SwitcherState state;
	    std::vector<int> last_button_values;
	    std::vector<double> last_axes_values;
	    base::commands::Joints ww_commands;
	    base::commands::Joints lc_commands;
	    base::commands::Joints joints_commands;
          // These are the readings from all active+passive joints
	    base::commands::Joints joints_readings;
          // These are the readings from the motors (active joints)
            base::commands::Joints motors_readings;
            base::commands::Motion2D joystick_motion_command;
	    base::samples::Joints bema_joints;
            std::vector<base::Waypoint> trajectory;
	    int current_locomotion_mode;
	    int new_locomotion_mode;
            int locomotionMode;
        public:
            Task(std::string const& name = "locomotion_switcher::Task");
            Task(std::string const& name, RTT::ExecutionEngine* engine);
	    ~Task();
            bool configureHook();
            bool startHook();
            void updateHook();

	    bool isZeroSteering();
	    bool isZeroWalking();
      bool isZeroSpeeds();
      bool kill_switch;
      bool resetDepJoints;

	    base::commands::Joints rectifySteering();
	    base::commands::Joints rectifyWalking();
            void initWW(const controldev::RawCommand joystick_command);

            void evaluateJoystickCommands(const controldev::RawCommand joystick_commands);

            void errorHook();
            void stopHook();
            void cleanupHook();
    };
}

#endif
