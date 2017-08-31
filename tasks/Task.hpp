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
	    base::commands::Joints ww_joints_commands;
	    base::commands::Joints lc_joints_commands;
            base::commands::Joints motors_readings;
            base::commands::Motion2D motion_command;

            std::vector<base::Waypoint> trajectory;
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

            void errorHook();
            void stopHook();
            void cleanupHook();
    };
}

#endif
