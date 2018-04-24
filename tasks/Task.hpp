#ifndef LOCOMOTION_SWITCHER_TASK_TASK_HPP
#define LOCOMOTION_SWITCHER_TASK_TASK_HPP

#include "locomotion_switcher/TaskBase.hpp"
#include <base/samples/Joints.hpp>
#include <base/commands/Joints.hpp>
#include <base/commands/Motion2D.hpp>
#include <vector>
#include <base-logging/Logging.hpp>

#define BASE_LOG_NAMESPACE locomotion_switcher
#define BASE_LOG_DISABLE

namespace locomotion_switcher {

    enum SwitcherState {INITIAL, LC, WWC, DC};

    class Task : public TaskBase
    {
        friend class TaskBase;
        protected:
            double window;
            SwitcherState state;
            base::commands::Joints ww_joints_commands;
            base::commands::Joints lc_joints_commands;
            base::commands::Joints motors_readings;
            base::commands::Motion2D motion_command;

            LocomotionMode locomotionMode;

            bool stopRover;
            bool kill_switch;
            bool resetDepJoints;

            bool isModeOverrideEnabled;
            LocomotionMode locomotionModeOverride;
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

            base::commands::Joints rectifySteering();

            void errorHook();
            void stopHook();
            void cleanupHook();
    };
}
#endif
