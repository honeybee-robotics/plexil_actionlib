#ifndef ACTIONLIB_HANDLER_HH
#define ACTIONLIB_HANDLER_HH

// PLEXIL API
#include "AdapterConfiguration.hh"
#include "commandHandlerDefs.hh"
#include "InterfaceAdapter.hh"
#include "Value.hh"

#include <functional>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/action_definition.h>

namespace PLEXIL {
    class Command;
    class AdapterExecInterface;
}

namespace plexil_actionlib {
    template<typename ActionT>
    class ActionlibHandler : public PLEXIL::CommandHandler {
        /** Defines:
          Goal
          Result
          Feedback

          GoalConstPtr
          ResultConstPtr
          FeedbackConstPtr

          ActionGoal
          ActionResult
          ActionFeedback

          ActionGoalPtr
          ActionGoalConstPtr
          ActionResultConstPtr
          ActionFeedbackConstPtr
         **/
        ACTION_DEFINITION(ActionT);

        public:
            // Function signature for goal factory
            typedef std::function<Goal(PLEXIL::Command*, PLEXIL::AdapterExecInterface*)>
                GoalFactoryFunT;

            // Function signature for result handler
            typedef std::function<void(
                    const actionlib::SimpleClientGoalState&,
                    const ResultConstPtr&,
                    PLEXIL::Command*,
                    PLEXIL::AdapterExecInterface*)>
                ResultHandlerFunT;

            static Goal DefaultGoalFactoryFun(PLEXIL::Command* cmd, PLEXIL::AdapterExecInterface* aei) {
                ROS_WARN_STREAM("Default goal function called.");
                return Goal();
            }

            static void DefaultResultFun(
                    const actionlib::SimpleClientGoalState&,
                    const ResultConstPtr&,
                    PLEXIL::Command* cmd,
                    PLEXIL::AdapterExecInterface* aei)
            {
                ROS_WARN_STREAM("Default result function called.");
            }

            ActionlibHandler(
                    ros::NodeHandle nh,
                    std::string command_name,
                    std::string action_topic,
                    GoalFactoryFunT goal_fun = DefaultGoalFactoryFun,
                    ResultHandlerFunT result_fun = DefaultResultFun);

            ActionlibHandler(const ActionlibHandler&) = delete;
            ActionlibHandler& operator= (const ActionlibHandler&) = delete;

            virtual ~ActionlibHandler();

            virtual void executeCommand(PLEXIL::Command *cmd, PLEXIL::AdapterExecInterface *aei);

            virtual void abortCommand(PLEXIL::Command *cmd, PLEXIL::AdapterExecInterface *aei);

            void doneCallback(
                    const actionlib::SimpleClientGoalState& state,
                    const ResultConstPtr& result,
                    PLEXIL::Command *cmd,
                    PLEXIL::AdapterExecInterface *aei);
        private:
            std::string m_command_name;
            actionlib::SimpleActionClient<ActionT> m_client;
            GoalFactoryFunT m_goal_fun;
            ResultHandlerFunT m_result_fun;
    };

}

template<typename ActionT>
plexil_actionlib::ActionlibHandler<ActionT>::ActionlibHandler(
        ros::NodeHandle nh,
        std::string command_name,
        std::string action_topic,
        ActionlibHandler::GoalFactoryFunT goal_fun,
        ActionlibHandler::ResultHandlerFunT result_fun)
    : PLEXIL::CommandHandler(),
    m_command_name(command_name),
    m_client(nh, action_topic, false),
    m_goal_fun(goal_fun),
    m_result_fun(result_fun)
{
    PLEXIL::g_configuration->registerCommandHandler(m_command_name, this);
}

template<typename ActionT>
plexil_actionlib::ActionlibHandler<ActionT>::~ActionlibHandler()
{
}

template<typename ActionT>
void plexil_actionlib::ActionlibHandler<ActionT>::executeCommand(PLEXIL::Command *cmd, PLEXIL::AdapterExecInterface *aei) {
    // Send goal
    try {
        Goal goal = m_goal_fun(cmd, aei);

        m_client.sendGoal(goal,
                boost::bind(
                    &ActionlibHandler<ActionT>::doneCallback,
                    this, _1, _2, cmd, aei));

        aei->handleCommandAck(cmd, PLEXIL::CommandHandleValue::COMMAND_ACCEPTED);
        aei->notifyOfExternalEvent();
    } catch(std::runtime_error &err) {
        ROS_ERROR_STREAM("Error sending goal: "<<err.what());
        aei->handleCommandAck(cmd, PLEXIL::CommandHandleValue::COMMAND_DENIED);
    }
}

template<typename ActionT>
void plexil_actionlib::ActionlibHandler<ActionT>::abortCommand(PLEXIL::Command *cmd, PLEXIL::AdapterExecInterface *aei) {
    // Abort goal
    m_client.cancelGoal();
}

template<typename ActionT>
void plexil_actionlib::ActionlibHandler<ActionT>::doneCallback(
        const actionlib::SimpleClientGoalState& state,
        const ResultConstPtr& result,
        PLEXIL::Command *cmd,
        PLEXIL::AdapterExecInterface *aei)
{
    ROS_INFO_STREAM("Action goal done.");

    // Process result if necessary
    this->m_result_fun(state, result, cmd, aei);

    // Handle result status
    switch(state.state_) {
        case actionlib::SimpleClientGoalState::SUCCEEDED:
            aei->handleCommandAck(cmd, PLEXIL::CommandHandleValue::COMMAND_SUCCESS);
            aei->notifyOfExternalEvent();
            return;
        case actionlib::SimpleClientGoalState::RECALLED:
        case actionlib::SimpleClientGoalState::REJECTED:
        case actionlib::SimpleClientGoalState::PREEMPTED:
        case actionlib::SimpleClientGoalState::ABORTED:
            aei->handleCommandAck(cmd, PLEXIL::CommandHandleValue::COMMAND_FAILED);
            aei->notifyOfExternalEvent();
            return;
        default:
            ROS_ERROR_STREAM("Unknown result state: "<<state.state_);
    };
}

#endif // ifndef ACTIONLIB_HANDLER_HH

