#pragma once

class Action {
public:
    /**
     * Returns whether or not the code has finished execution. When implementing this interface, this method is used by
     * the runAction method every cycle to know when to stop running the action
     *
     * @return boolean
     */
    virtual bool isFinished() = 0;

    /**
     * Called by runAction in AutoModeBase iteratively until isFinished returns true. Iterative logic lives in this
     * method
     */
    virtual void update() = 0;

    /**
     * Run code once when the action finishes, usually for clean up
     */
    virtual void done() = 0;

    /**
     * Run code once when the action is started, for set up
     */
    virtual void start() = 0;
};