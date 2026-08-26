# Atomic_26-27

## Description
The VEX V5RC PROS codebase and designed for and by team 45434A Atomic for the 2026-2027 Override VEX season.

### Note
As of now, nothing has been tested, and some functionality may be broken/or unintentionally left unfinished.

### What's New:
- Most of the code now uses proper units and is formatted consistently
- Ramsete controller (untested)
- Realtime motion-profiled trajectory generation
- Trajectory following
- Turn to
- Move to Pose
- Move to Point
- Chassis class was removed
- toAngularVelocity() was added to units class to handle calculating angular velocity

### ToDo
- Create a formatting guide
- Driver control recording/playback
- Verify driver control functionality
- Review RAMSETE controller code
- Review and potentially remove atomic::Pose in favor of units::pose
