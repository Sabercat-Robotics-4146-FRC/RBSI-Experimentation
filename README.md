[![CI](https://github.com/AZ-First/Az-RBSI/actions/workflows/main.yml/badge.svg)](https://github.com/AZ-First/Az-RBSI/actions/workflows/main.yml)


![AzFIRST Logo](https://github.com/AZ-First/Az-RBSI/blob/main/AZ-First-logo.png?raw=true)

# Az-RBSI
Arizona's Reference Build and Software Implementation for FRC Robots (read: "A-Z-ribsy")

## Notes

Will need to update the drive subsystem once AdvantageKit's 2025-beta is
publicly available, as it nicely wraps CTRE's swerve library in the usual
AK formalism (with all the logging!).

https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/docs/example-projects/talonfx-swerve-template.md


## Purpose

The purpose of Az-RBSI is to help Arizona FRC teams with:
* Improving robot reliability / performance during “Autonomous Play”
* Improving robot build & endurance, gameplay reliability and troubleshooting
    skills
* Providing a standardized robot “stack” to allow for quick software setup and
    troubleshooting, and make it easier for Arizona teams to form effective
    in-state alliances


## Design Philosophy

The Az-RBSI is centered around a "Reference Build" robot that allows for teams
to communicate quickly and effectivly with each other about gameplay strategy
and troubleshooting.  Additionally, the consolidation around a standard robot
design allows for easier swapping of spare parts and programming modules.

The Az-RBSI software is an outline of an FRC robot program upon which teams can
build with their particular mechanisms and designs.  It weaves together popular
and currently maintained FIRST- and community-sponsored software libraries to
provide a baseline robot functionality that combines robust reliability with
effective logging for troubleshooting.


## Library Dependencies

* [WPILib](https://docs.wpilib.org/en/stable/index.html) -- FIRST basic libraries
* [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/WHAT-IS-ADVANTAGEKIT.md) -- Logging
* [CTRE Phoenix6](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/mechanisms/swerve/swerve-overview.html) / [YAGSL](https://yagsl.gitbook.io/yagsl) -- Swerve drive library
* [PathPlanner](https://pathplanner.dev/home.html) / [Choreo](https://sleipnirgroup.github.io/Choreo/) -- Autonomous path planning
* [PhotonVision](https://docs.photonvision.org/en/latest/) -- Robot vision / tracking


## Basic Set Up Information

The Az-RBSI supports two different drive train options, both based on
AdvantageKit example projects.  Each option has a different method of creating
the necessary input files containing details of your setup (motor ID, gear
ratios, etc.):

* If you are using an entirely CTRE drivebase (*i.e.*, 8x TalonFX-controlled
motors, 4x CANcoder, 1x Pigeon 2.0), you will use the CTRE Tuner X
[Swerve Project Generator](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html)
to create the necessary ``TunerConstants.java`` file to replace the example
in ``src/main/java/frc/robot/generated``.

* For all other drivebases (all-REV, blended CTRE/REV, etc.), you will use the
[YAGSL configuration tool](https://broncbotz3481.github.io/YAGSL-Example/) to
create the necessary ``*.json`` files to replace the examples in
``src/main/deploy/swerve``.

When editing the ``Constants.java`` file, you will need to specify which
drivetrain type is being used via the ``swerveType`` variable near the top of
the file (options are ``SwerveType.PHOENIX6`` or ``SwerveType.YAGSL``) so that
the code reads in the proper configuration file(s).
