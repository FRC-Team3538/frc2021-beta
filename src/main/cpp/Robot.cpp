/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/Timer.h>

#include "Drivetrain.h"

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override
  {
    auto config = frc::TrajectoryConfig(2_mps, 2_mps_sq);

    // m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    //     frc::Pose2d(2_m, 2_m, 0_deg), {}, frc::Pose2d(5_m, 2_m, 0_deg),
    //     config);

    config.SetReversed(true);
    m_trajectory_back = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(3_m, 6_m, 0_deg), {}, frc::Pose2d(2_m, 2_m, 0_deg),
        config);
  }

  void RobotPeriodic() override { m_drive.Periodic(); }

  void AutonomousInit() override
  {
    m_timer.Reset();
    m_timer.Start();
    m_drive.ResetOdometry(m_trajectory_back.InitialPose());
    m_state = 0;
  }

  void AutonomousPeriodic() override
  {
    auto elapsed = m_timer.Get();

    frc::Trajectory::State reference;
    // switch(m_state) {
    //   case 0:
    //     reference = m_trajectory.Sample(elapsed);
    //     break;
    //   case 1:
        reference = m_trajectory_back.Sample(elapsed);
    //     break;
    //   default:
    //     m_drive.Drive(0_mps, 0_deg_per_s);
    //     return;
    // }
    
    auto speeds = m_ramsete.Calculate(m_drive.GetPose(), reference);
    m_drive.Drive(speeds.vx, speeds.omega);

    // if(elapsed > 5_s)
    // {
    //   m_state++;
    //   m_timer.Reset();
    // }
  }

  void TeleopPeriodic() override
  {
    const auto xSpeed = -m_speedLimiter.Calculate(
                            deadband(m_controller.GetRawAxis(1))) *
                        Drivetrain::kMaxSpeed;

    auto rot = -m_rotLimiter.Calculate(
                   deadband(m_controller.GetRawAxis(3))) *
               Drivetrain::kMaxAngularSpeed;

    m_drive.Drive(xSpeed, rot);
  }

  void SimulationPeriodic() override { m_drive.SimulationPeriodic(); }

  double deadband(double in)
  {

    if (abs(in) < 0.05)
      return 0.0;

    return in;
  }

private:
  frc::XboxController m_controller{0};

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_speedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  Drivetrain m_drive;
  frc::Trajectory m_trajectory;
  frc::Trajectory m_trajectory_back;
  frc::RamseteController m_ramsete;
  frc2::Timer m_timer;
  int m_state = 0;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
