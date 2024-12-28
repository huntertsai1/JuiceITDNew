# FTC Robot Controller

This repository contains the FTC robot controller code for our team's competition robot. The code is structured using a subsystem-based architecture for better organization and maintainability.

## Project Structure

```
TeamCode/
├── src/main/java/org/firstinspires/ftc/teamcode/
│   ├── Robot.java              # Main robot class
│   ├── auton/                  # Autonomous mode opmodes
│   ├── commands/               # Command patterns for robot actions
│   ├── roadrunner/            # Path planning and motion control
│   ├── subsystems/            # Robot subsystems (lift, arm, claw, etc.)
│   ├── teleop/                # Teleop mode opmodes
│   └── util/                  # Utility classes and helpers
```

## Dependencies

- FTC SDK
- Road Runner (for advanced path planning)
- FTCLib (for additional utilities)

## License

This project is licensed under the MIT License - see the LICENSE file for details.
