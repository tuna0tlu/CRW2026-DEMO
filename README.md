# CRW 2026 Swerve Drive Demo Project

This is a simulation project for a FIRST Robotics Competition (FRC) robot featuring a swerve drive chassis. The project is designed to meet all the requirements outlined in the `CRW 2026 Software Assignment` document.

## 🤖 Key Features

-   **Swerve Drive:** High-maneuverability drivetrain with four independently steered and driven modules.
-   **Hybrid Localization:** The robot's on-field position is accurately estimated by fusing wheel odometry with AprilTag tracking via [PhotonVision](vendordeps/photonlib.json).
-   **Dynamic Autonomous:** Utilizes the [PathPlanner](vendordeps/PathplannerLib-2025.2.7.json) library to generate dynamic paths at runtime and execute randomized tasks based on the current alliance color.
-   **Advanced TeleOp Controls:** Intuitive and smart joystick controls designed to assist the driver.

---

##  Controls (TeleOp Mode)

The robot is controlled using a standard Xbox controller.

| Button/Axis                  | Function                                                                 |
| ---------------------------- | ------------------------------------------------------------------------ |
| **Left Analog Stick (Y-Axis)** | Translates the robot forward and backward.                               |
| **Left Analog Stick (X-Axis)** | Translates (strafes) the robot left and right.                           |
| **Right Analog Stick (X-Axis)**| Rotates the robot.                                                       |
| **Start Button** | Toggles between **Field Oriented** and **Robot Oriented** drive modes. |
| **Y Button** | Rotates the robot **180 degrees** from its current heading.              |
| **A Button (Hold)** | Forces the robot to constantly face the center of the field (`FIELD_CENTER`). |

---

## 🎯 Autonomous Mode

The autonomous routine executes the following sequence of tasks at the beginning of the match:

1.  **Path "A to B":** Follows a pre-defined path named "A to B" created in PathPlanner.
2.  **Cyclic Tasks:**
    -   Travels to a random **Coral Station** based on the current alliance color.
    -   Waits for **2 seconds** at the destination.
    -   Travels to a random **Reef** location based on the current alliance color.
    -   Waits for **2 seconds** at the destination.
    -   This cycle repeats until the autonomous period ends.

---

##  Technical Stack

### Software

-   **WPILib 2025:** The standard FRC control system and library framework.
-   **PathplannerLib:** A library for creating and following advanced autonomous paths.
-   **PhotonVision (photonlib):** A vision processing library used for AprilTag detection and pose estimation.
-   **REVLib:** Vendor library for controlling REV Robotics hardware (Spark MAX, NEO).

### Compatible Hardware

-   **Motors:** REV NEO Brushless Motors.
-   **Motor Controllers:** REV Spark MAX.
-   **IMU:** Studica NavX MXP.
-   **Encoders:** Thrifty Absolute Magnetic Encoder (Analog).
