#include "main.h"
#include <cmath>

// ============================================================
// MECHANISM DEVICES
// ============================================================

// Pivot Motors (-6 and 19 based on your configuration)
pros::Motor pivot_left(-6);
pros::Motor pivot_right(19);

// Rotation Sensor on Port 4
// Zero was set on the Brain with the pivot straight up
pros::Rotation pivot_rotation(4);

// Intake & Elevator Motors
pros::Motor elevator_left(5);
pros::Motor elevator_right(-8);
pros::Motor intake_motor(11);

int elevator_power = 127;

// ======================= =====================================
// PIVOT TARGET CONTROL
// ============================================================

// Pivot target in degrees
// 0 degrees = the zero position you set on the Brain
// 90 degrees = 90 degrees away from that position
double pivot_target = 0.0;

bool pivot_a_last = false;

// ============================================================
// PIVOT PID CONSTANTS
// ============================================================

// PID constants
// These will probably need to be tuned for your pivot
double pivot_kP = 3.0;
double pivot_kI = 0.0;
double pivot_kD = 0.02;

// PID variables
double pivot_integral = 0.0;
double pivot_previous_error = 0.0;

// Prevents the integral from getting too large
double pivot_integral_limit = 1000.0;

// Maximum motor power
double pivot_max_power = 127.0;

// ============================================================
// PIVOTING ELEVATOR GRAVITY COMPENSATION
// ============================================================

// Gravity compensation for the pivoting elevator
//
// 0 degrees = elevator straight up
// 90 degrees = elevator horizontal
//
// This value controls how much power is added to fight gravity.
// Start with this value and increase it if the elevator drops.
//
// If the pivot tries to move in the wrong direction,
// change this value to a negative number.
double pivot_kG = -15.0;

// ============================================================
// PIVOT PID CONTROL
// ============================================================

void pivot_pid_control() {

  // Read the Rotation Sensor position
  // PROS reports the position in centidegrees, so divide by 100
  double current = pivot_rotation.get_position() / 100.0;

  // Calculate how far the pivot is from the target
  double error = pivot_target - current;

  // Calculate how quickly the error is changing
  double derivative = (error - pivot_previous_error) / 0.020;

  // Add error to the integral
  pivot_integral += error;

  // Prevent integral windup
  if (pivot_integral > pivot_integral_limit) {
    pivot_integral = pivot_integral_limit;
  }

  if (pivot_integral < -pivot_integral_limit) {
    pivot_integral = -pivot_integral_limit;
  }

  // Calculate gravity compensation
  //
  // Since 0 degrees is straight up,
  // gravity becomes strongest as the elevator approaches 90 degrees.
  //
  // sin(0) = 0
  // sin(90) = 1
  double gravity_power =
      pivot_kG * sin(current * M_PI / 180.0);

  // Calculate PID motor power
  double power =
      (pivot_kP * error) +
      (pivot_kI * pivot_integral) +
      (pivot_kD * derivative) +
      gravity_power;

  // Limit motor power
  if (power > pivot_max_power) {
    power = pivot_max_power;
  }

  if (power < -pivot_max_power) {
    power = -pivot_max_power;
  }

  // Stop applying PID power when very close to the target
  if (fabs(error) < 1.0) {

    // Keep gravity compensation active
    // so the pivot can hold its position
    power = gravity_power;
  }

  // Move both pivot motors
  pivot_left.move(power);
  pivot_right.move(power);

  // Save the current error for the next PID calculation
  pivot_previous_error = error;
}

// ============================================================
// RESET PIVOT PID
// ============================================================

void reset_pivot_pid() {

  // Reset the integral so old error does not affect the new target
  pivot_integral = 0.0;

  // Reset the previous error to the current error
  double current = pivot_rotation.get_position() / 100.0;
  pivot_previous_error = pivot_target - current;
}

// Background Task for Pivot Movement
void mechanism_task_fn(void *) {
  while (true) {

    // Run the pivot PID controller
    pivot_pid_control();

    // Wait 20 milliseconds before running again
    pros::delay(20);
  }
}

// ============================================================
// INITIALIZE
// ============================================================

void initialize() {
  ez::ez_template_print();
  pros::delay(500);

  // Set brake modes to HOLD so motor holds position when stopped
  pivot_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  pivot_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // DO NOT reset the Rotation Sensor here.
  // The zero was already set on the Brain with the pivot straight up.

  // Default target at start
  pivot_target = 0.0;

  // Reset PID variables
  reset_pivot_pid();

  // Start background task
  pros::Task mechanism_task(mechanism_task_fn);
}

void disabled() {}
void competition_initialize() {}

void autonomous() {}

// ============================================================
// OPERATOR CONTROL
// ============================================================

void opcontrol() {
  while (true) {

    // ========================================================
    // A BUTTON TOGGLE FOR 90 DEGREES
    // ========================================================

    bool a_pressed =
        master.get_digital(pros::E_CONTROLLER_DIGITAL_A);

    // Only activate once when the A button is pressed
    // instead of repeatedly activating while it is held
    if (a_pressed && !pivot_a_last) {

      // Toggle between 0 degrees and 90 degrees
      if (fabs(pivot_target - 90.0) < 0.1) {
        pivot_target = 0.0;   // Move back down
      } else {
        pivot_target = 90.0;  // Move to 90 degrees
      }

      // Reset PID variables when changing targets
      reset_pivot_pid();
    }

    // Remember the current A button state
    pivot_a_last = a_pressed;

    // ========================================================
    // INTAKE & ELEVATOR
    // ========================================================

    // Intake control
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake_motor.move(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake_motor.move(-127);
    } else {
      intake_motor.move(0);
    }

    // Elevator control
    bool l1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool l2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    if (l1 && !l2) {
      elevator_left.move(elevator_power);
      elevator_right.move(elevator_power);
    } else if (l2 && !l1) {
      elevator_left.move(-elevator_power);
      elevator_right.move(-elevator_power);
    } else {
      elevator_left.move(0);
      elevator_right.move(0);
    }

    pros::delay(20);
  }
}