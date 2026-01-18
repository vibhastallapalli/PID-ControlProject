#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <iomanip>

// Physics Constants
const double MASS = 100.0;          // kg
const double AIR_DENSITY = 1.225;   // kg/m^3
const double DRAG_COEFF = 0.9;      // Cd
const double FRONTAL_AREA = 0.5;    // m^2
const double GRAVITY = 9.81;        // m/s^2
const double ROLL_RESIST_COEFF = 0.01; // Crr

// Simulation Constants
const double DT = 0.05;             // Time step (s)
const double SIM_DURATION = 30.0;   // Total simulation time (s)
const double MAX_THRUST = 500.0;    // Max force motor can provide (N) - arbitrary reasonable limit for scaling throttle

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0) {}

    double update(double setpoint, double measured_value, double dt) {
        double error = setpoint - measured_value;
        
        // Proportional term
        double P = kp_ * error;
        
        // Integral term with anti-windup clamping
        integral_ += error * dt;
        
        // Clamp integral to avoid windup
        double integral_limit = 100.0; // Arbitrary limit suitable for this scale
        if (integral_ > integral_limit) integral_ = integral_limit;
        if (integral_ < -integral_limit) integral_ = -integral_limit;
        
        // Derivative term
        double D = kd_ * (error - prev_error_) / dt;
        prev_error_ = error;

        double output = P + ki_ * integral_ + D;
        
        return output;
    }

    // Reset for safety if needed
    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }

private:
    double kp_, ki_, kd_;
    double integral_;
    double prev_error_;
};

int main() {
    // PID Gains - Tuned manually for this system
    double Kp = 15.0;
    double Ki = 2.0;
    double Kd = 0.5;
    
    PIDController pid(Kp, Ki, Kd);

    std::ofstream outfile("results.csv");
    outfile << "Time,Setpoint,Velocity,Throttle,Disturbance,Energy,Efficiency\n";

    double velocity = 0.0;
    double current_time = 0.0;
    double total_energy = 0.0;

    std::cout << "Starting simulation..." << std::endl;

    while (current_time <= 20.0) { // Duration changed to 20s as per profile request (0-20s)
        // 1. Determine Setpoint based on Step Profile
        double setpoint = 0.0;
        if (current_time < 5.0) {
            setpoint = 5.0;
        } else if (current_time < 10.0) {
            setpoint = 15.0;
        } else {
            setpoint = 25.0;
        }

        // 2. Calculate Disturbance (Keep hill logic if desired, or remove? Request said "Hill Disturbance: Apply a sudden +150N...". 
        // The new request says "Variable Setpoint...". It doesn't explicitly say remove the hill, 
        // but often these tests isolate variables. 
        // However, the prompt implies "Modify main.cpp... Logic Changes: Variable Setpoint...". 
        // It didn't say "Remove Hill". I'll keep the hill logic but maybe adjust time? 
        // The hill was at 8s. That overlaps with the 15m/s section.
        // Let's keep it to see how it handles it, unless stated otherwise. 
        // Actually, for "Optimal Cruise Speed" testing, usually you want steady state. 
        // But the prompt was "Logic Changes: ... Hill Disturbance: Apply a sudden +150N ... " in the FIRST request.
        // The SECOND request says "Logic Changes: Variable Setpoint... Efficiency Tracking...". 
        // It doesn't explicitly say to remove the hill.
        // I will keep the hill logic as it makes the simulation more interesting (and part of the physical system defined previously).
        
        double disturbance = 0.0;
        if (current_time >= 8.0) {
            disturbance = 150.0; 
        }

        // 3. PID Control
        double throttle_command = pid.update(setpoint, velocity, DT);
        
        // 4. Actuator Saturation & Scaling
        double throttle = throttle_command;
        if (throttle > 1.0) throttle = 1.0;
        if (throttle < 0.0) throttle = 0.0;

        double engine_force = throttle * MAX_THRUST;

        // 5. Physics Model
        double force_drag = 0.5 * AIR_DENSITY * DRAG_COEFF * FRONTAL_AREA * velocity * velocity;
        double force_roll = ROLL_RESIST_COEFF * MASS * GRAVITY;
        
        double net_force = engine_force - force_drag - force_roll - disturbance;
        
        double acceleration = net_force / MASS;
        
        // Euler integration
        velocity += acceleration * DT;
        if (velocity < 0) velocity = 0; 

        // 6. Energy & Efficiency
        // Power = Force * Velocity
        double power = engine_force * velocity; // Mechanical power output of motor
        total_energy += power * DT;

        // Instantaneous Efficiency: meters traveled per Joule? 
        // Or ratio of useful work? 
        // The user asked for "Efficiency vs Velocity". 
        // Common metric for vehicles is "Wh/mi" or "measure of efficiency".
        // User definition: "instantaneous_efficiency". 
        // Let's define it as kinetic energy gain / input energy? Or simply inverse of power consumption?
        // User didn't strictly define "instantaneous_efficiency" formula, but said "Efficiency Tracking...".
        // Wait, commonly "Efficiency" map is a motor property. Here we have a simple force source.
        // Maybe "m/J" (distance per energy) = velocity / power (m/s / J/s = m/J).
        // Let's use that as it's a "miles per gallon" equivalent.
        double efficiency = 0.0;
        if (std::abs(power) > 1e-4) {
            efficiency = velocity / power;
        }

        // 7. Data Logging
        outfile << current_time << "," << setpoint << "," << velocity << "," << throttle << "," << disturbance 
                << "," << total_energy << "," << efficiency << "\n";

        current_time += DT;
    }

    outfile.close();
    std::cout << "Simulation complete. Data written to results.csv" << std::endl;

    return 0;
}
