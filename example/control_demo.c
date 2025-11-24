
/**
 * @author Andrea Pineda
 * @date Created 6 Oct 2025
 * Last Modified 6 Oct 2025
 * 
 * Example for learning to use the robotat_control library of Robotat Linalg
 */

// ---------------------------------------------------------------------------
// Libraries
// ---------------------------------------------------------------------------

#include "matf32.h"
#include "robotat_control.h"

// ---------------------------------------------------------------------------
// Matrices and variables definitions
// ---------------------------------------------------------------------------

static float m1data[MAX_MAT_SIZE];
static matf32_t m1;
static float m2data[MAX_MAT_SIZE];
static matf32_t m2;
static float m3data[MAX_MAT_SIZE];
static matf32_t m3;
static float m4data[MAX_MAT_SIZE];
static matf32_t m4;
static float m5data[MAX_MAT_SIZE];
static matf32_t m5;



// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(void)
{
    // ---------------------------------------------------------------------------
    // PID functions
    // ---------------------------------------------------------------------------

    // A PID controller is initialized as follows:

    ctr_pid_t pid; // Create pid object
    
    // Define kp, ki and kd constants
    float kp = 0.3;
    float ki = 0.2;
    float kd = 0.1;

    // Initialize the PID, selecting the discretization method
    ctr_pid_init(&pid, kp, ki, kd, FWD_EULER, 0);
    ctr_pid_print(&pid); // Print the PID data in the terminal

    /*
        The available discretization methods are:
            Pure Discrete   -> PURE_DISCRETE
            Forward Euler   -> FWD_EULER   
            Backward Euler  -> BWD_EULER
            Tustin          -> TUSTIN
    
        where the constants at the right side are the corresponding argument to select
        each method in the ctr_pid_init function.

        To set specific limits to the i parameter, write 1 (instead of 0) in the sixth paremeter
        and then the values corresponding to: sampling period (dt), tau, i_min, i_max

            ctr_pid_init(&pid, kp, ki, kd, TUSTIN, 1, dt, tau, i_min, i_max);

        Setting 0 allows to skip over the dt, tau, i_min and i_max arguments. In that case,
        i_max is set by default to the limit of a float value (an extremely high value).
    */

    kp = 0.4;
    ki = 0.5;
    kd = 0.6;

    ctr_pid_set_gains(&pid, kp, ki, kd); // Manually set new gains in the PID
    ctr_pid_print(&pid);

    float r_k = 5;
    float y_k = 0;
    float u_k = 0;

    u_k = ctr_pid_update(&pid, r_k, y_k); // See comment below

    /*
        ctr_pid_update implements updates the controller output u_k based on the discretization method
        used for the PID.

        
    
    */
}   