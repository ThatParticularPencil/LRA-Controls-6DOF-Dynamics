#include <iostream>

//Given: F_vector = [net Fx, net Fy, net Fz, net Mx, net My, Net Mz]T, mass, time
//Output: State_vector = [x, y, z, yaw, pitch, roll, x_dot, y_dot, z_dot, yaw_dot, pitch_dot, roll_dot, mass, time]T

//F_vector
struct F_vector {
    double fx, fy, fz, mx, my, mz;
};

//State_vector
struct State_vector {
    double x,y,z, yaw, p, r,vx, vy, vz, yaw_dot, p_dot, r_dot, mass, time;
};



int main(){
    //Example:
    double mass = 1200.0; 
    double time_step = 0.01; 
    F_vector forces = {5000.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 5000 Newtons of force in X  
    State_vector current_state = {0,0,0, 0,0,0, 10.0,0,0, 0,0,0, mass, 0.0}; // Starting at 10 m/s

    // Initialize next state as a copy of current
    State_vector output_state = current_state;

    /////////////////////////////////
    //Linear Integration Formulas here:
    // Goal: Use netF = m*a --> to get v and s vectors (which is [x_dot, y_dot, z_dot] and [x,y,z])

    //Idea is:
        // using a = F/m and a = dv/dt
        // integral a dt (t1 to t2) = integral dv (v1 to v2)
        // ax(t2 - t1) = v2 - v1 = delta_v
        // v2 = v1 + delta_v
        // v2 is the output velocity

        // using v= dx/dt
        // integral v dt (t1 to t2) = integral dx (x1 to x2)
        // v2(t2 - t1) = x2 - x1 = delta_x
        // x2 = x1 + delta_x
        // x2 is the output position

    /////////////////////////////////

    //netFx = max:
    double ax = forces.fx/current_state.mass; 
    double delta_vx = ax*(time_step); 
    output_state.vx = current_state.vx + delta_vx; //get x velocity

    double delta_x = (output_state.vx * time_step);
    output_state.x = current_state.x + delta_x; // get x position


    //netFy = may
    double ay = forces.fy/current_state.mass;
    double delta_vy = ay*(time_step);
    output_state.vy = current_state.vy + delta_vy;

    double delta_y = (output_state.vy * time_step);
    output_state.y = current_state.y + delta_y;

    //netFz = maz
    double az = forces.fz/current_state.mass;
    double delta_vz = az*(time_step);
    output_state.vz = current_state.vz + delta_vz;

    double delta_z = (output_state.vz * time_step);
    output_state.z = current_state.z+ delta_z;

    //Angular Integration Formulas here:
    //Use netM= I*alpha --> w, theta

    std::cout << "Output State:\n";
    std::cout << "x  = " << output_state.x << "\n";
    std::cout << "y  = " << output_state.y << "\n";
    std::cout << "z  = " << output_state.z << "\n";

    std::cout << "vx = " << output_state.vx << "\n";
    std::cout << "vy = " << output_state.vy << "\n";
    std::cout << "vz = " << output_state.vz << "\n";


    return 0;

};

