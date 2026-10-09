#include <iostream>

//Given: F_vector = [net Fx, net Fy, net Fz, net Mx, net My, Net Mz]T, mass, time
//Output: State_vector = [x, y, z, q, p, r, x_dot, y_dot, z_dot, q_dot, p_dot, r_dot, mass, time]T

//F_vector
struct F_vector {
    double fx, fy, fz, mx, my, mz;
};

//State_vector
struct State_vector {
    double x,y,z, p, q, r,vx, vy, vz, p_dot, q_dot, r_dot, mass, time, inertia;
};

struct Inertia {
    double Ixx, Iyy, Izz;
};


// Helper function to normalize a quaternion to keep it valid HERE: if needed?


int main(){
    //Example:
    double mass = 1200.0; 
    double time_step = 0.01; 
    F_vector forces = {5000.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 5000 Newtons of force in X  
    State_vector current_state = {0,0,0, 0,0,0, 10.0,0,0, 0,0,0, mass, 0.0}; // Starting at 10 m/s4
    Inertia inertia = { 100.0, 500.0, 500.0 }; 

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

    //Step 1: solve for a = F/m
    double ax = forces.fx/current_state.mass; 
    double ay = forces.fy/current_state.mass;
    double az = forces.fz/current_state.mass;

    //Step 2: get change in v = a*t
    double delta_vx =  ax*(time_step); 
    double delta_vy = ay*(time_step);
    double delta_vz = az*(time_step);

    //Step 3: get v = v_current + at
    output_state.vx = current_state.vx + delta_vx; //could use output_state.vx ? i'll see later
    output_state.vy = current_state.vy + delta_vy;
    output_state.vz = current_state.vz + delta_vz;

    //Step 4: get change in position
    double delta_x = (output_state.vx * time_step);
    double delta_y = (output_state.vy * time_step);
    double delta_z = (output_state.vz * time_step);

    //Step 5: Get x = x_current + vt
    output_state.x = current_state.x + delta_x; // get x position
    output_state.y = current_state.y + delta_y;
    output_state.z = current_state.z+ delta_z;

    std::cout << "Output State:\n";
    std::cout << "x  = " << output_state.x << "\n";
    std::cout << "y  = " << output_state.y << "\n";
    std::cout << "z  = " << output_state.z << "\n";

    std::cout << "vx = " << output_state.vx << "\n";
    std::cout << "vy = " << output_state.vy << "\n";

    //Angular Integration Formulas here:
    //Use netM= I*alpha --> w, theta
    //similar concept

    //Step 1: solve for angular acceleration alpha

    double alpha_p = (forces.mx - (inertia.Izz - inertia.Iyy) * q * r) / inertia.Ixx; 
    double alpha_q =  (forces.my - (inertia.Ixx - inertia.Izz) * r * p)  / inertia.Iyy;
    double alpha_r =  (forces.mz - (inertia.Iyy - inertia.Ixx) * p * q) / inertia.Izz;

    //Step 2: get change in w
    delta_wp = alpha_p * time_step;
    delta_wq = alpha_q * time_step;
    delta_wr = alpha_r * time_step;

    //Step 3: get w = w_current + alpha*t
    output_state.p = current_state.p + delta_wx
    output_state.q = current_state.q + delta_wy
    output_state.r = current_state.r + delta_wz


    //Step 4: QUATERNION INTEGRATION!! for angles
    // Calculate quaternion derivative
    // Scalar-first quaternion: qw, qx, qy, qz
    // Body-to-world orientation convention

    //Get Q = [qw, qx, qy, qz] 
    double qw = current_state.qw;
    double qx = current_state.qx;
    double qy = current_state.qy;
    double qz = current_state.qz;

    // neeed to use NEW body angular velocity (found above)
    double p_new = output_state.p;
    double q_new = output_state.q;
    double r_new = output_state.r;

    //Get Q_dot =1/2 Q * [0, p,q,r]
    double qw_dot = -0.5 * (qx * p_new + qy * q_new + qz * r_new);
    double qx_dot =  0.5 * (qw * p_new + qy * r_new - qz * q_new);
    double qy_dot = 0.5 * (qw * q_new + qz * p_new - qx * r_new);
    double qz_dot = 0.5 * (qw * r_new + qx * q_new - qy * p_new);

    //change in Q
    delta_qw = qw_dot * time_step;
    delta_qx = qx_dot * time_step;
    delta_qy = qy_dot * time_step;
    delta_qz = qz_dot * time_step;

    //get new Q
    output_state.qw = qw + delta_qw
    output_state.qx = qx + delta_qx
    output_state.qy = qy + delta_qy
    output_state.qz = qz + delta_qz

    //normalized new Q
    normalizeQuaternion(output_state)

    //FINAL UPDATE/OUTPUT:
    output_state.time = current_state.time + time_step;


    return 0;

};
