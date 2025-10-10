from casadi import *

def pf_model():

    Xv_bar = MX.sym('Xv_bar', 8)
    Eps_ = MX.sym('Eps_', 1)
    rho_ = MX.sym('rho_', 1)
    theta_ = MX.sym('theta_', 1)

    v_bar_min_ = -1.0
    u_ref_min_ = 0.5
    u_tar_min_ = 0.4
    r_ref_min_ = -0.3
    Delta_u_ref_min_ = -0.5
    Delta_u_tar_min_ = -0.5
    Delta_r_ref_min_ = -0.5

    v_bar_max_ = 1.0
    u_ref_max_ = 2.0
    u_tar_max_ = 3.0
    r_ref_max_ = 0.3
    Delta_u_ref_max_ = 0.5
    Delta_u_tar_max_ = 0.5
    Delta_r_ref_max_ = 0.5

    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "PF_asv_model"

    # Variables de estado (x_e_bar, y_e_bar, psi, w, v_bar, uref, u_tar, r_ref)

    x_e_bar = MX.sym("x_e_bar")
    y_e_bar = MX.sym("y_e_bar")
    psi = MX.sym("psi")
    w = MX.sym("w")
    v_bar = MX.sym("v_bar")
    u_ref = MX.sym("u_ref")
    u_tar = MX.sym("u_tar")
    r_ref = MX.sym("r_ref")
    states = vertcat(x_e_bar, y_e_bar, psi, w, v_bar, u_ref, u_tar, r_ref)

    # xdot
    x_e_bar_dot = MX.sym("x_e_bar_dot")
    y_e_bar_dot = MX.sym("y_e_bar_dot")
    psi_dot = MX.sym("psi_dot")
    w_dot = MX.sym("w_dot")
    v_bar_dot = MX.sym("v_bar_dot")
    u_ref_dot = MX.sym("u_ref_dot")
    u_tar_dot = MX.sym("u_tar_dot")
    r_ref_dot = MX.sym("r_ref_dot")

    xdot = vertcat(x_e_bar_dot, y_e_bar_dot, psi_dot, w_dot, v_bar_dot, u_ref_dot, u_tar_dot, r_ref_dot)

    # Variables de control (d_u_ref, d_u_tar, d_r_ref)
    d_u_ref = MX.sym('d_u_ref')
    d_u_tar = MX.sym('d_u_tar')
    d_r_ref = MX.sym('d_r_ref')
    controls = vertcat(d_u_ref, d_u_tar, d_r_ref)

    # Variables del path
    coef = MX.sym('coef', 4)  # coef = [a, b, c , d]

    disturbances = vertcat(coef)

    # Número de estados, controles y entradas
    n_states = states.size(1) 
    n_controls = controls.size(1)
    n_inputs = disturbances.size(1)

    # Definir la dinámica
    dx     = coef[0]*sin(w) + coef[2]# 30*sin(w) # coef[0] si x = -30*cos(w)
    dy     = coef[1]*cos(w) + coef[3]# 30*cos(w) # coef[1] si y = 30*sin(w)
    ddx    = coef[0]*cos(w)   # 30*cos(w)  # 0
    ddy    = -coef[1]*sin(w)  # -30*sin(w) # 0
    d3x    = -coef[0]*sin(w)  # -30*sin(w) # 0
    d3y    = -coef[1]*cos(w)  # -30*cos(w) # 0
    phi_c  = atan2(dy, dx)
    F = sqrt(dx*dx + dy*dy)
    dphi_c = (ddy*dx - ddx*dy)/(dx*dx + dy*dy)
    ddphi_c= ( (d3y*dx - d3x*dy)*(dx*dx + dy*dy) - 2 *(ddy*dx - ddx*dy) * (ddx*dx + dy*ddy) )/((dx*dx + dy*dy)*(dx*dx + dy*dy))

    M = rho_*cos(phi_c+theta_)
    N = rho_*sin(phi_c+theta_)

    phi = atan2(dy + dphi_c * M ,dx - dphi_c * N)
    dphi_p = ((ddy*dx - ddx*dy) + (dphi_c**3*rho_**2) + N*(-ddy*dphi_c- dphi_c**2*dx + ddphi_c*dy) + M*(-ddx*dphi_c + dphi_c**2*dy + ddphi_c*dx))
    dphi = dphi_p / ( (dx**2+dy**2) + 2*dphi_c*(dy*M-dx*N) + (dphi_c**2*rho_**2))

    v = v_bar - Eps_*r_ref

    x_dot = MX.zeros(n_states)

    x_dot[0] = u_ref*cos(psi - phi) - v_bar*sin(psi - phi) + u_tar*((1/F)*dphi*y_e_bar - 1)
    x_dot[1] = u_ref*sin(psi - phi) + v_bar*cos(psi - phi) - u_tar*(1/F)*dphi*x_e_bar
    x_dot[2] = r_ref
    x_dot[3] = u_tar/F
    x_dot[4] = (Xv_bar[0] * v * fabs(v) + Xv_bar[1] * v * fabs(r_ref) + Xv_bar[2] * r_ref * fabs(v) + Xv_bar[3] * r_ref * fabs(r_ref)
                + Xv_bar[4] * u_ref * v  + Xv_bar[5] * u_ref * r_ref + Xv_bar[6] * v + Xv_bar[7] * r_ref)
    
    x_dot[5] = d_u_ref
    x_dot[6] = d_u_tar
    x_dot[7] = d_r_ref

    # Modelo dinámico final
    f_expl = x_dot  # Aquí usas la dinámica que definimos

    f_impl = xdot - f_expl
   
    # algebraic variables
    z = vertcat([])

    # parameters
    p = vertcat(
        Xv_bar,
        Eps_,
        rho_,
        theta_,
        coef)  # Concatenamos los parámetros constantes y los disturbios
    
    # constraint on forces    

    constraint.v_bar_min_ = v_bar_min_
    constraint.u_ref_min_ = u_ref_min_
    constraint.u_tar_min_ = u_tar_min_
    constraint.r_ref_min_ = r_ref_min_
    constraint.Delta_u_ref_min_ = Delta_u_ref_min_
    constraint.Delta_u_tar_min_ = Delta_u_tar_min_
    constraint.Delta_r_ref_min_ = Delta_r_ref_min_

    constraint.v_bar_max_ = v_bar_max_
    constraint.u_ref_max_ = u_ref_max_
    constraint.u_tar_max_ = u_tar_max_
    constraint.r_ref_max_ = r_ref_max_
    constraint.Delta_u_ref_max_ = Delta_u_ref_max_
    constraint.Delta_u_tar_max_ = Delta_u_tar_max_
    constraint.Delta_r_ref_max_ = Delta_r_ref_max_    

    # Define model struct
    model.f_expl_expr = f_expl
    model.x = states
    # model.xdot = statesdot
    model.u = controls
    model.z = z
    model.p = p
    model.name = model_name
    model.f_impl_expr = f_impl
    model.xdot = xdot

    # Define initial conditions
    model.x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0])

    return model, constraint