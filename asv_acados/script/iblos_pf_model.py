from casadi import *
import casadi as ca

def pf_model():

    Xv_bar = MX.sym('Xv_bar', 8)
    Eps_ = MX.sym('Eps_', 1)

    # Variables del path
    coef = MX.sym('coef', 6)  # coef = [a, b, c , d, e, f]

    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "iblos_pf_model"

    # Variables de estado (x_e_bar, y_e_bar, psi, w, v_bar)
    x_e_bar = MX.sym("x_e_bar")
    y_e_bar = MX.sym("y_e_bar")
    psi = MX.sym("psi")
    w = MX.sym("w")
    v_bar = MX.sym("v_bar")
    states = vertcat(x_e_bar, y_e_bar, psi, w, v_bar)

    # xdot
    x_e_bar_dot = MX.sym("x_e_bar_dot")
    y_e_bar_dot = MX.sym("y_e_bar_dot")
    psi_dot = MX.sym("psi_dot")
    w_dot = MX.sym("w_dot")
    v_bar_dot = MX.sym("v_bar_dot")
    xdot = vertcat(x_e_bar_dot, y_e_bar_dot, psi_dot, w_dot, v_bar_dot)

    # Variables de control 
    u_ref = MX.sym("u_ref")
    u_tar = MX.sym("u_tar")
    r_ref = MX.sym("r_ref")
    controls = vertcat(u_ref, u_tar, r_ref,)

    x_e = MX.sym("x_e")
    y_e = MX.sym("y_e")
    output = vertcat(x_e, y_e)

    # Número de estados, controles y entradas
    n_states = states.size(1) 
    n_oututs = output.size(1) 

    # Definir la dinámica
    dx     = -coef[0]*sin(w) + coef[2] + 2*coef[4]*cos(2*w)  #  x = coef[0]*cos(w)
    dy     =  coef[1]*cos(w) + coef[3] + coef[5]*cos(w+pi/2) #  y = coef[1]*sin(w)
    ddx    = -coef[0]*cos(w) - 4*coef[4]*sin(2*w)
    ddy    = -coef[1]*sin(w) - coef[5]*sin(w+pi/2)
    phi    = atan2(dy, dx)
    F = sqrt(dx*dx + dy*dy)
    dphi = (ddy*dx - ddx*dy)/(dx*dx + dy*dy)


    z_exp = MX.zeros(n_oututs)
    z_exp[0] = x_e_bar - Eps_*cos(psi - phi)
    z_exp[1] = y_e_bar - Eps_*sin(psi - phi)

    v = v_bar - Eps_*r_ref

    # x_e_bar, y_e_bar, psi, w, v_bar
    x_dot = MX.zeros(n_states)
    x_dot[0] = u_ref*cos(psi - phi) - v_bar*sin(psi - phi) + u_tar*((1/F)*dphi*y_e_bar - 1)
    x_dot[1] = u_ref*sin(psi - phi) + v_bar*cos(psi - phi) - u_tar*(1/F)*dphi*x_e_bar
    x_dot[2] = r_ref
    x_dot[3] = u_tar/F
    x_dot[4] = (Xv_bar[0] * v * fabs(v) + Xv_bar[1] * v * fabs(r_ref) + Xv_bar[2] * r_ref * fabs(v) + Xv_bar[3] * r_ref * fabs(r_ref)
                + Xv_bar[4] * u_ref * v  + Xv_bar[5] * u_ref * r_ref + Xv_bar[6] * v + Xv_bar[7] * r_ref)

    # Modelo dinámico final
    f_expl = x_dot  # Aquí usas la dinámica que definimos
    f_impl = vertcat( xdot - f_expl, output - z_exp)

    # algebraic variables
    z = output

    # parameters
    p = vertcat(
        Xv_bar,
        Eps_,
        coef)  # Concatenamos los parámetros constantes y los disturbios
     

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
    model.x0 = np.array([0, 0, 0, 0, 0])

    return model, constraint