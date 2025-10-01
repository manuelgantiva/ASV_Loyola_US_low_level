from casadi import *

def asv_model():
   
    Xu = MX.sym('Xu', 6)
    Xv = MX.sym('Xv', 12)
    Xr = MX.sym('Xr', 12)

    Dz_up = MX.sym('Dz_up', 1)
    Dz_down = MX.sym('Dz_down', 1)
    
    u_min_ = 0.0
    v_min_ = -1.5
    r_min_ = -1.0
    Delta_mean_min_ = -0.5
    Delta_diff_min_ = -0.5

    u_max_ = 3.0
    v_max_ = 1.5
    r_max_ = 1.0
    Delta_mean_max_ = 0.5
    Delta_diff_max_ = 0.5

    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "ASV_model"

    # Variables de estado (u, v, r, psi)
    u = MX.sym('u')
    v = MX.sym('v')
    r = MX.sym('r')
    psi = MX.sym('psi')
    mean = MX.sym('mean')
    diff = MX.sym('diff')
    states = vertcat(u, v, r, psi, mean, diff)
    
    # Xdot (u, v, r, psi)
    # udot = MX.sym('udot')
    # vdot = MX.sym('vdot')
    # rdot = MX.sym('rdot')
    # psidot = MX.sym('psidot')
    # meandot = MX.sym('meandot')
    # diffdot = MX.sym('diffdot')
    # statesdot = vertcat(udot, vdot, rdot, psidot, meandot, diffdot)

    # Variables de control (d_mean, d_diff)
    d_mean = MX.sym('d_mean')
    d_diff = MX.sym('d_diff')
    controls = vertcat(d_mean, d_diff)

    # Variables de perturbación (se_u, se_v, se_r)
    se_u = MX.sym('se_u')
    se_v = MX.sym('se_v')
    se_r = MX.sym('se_r')
    disturbances = vertcat(se_u, se_v, se_r)

    # Número de estados, controles y entradas
    n_states = states.size(1) 
    n_controls = controls.size(1)
    n_inputs = disturbances.size(1)

    # Definir la dinámica
    sig = if_else(diff >= 0, 1, -1)

    beta = if_else(mean >= fabs(diff), 1.0, 0.0)
    sum_1 = (mean**2 + (diff**2 / 4.0))

    IG = MX.zeros(n_states)
    IG[0] = Xu[4]*sum_1 + Xu[5]*mean
    IG[1] = Xv[8]*sum_1*(1-beta)*sig + Xv[9]*mean*diff + Xv[10]*mean*(1-beta)*sig + Xv[11]*diff/2.0
    IG[2] = Xr[8]*sum_1*(1-beta)*sig + Xr[9]*mean*diff + Xr[10]*mean*(1-beta)*sig + Xr[11]*diff/2.0
    IG[3] = r
    IG[4] = d_mean
    IG[5] = d_diff

    sigma = MX.zeros(n_states)
    sigma[0] = (Xu[0]*u*fabs(u) + Xu[1]*v*r + Xu[2]*r**2 + Xu[3]*u) + se_u
    sigma[1] = (Xv[0]*v*fabs(v) + Xv[1]*v*fabs(r) + Xv[2]*r*fabs(v) + Xv[3]*r*fabs(r) + Xv[4]*u*v + Xv[5]*u*r + Xv[6]*v + Xv[7]*r) + se_v
    sigma[2] = (Xr[0]*v*fabs(v) + Xr[1]*v*fabs(r) + Xr[2]*r*fabs(v) + Xr[3]*r*fabs(r) + Xr[4]*u*v + Xr[5]*u*r + Xr[6]*v + Xr[7]*r) + se_r

    # Modelo dinámico final
    f_expl = IG + sigma  # Aquí usas la dinámica que definimos

    # Variables de estado y control
    model.x = states  # Vectores de estados
    model.u = controls  # Vectores de control

    # algebraic variables
    z = vertcat([])

    # parameters
    p = vertcat(
        Xu,
        Xv,
        Xr,
        Dz_up,
        Dz_down, 
        se_u,
        se_v,
        se_r)  # Concatenamos los parámetros constantes y los disturbios

    # constraint on forces

    eq1 = mean - ((-1+Dz_up)/(2-Dz_up+Dz_down))*(diff)
    eq2 = mean - ((1+Dz_down)/(-2+Dz_up-Dz_down))*(diff)
    eq3 = mean - ((-1+Dz_up)/(-2+Dz_up-Dz_down))*(diff)
    eq4 = mean - ((1+Dz_down)/(2-Dz_up+Dz_down))*(diff)
    eq5 = mean + fabs(diff) / 2

    constraint.expr = vertcat(eq1, eq2, eq3, eq4, eq5)

    constraint.eq1_min = -1e9  # “-inf”
    constraint.eq2_min = -0.999 # -1.0 - Dz_down
    constraint.eq3_min = -1e9  # “-inf”
    constraint.eq4_min = -0.999 # -1.0 - Dz_down
    constraint.eq5_min = 0.0

    constraint.eq1_max = 0.999 #1.0 - Dz_up
    constraint.eq2_max = 1e9  # “inf”
    constraint.eq3_max = 0.999 #1.0 - Dz_up
    constraint.eq4_max = 1e9   # “inf”
    constraint.eq5_max = 1e9   # “inf”

    constraint.u_min = u_min_
    constraint.v_min = v_min_
    constraint.r_min = r_min_
    constraint.Delta_mean_min = Delta_mean_min_
    constraint.Delta_diff_min = Delta_diff_min_

    constraint.u_max = u_max_
    constraint.v_max = v_max_
    constraint.r_max = r_max_
    constraint.Delta_mean_max = Delta_mean_max_
    constraint.Delta_diff_max = Delta_diff_max_

    # Define model struct
    # model.f_impl_expr = statesdot - f_expl
    model.f_expl_expr = f_expl
    model.x = states
    # model.xdot = statesdot
    model.u = controls
    model.z = z
    model.p = p
    model.name = model_name

    # Define initial conditions
    model.x0 = np.array([0, 0, 0, 0, 0, 0])

    return model, constraint