from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import numpy as np
import scipy.linalg
from casadi import *
from asv_model import asv_model


def export_asv_ocp(): 
    # Configurar el horizonte de predicción (en pasos discretos)
    N = 20  # Número de pasos de discretización
    Ts = 0.1  # Tiempo de muestreo
    Tf = N * Ts

    # create render arguments
    ocp = AcadosOcp()

    # export model
    model, constraint = asv_model()
    
    # define acados ODE
    model_ac = AcadosModel()
    # model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.disc_dyn_expr = model.x + Ts * model.f_expl_expr
    model_ac.x = model.x
    # model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.z = model.z
    model_ac.p = model.p
    model_ac.name = model.name
    ocp.model = model_ac

    # Establecer el horizonte total
    ocp.dims.N = N  # Número de pasos de predicción

    # set dimensions
    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx

    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = N * Ts  

    Q = np.diag([20.0, 0.1, 30.0, 60.0, 0.0, 5.0])       # penalización estados
    R = np.diag([0.0, 0.0])               # penalización control
    Qe = np.diag([20.0, 0.1, 30.0, 60.0, 0.0, 5.0])

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ocp.cost.W = scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Qe 

    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[6, 0] = 1.0
    Vu[7, 1] = 1.0
    ocp.cost.Vu = Vu

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e

    # set initial references
    ocp.cost.yref = np.array([0.5, 0, 0, 0, 0, 0, 0, 0])
    ocp.cost.yref_e = np.array([0.5, 0, 0, 0, 0, 0])

    ocp.parameter_values = np.zeros((model_ac.p.shape[0],))

    # setting constraints
    ocp.constraints.lbx = np.array([constraint.u_min, constraint.v_min, constraint.r_min])
    ocp.constraints.ubx = np.array([constraint.u_max, constraint.v_max, constraint.r_max])
    ocp.constraints.idxbx = np.array([0,1,2])

    ocp.constraints.lbx_e = np.array([constraint.u_min, constraint.v_min, constraint.r_min])
    ocp.constraints.ubx_e = np.array([constraint.u_max, constraint.v_max, constraint.r_max])
    ocp.constraints.idxbx_e = np.array([0,1,2])

    ocp.constraints.lbu = np.array([constraint.Delta_mean_min/Ts, constraint.Delta_diff_min/Ts])
    ocp.constraints.ubu = np.array([constraint.Delta_mean_max/Ts, constraint.Delta_diff_max/Ts])
    ocp.constraints.idxbu = np.array([0, 1])

    # define constraint
    model_ac.con_h_expr = constraint.expr

    ocp.constraints.lh = np.array(
        [
            constraint.eq1_min,
            constraint.eq2_min,
            constraint.eq3_min,
            constraint.eq4_min,
            constraint.eq5_min,
        ]
    )
    ocp.constraints.uh = np.array(
        [
            constraint.eq1_max,
            constraint.eq2_max,
            constraint.eq3_max,
            constraint.eq4_max,
            constraint.eq5_max,
        ]
    )

    # set initial condition
    ocp.constraints.x0 = model.x0

    # set QP solver and integration
    ocp.solver_options.tf = Tf
    # ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 1
    ocp.solver_options.sim_method_num_steps = 1

    ocp.solver_options.qp_solver_cond_N = N
    ocp.solver_options.N_horizon = N
    
    #ocp.solver_options.qp_solver_iter_max = 20
    #ocp.solver_options.qp_solver_tol_stat = 1e-2
    #ocp.solver_options.qp_solver_tol_eq = 1e-2
    #ocp.solver_options.qp_solver_tol_ineq = 1e-2
    #ocp.solver_options.qp_solver_tol_comp = 1e-2
    # ocp.solver_options.qp_solver_warm_start = 1
    # ocp.solver_options.nlp_solver_warm_start_first_qp = True

    # ocp.solver_options('print_level', 2)

    return ocp

    # Generar el solver de ACADOS
    # AcadosOcpSolver(ocp, json_file="acados_ocp.json", generate=True, build=True)

