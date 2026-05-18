from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import numpy as np
import scipy.linalg
from casadi import *
from iblos_pf_model import pf_model


def export_asv_pf_ocp(): 
    # Configurar el horizonte de predicción (en pasos discretos)
    N = 30  # Número de pasos de discretización
    Ts = 0.2  # Tiempo de muestreo
    Tf = N * Ts

    # create render arguments
    ocp = AcadosOcp()

    # export model
    model, constraint = pf_model()
    
    # define acados ODE
    model_ac = AcadosModel()
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
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
    nz = model.z.rows()
    ny = nx + nu + nz
    ny_e = nx

    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = N * Ts  

    Q = np.diag([1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0])       # penalización estados
    R = np.diag([0.0, 0.0, 0.0])               # penalización control
    Qe = np.diag([1.0, 1.0, 0.0, 0.0, 0.0])

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ocp.cost.W = scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Qe 

    Vx = np.zeros((ny, nx))
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    ocp.cost.Vu = Vu

    Vz = np.zeros((ny, nz))
    ocp.cost.Vz = Vz

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e

    # set initial references
    ocp.cost.yref = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    ocp.cost.yref_e = np.array([0, 0, 0, 0, 0])

    ocp.parameter_values = np.zeros((model_ac.p.shape[0],))
    
    # set initial condition
    ocp.constraints.x0 = model.x0

    # setting constraints
    ocp.constraints.lbx = np.array([])  # Lower bounds vacío
    ocp.constraints.ubx = np.array([])  # Upper bounds vacío
    ocp.constraints.idxbx = np.array([])

    ocp.constraints.lbx_e = np.array([])  # Lower bounds vacío
    ocp.constraints.ubx_e = np.array([])  # Upper bounds vacío
    ocp.constraints.idxbx_e = np.array([])
    
    ocp.constraints.lbu = np.array([])  # Lower bounds vacío
    ocp.constraints.ubu = np.array([])  # Upper bounds vacío
    ocp.constraints.idxbu = np.array([])

    # define constraint
    model_ac.con_h_expr = MX([])  # Expresión vacía de CasADi
    ocp.constraints.lh = np.array([])  # Lower bounds vacío
    ocp.constraints.uh = np.array([])  # Upper bounds vacío

    
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "IRK"
    ocp.solver_options.sim_method_num_stages = 3  # Método de Kutta orden 3
    ocp.solver_options.sim_method_num_steps = 2   # Balance entre precisión y costo
    ocp.solver_options.sim_method_jac_reuse = 2   # Reutilizar Jacobiano por más pasos
    
    ocp.solver_options.qp_solver_cond_N = N
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = Tf

    return ocp


