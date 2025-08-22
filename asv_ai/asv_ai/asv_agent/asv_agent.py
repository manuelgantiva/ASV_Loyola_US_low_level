import numpy as np
import gymnasium as gym

# Surge o avance
# Sway o desplazamiento lateral 
# Yaw o giro del angulo de la proa

class ASVAgent():
    """
    ASV According to state
    X = [x_est, y_est, psi_est, u_est, v_est, r_est]
    """
    mass = 23.8 # Peso del barco en kg
    
    rotationalInertia = 1.76 # Momento de inercia del barco en kg*m^2 (yaw) 
    
    xG = 0.046 # Centro de gravedad ligeramente desplazado

    X_udot =  -2 # Resistance of water to linear acceleration in the surge direction
    Y_vdot = -10 # Same for sway 
    Y_rdot = 0 # Added mass coupling between yaw and sway. Indicates how yaw motion affects the sway hydrodynamic forces.
    N_vdot = 0 # Represents how sway motion affects the yaw hydrodynamic forces.
    N_rdot = -1 # Resistance of water to rotational acceleration in yaw.
    
    x_u = -0.72253 # Linear drag coefficient in surge
    x_u_abs_u = -1.32742 # Nonlinear drag forces due to water resistance, proportional to the square of velocity.
    y_v = -0.88965
    y_v_abs_v = -36.47287
    y_v_abs_r = -0.805
    y_r = -7.25
    y_r_abs_v = -0.845
    y_r_abs_r = -3.45
    n_v = 0.0313
    n_v_abs_v = 3.95645
    n_v_abs_r = 0.13
    n_r = -1.9
    n_r_abs_v = 0.08
    n_r_abs_r = -0.75
    c = np.zeros((3,3), dtype=np.float32)

    # Pesos de importancia al error
    k_v=2.75  #1 si aumenta, es mas importante que la dirección al punto deseado sea la correcta
    k_d=2  # si aumenta, es más importante que la distancia al punto deseado sea la correcta


    # TODO en ros tendremos que hacer una funcion auxiliar de inicizalizacion de posicion porque no vamos a poder elegirla
    def __init__(self, id, x_ini=None, dt = 0.05, lm=5):
        self.x = x_ini if x_ini is not None else np.array([[10], [5], [np.pi/4], [0], [0], [0]], dtype=np.float32)
        self.dt = dt
        self.id = id # Identificador del ASV (no se puede definir, es el que es)
        self.lm = lm # En el paper es l, longitud de la formación con respecto al cenrtoide de la formaicon
        self.beta_m=None # En el paper es beta, angulo de la formación con respecto al eje x (Si es 0 debe estar justo enfrente del centroide) Norte a este equivale a rotacion positiva

        # Mass or effective intertia matrix
        # Represents the ASV's resistance to linear and rotational accelerations 
        # in surge (x-direction), sway (y-direction), and yaw (rotation about z-axis).
        self.m = np.array([[self.mass - self.X_udot, 0, 0],
                           [0, self.mass - self.Y_vdot, self.mass*self.xG - self.Y_rdot],
                           [0, self.mass*self.xG - self.N_vdot, self.rotationalInertia - self.N_rdot]])
        self.invM = np.linalg.inv(self.m)
        # self.x_u =  0
        # self.x_u_abs_u =  0
        # self.y_v =  0
        # self.y_v_abs_v =  0
        # self.y_v_abs_r =  0
        # self.y_r =  0
        # self.y_r_abs_v =  0
        # self.y_r_abs_r =  0
        # self.n_v =  0
        # self.n_v_abs_v =  0
        # self.n_v_abs_r =  0
        # self.n_r = 0
        # self.n_r_abs_v =  0
        # self.n_r_abs_r =  0


    # Devuelve la observación del ASV (posición y velocidad)
    # Se puede agregar ruido a la observación y se puede especificar la varianza del ruido
    def observe(self, add_noise=False, sigma2=0.01):
        return self.x.T[0]
    
    @staticmethod
    # Traduce la acción del agente a fuerza y momento de rotación para el ASV
    def get_force_tau(action):
        '''                            intervalos de acción
        action 0 es la fuerza de surge [0, 2] (1 como minimo ¿por qué?)
        action 1 es el momento de rotación [-0.5, 0.5]
        '''
        return np.vstack([action[0] + 1, 0, action[1]/2.3])
    
    # Simular el movimiento del ASV. 
    # Calcula cómo el estado del ASV (posición, orientación y velocidades)
    # cambia bajo la acción de fuerzas y momentos proporcionados por action.
    # Este es un simulador de la dinámica del ASV. Para ls deberes a ros solo vamos a hacer que evolve envie datos de accion. 
    def evolve(self, action): #specific for that boat
        
        for _ in range(4):
            _, _, psi, u_r, v_r, r = self.x.flat
            ROT = np.array([[np.cos(psi), -np.sin(psi), 0],
                        [np.sin(psi), np.cos(psi), 0],
                        [0, 0, 1]])

            c13 = -self.m[1, 1] * v_r - self.m[1, 2]*r
            c23 = self.m[0, 0] * u_r
            self.c[0, 2] = c13
            self.c[1, 2] = c23
            self.c[2, 0] = -c13
            self.c[2, 1] = -c23

            abs_nu = np.abs(self.x[3:].flat)
            D = np.array([
                [-self.x_u - self.x_u_abs_u *abs_nu[0], 0, 0 ],
                [0, -self.y_v - self.y_v_abs_v *abs_nu[1] - self.y_v_abs_r*abs_nu[2], - self.y_r - self.y_r_abs_v*abs_nu[1] - self.y_r_abs_r*abs_nu[2]],
                [0, -self.n_v - self.n_v_abs_v *abs_nu[1] - self.n_v_abs_r*abs_nu[2], - self.n_r - self.n_r_abs_v*abs_nu[1] - self.n_r_abs_r*abs_nu[2]]
                ])
            eta_dot = np.matmul(ROT, self.x[3:])# variation of position
            nu_dot = np.matmul(self.invM, self.get_force_tau(action) - np.matmul(self.c+D, self.x[3:]))# speed variation
            
            # Update the state of the ASV with new position and velocity after the action has taken place
            xdot = np.vstack([eta_dot, nu_dot])
            self.x += xdot*self.dt

            # -180 180 (Normalizacion del angulo)
            self.x[2] = (self.x[2] + np.pi) % (2 * np.pi) - np.pi
    
    # Distancia entre el ASV y un punto dado
    def distanceTo(self, point):
        return np.linalg.norm(np.subtract(self.x[:2], point))
    
    # Calcula la posición esperada de un vehículo en formación 
    # (o de referencia) en base a un punto líder y un ángulo de la línea de formación
    def expected_position(self, x_v,  slope):
        '''
        x_v: [x, y]
        slope: angle of the line
        '''
        #cambiamos l-->lm, beta-->beta_m
        return x_v + self.lm*np.array([np.cos(slope + self.beta_m), np.sin(slope + self.beta_m)])
    #formation error

    # Calcula el error de formación entre el ASV y un punto de referencia (posición esperada)
    def error_f(self, x_v, slope):
        '''
        x_v: [x, y]
        slope: angle of the line
        x_t: [x, y]
        '''
        return np.linalg.norm(self.expected_position(x_v, slope) - self.x[:2].flat)

#verify extistence 
    # Se remplaza la sgte funcion
    #def Rv(self):
    #    return self.k_v*(self.x[4]*np.cos(self.beta) - self.x[4]*np.sin(self.beta))
    # Ahora la funcion tambien depende de x_v y slope, por lo que hará falta agregar estos datos argumentos
    # Recompensa por velocidad (no de posicion)
    def Rv(self, x_v, slope):
        x_p1 = self.expected_position(x_v, slope) - self.x[:2]
        angle = np.arctan2(x_p1[1], x_p1[0]) - self.x[2]
        return self.k_v*(self.x[3]*np.cos(angle) - (np.abs(self.x[4]) + np.abs(self.x[5]))*np.abs(np.sin(angle)))  # 
                              # x[3] (surge) grande y cos pequeño (mejor). x[4] (sway) y x[5] (yaw) pequeños y sen grande (mejor)
    
    # Recompensa por distancia (posicion)
    def Rd(self,x_v, slope):
        err_max=10
        return self.k_d*-self.error_f(x_v,slope)/err_max
        # negativo para restarle recompesa si es muy grande

# Recomepensa y penalización

