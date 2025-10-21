import cv2
import gymnasium as gym
import numpy as np
from gymnasium import spaces

from ..asv_agent.asv_agent import ASVAgent
from ..asv_path.asv_path import ParametrizedPath


class Environment(gym.Env):
    def __init__(self, num_agents=2):
        super(Environment, self).__init__()
        self.num_agents = num_agents

        # Action: [vyaw_rate, forward_velocity_change] for each agent
        self.action_space = spaces.Box(low=-1, high=1, shape=(self.num_agents, 2), dtype=np.float32)
        # Observation: [x, y, yaw, vx, vy, vyaw] for each agent
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.num_agents, 6), dtype=np.float32)

        self.state = np.zeros((self.num_agents, 6))
        self.dt = 0.1  # Simulation time step

        # Add formation control components
        self.param_path = ParametrizedPath()
        self.formation_distance = 3.0
        self.agent_betas = [2 * np.pi * i / self.num_agents for i in range(self.num_agents)]

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Initialize path parameter
        self.param_path.theta = np.random.uniform(5, 15)

        for i in range(self.num_agents):
            start_x = i * 4.0 - (self.num_agents - 1) * 2.0  # Smaller spacing
            self.state[i] = [start_x, 0.0, np.pi / 2, 1.0, 0.0, 0.0]
        return self._get_obs(), self._get_info()

    def step(self, action):
        # Reshape action from flat array to per-agent actions
        action = action.reshape((self.num_agents, 2))

        for i in range(self.num_agents):
            # Apply actions: action[0] controls turning, action[1] controls speed
            self.state[i][5] = action[i][0] * 0.5  # vyaw (turning rate)
            self.state[i][3] += action[i][1] * 0.1 # vx (forward speed)
            self.state[i][3] = np.clip(self.state[i][3], 0.0, 5.0) # Clamp speed

            # Update physics
            x, y, yaw, vx, vy, vyaw = self.state[i]
            new_yaw = yaw + vyaw * self.dt
            new_x = x + (vx * np.cos(new_yaw)) * self.dt
            new_y = y + (vx * np.sin(new_yaw)) * self.dt
            self.state[i] = [new_x, new_y, new_yaw, vx, vy, vyaw]

        # Calculate reward and done state
        reward = self._calculate_reward()
        done = self._check_done()

        return self._get_obs(), reward, done, False, self._get_info()

    def _calculate_reward(self):
        """
        Calculate formation-following reward based on velocity direction and formation error.
        This matches the sophisticated reward system from Environment2 and asv_env_node.py
        """
        # Get virtual leader position and path derivative
        pos_v, deriv = self.param_path.path(self.param_path.theta, True)

        # Calculate expected positions for each agent
        cos_angles = np.cos(deriv.item() + np.array(self.agent_betas))
        sin_angles = np.sin(deriv.item() + np.array(self.agent_betas))
        expected_positions = pos_v.flatten()[np.newaxis, :] + self.formation_distance * np.column_stack([cos_angles, sin_angles])

        # Get current agent positions and orientations
        agent_positions = self.state[:, :2]  # [x, y] for each agent
        agent_orientations = self.state[:, 2]  # yaw for each agent
        agent_velocities = self.state[:, 3:6]  # [vx, vy, vyaw] for each agent

        # Calculate position errors and angles to target
        position_errors = expected_positions - agent_positions
        angles_to_target = np.arctan2(position_errors[:, 1], position_errors[:, 0]) - agent_orientations

        # Velocity rewards (Rv components) - rewards moving toward formation goal
        k_v = 2.75
        rv_components = k_v * (agent_velocities[:, 0] * np.cos(angles_to_target) -
                              (np.abs(agent_velocities[:, 1]) + np.abs(agent_velocities[:, 2])) *
                              np.abs(np.sin(angles_to_target)))

        # Distance rewards (Rd components) - penalizes formation error
        k_d = 2.0
        err_max = 10.0
        errors = np.linalg.norm(position_errors, axis=1)
        rd_components = k_d * (-errors / err_max)

        # Average rewards
        total_rv = np.mean(rv_components)
        total_rd = np.mean(rd_components)

        # Final reward (same as Environment2)
        reward = total_rv + total_rd

        # Penalty for collision (keep existing collision logic)
        if self._check_collision():
            reward -= 100

        return reward

    def _check_done(self):
        """
        Episode is done if:
        1. Boats collide (failure condition)
        2. Formation is achieved with low along-track error (success condition)
        """
        # Check collision (failure)
        if self._check_collision():
            return True

        # Check formation success (like Environment2)
        try:
            centroid = np.mean(self.state[:, :2], axis=0)
            ate = self.param_path.along_track_error(centroid)

            # Episode succeeds when along-track error is small
            if ate < 5.0:
                return True
        except:
            # If calculation fails, don't end episode
            pass

        return False

    def _check_collision(self):
        if self.num_agents < 2:
            return False
        pos1 = self.state[0, :2]
        pos2 = self.state[1, :2]
        distance = np.linalg.norm(pos1 - pos2)
        return distance < 5.0 # Collision if closer than 5 meters

    def get_state(self):
        return self.state

    def _get_obs(self):
        return self.state.flatten()

    def _get_info(self):
        return {}

############################## ENVIRONMENT ORIGINAL ##############################

    metadata = {"render_modes": ["human"], "render_fps": 30}
class EnvOriginal(gym.Env):
    # Configurar el entorno, incluyendo agentes (ASVs), el espacio de acciones y el espacio de observaciones
    def __init__(self, render_mode=None):
        super().__init__()
        #Agents + range
        self.n=2
        #inicializar clase parametrized path
        self.param_p = ParametrizedPath()
        self.current_step=0 #supervisar cuántos steps se han dado (contador)

        #Crear instancias de ASVAgent sin el argumento 'id' y con estado inicial aleatorio
        self.agents = [ASVAgent(id=n) for n in range (self.n)]
        self.beta_m()

        #Esta fórmula garantiza que cada vehículo se posicione en un ángulo
        #uniformemente distribuido alrededor del líder virtual, formando así
        #una formación circular o en función de la geometría deseada ajustando la fórmula de
        #beta_m = 2 * np.pi * i / self.n


        # 2 actionS might take place ai=(Tau ui, Tau vi)
        # Normalized for 3 USVs

        # Limite inferior y superior de las acciones surge y rotation (yaw) * numero de ASV (para 2 ASV [-1,-1,-1,-1] - [1,1,1,1])
        # Espacio de trabajo de empuje (eje x) y rotacion (eje y)
        self.action_space = spaces.Box(
            low=np.array([-1, -1]*self.n),
            high=np.array([1, 1, ]*self.n), dtype=np.float32)

        # posicion (dos primeros) orientacion (tercera), velocidad (cuarta y quinta) y velocidad angular (sexta) de los ASVs. Posicion deseada (dos ultimos)
        obs_space_l = np.array(
        [-15, -15, -np.pi, -3.6, -3.6, -np.pi/3, -15, -15]*self.n, dtype=np.float32)

        # Extiende obs_space_l para incluir los límites inferiores de las observaciones relacionadas con el líder virtual:
        obs_space_l = np.hstack(
            [obs_space_l, np.array(
                [-15, -15, 0, 0], dtype=np.float32)])#lider virtual pos (dos primeros) y error froma (tercero) y cross track error (cuarto) (minimos valores)

        # Límites superiores de las observaciones (posición y velocidad de los ASVs, posición y derivada de la trayectoria, error de formación y error de seguimiento)
        obs_space_h = np.array(
        [35, 35, np.pi, 3.6, 3.6, np.pi/3, 35, 35]*self.n, dtype=np.float32)

        obs_space_h = np.hstack(
            [obs_space_h, np.array(
                [35, 35, 100, 100], dtype=np.float32)]
        )

        # Example for using image as input (channel-first; channel-last also works):
        self.observation_space = spaces.Box(low=obs_space_l, high=obs_space_h, dtype=np.float32)

        # Render path
        self.render_mode = render_mode
        if self.render_mode == "human":
            thethas = np.linspace(0, 100, 2)
            painted_path = np.array([self.param_p.path(thetha) for thetha in thethas])
            self.start_path = np.array([painted_path[0][0], painted_path[0][1]])
            self.end_path = np.array([painted_path[-1][ 0], painted_path[-1][1]])

    # Asigna un angulo relativo para cada ASV
    def beta_m(self):
        for i, agent in enumerate(self.agents):
            agent.beta_m = 2 * np.pi * i / self.n

    # Devuelve el estado actual del ASV (observacion)
    def _get_obs(self, with_v=False):

        # Posición y velocidad del lider virtual
        pos_v, deriv = self.param_p.path(self.param_p.theta, True)

        # Inicializacion del vector observaciones y error de formación
        state = np.array([])
        error_f = 0

        # Almacena las observaciones y calcula la posicion esperada de cada ASV
        for agent in self.agents:
            if state.size:
                state = np.vstack(
                    [state, agent.x, agent.expected_position(pos_v, deriv)])
            else:
                state = np.vstack(
                    [agent.x, agent.expected_position(pos_v, deriv)]) # agent.x incluye posicion, orientacion y velocidad. expected_position incluye la posicion esperada de cada agente

            # Calcula el error de formación individual de cada ASV
            error_f += agent.error_f(pos_v, deriv)

        # Almacena la posicion del lider virutal, el error de formación promedio y cross-track error promedio (error perpendicular de los agentes respecto a la trayectoria)
        state = np.vstack([state, pos_v[0], pos_v[1], error_f/self.n,
                           self.param_p.cross_track_error(np.mean(
                               np.stack([agent.x[:2] for agent in self.agents]), axis=0))],
                                 dtype=np.float32).reshape(-1)
        if with_v:
            return state, pos_v, deriv
        return state

    # Interaccion entre entorno y ASV en un paso de tiempo
    def step(self, action): #aquí sucede la interacción
        #por cada gente fuerza y giro, observar
        self.current_step += 1 # en cada step estamos contando
        #actions = [Fi, Ti] * self.n
        #Slicing
        # Aplica la accion de cada ASV
        for i, agent in enumerate(self.agents):
            agent.evolve(action[i*2:i*2+2])
        observation, x_v, deriv = self._get_obs(with_v=True)
        #Imprime reward y el tipo de dato

        #recompenso mediante rv y rd
        # Calcula la metrica de direccion de la velocidad de cada ASV respecto a la trayectoria desdeada
        # Promedio que se usa en parte de la recompensa
        rv = [agent.Rv(x_v,deriv) for agent in self.agents] #deriva
        rv = sum(rv)/len(rv)
        # Desviacion de la posicion de cada ASV respecto a la posicion deseada
        rd = [agent.Rd(x_v, deriv) for agent in self.agents] #pos deseada (formation error)
        rd = sum(rd)/len(rd)
        #extrayendo el primer elemento de rv(que es un array) convirtiendolo en consecuencia en un escalar
        rv = rv.item() if isinstance(rv, np.ndarray) and rv.shape == (1,) else rv

        # Recompensa total
        reward = rv + rd

        # Calcula el error de formación promedio de los ASVs basado en la posicion promedio de los ASVs
        ate = self.param_p.along_track_error(np.mean(np.stack([agent.x[:2] for agent in self.agents]), axis=0))
        terminated = False

        # Termina el episodio si el error de formación es menor a 5 y suma 1 a la recompensa
        if (ate < 5):
        #     # self.param_p.theta += 1
            reward += 1
            terminated = True

        # self.param_p.update_theta(np.mean(np.stack([agent.x[:2] for agent in self.agents]), axis=0), 10)

        # Devuelve el estado actual del entorno despues del paso, la recompensa para el paso, si el episodio termino y si el paso fue truncado
        return observation, reward, terminated, False, {}


    # Reinicia el entorno y los agentes devolviendo los agentes a una posicion aleatoria
    def reset(self, seed=None, options=None):

        # for agent in self.agents:
        #     # randomize agents position
        #     agent.x = np.array([[np.random.uniform(0, 75)], [np.random.uniform(0, 75)], [
        #                        np.random.uniform(-np.pi, np.pi)], [0], [0], [0]], dtype=np.float32)
        # self.current_step = 0 #cuando reiniciamos la simulación, decimos que no ocurrió ningún paso del step (reinicio contador)

        # Reinicia el parametro que controla la posicion del lider virtual y la trayectoria
        self.param_p.theta = np.random.uniform(5, 15)  # Near origin

        # Reinicia la posicion de los ASVs
        for i, agent in enumerate(self.agents):
            agent.x = np.array([[(2 - -1^(i+1))*5],
                                [(2 + -1^(i+1))*5],
                                [np.random.rand()*3],[0], [0], [0]], dtype=np.float32)

            #
            # 14 sin rand
            factor = 0.4 # El 60% de las veces se reinicia la posicion de los ASVs
            if np.random.rand() > factor:
                pos_v, deriv = self.param_p.path(self.param_p.theta, True)

                agent.x[0] =  agent.expected_position(pos_v, deriv)[0] - 5 # Small offset toward origin
                agent.x[1] =  agent.expected_position(pos_v, deriv)[1] - 5

                x_p1 = agent.expected_position(pos_v, deriv) - agent.x[:2]
                angle = np.arctan2(x_p1[1], x_p1[0]) # Angulo entre la posicion esperada y la posicion actual
                agent.x[2] = angle
                agent.x[3] = np.random.rand() # Velocidad lineal aleatoria dentro del rango

        self.current_step = 0 #en cada step estamos contando (reinicio contador)

        return self._get_obs(), {}


    def render(self, mode="human"):
        # NED framework (inicializacion del entorno)
        self.canvas = np.ones((240, 320, 3), np.uint8) # Crea un lienzo de 240x320 pixeles con 3 canales de color
        RED = (0, 0, 255) # Color rojo para lineas o poligonos
        x_pad = 240/2 # Posicion en x e y del centro del lienzo
        y_pad = 160/2

        # Dibuja la trayectoria deseada
        cv2.line(self.canvas, (int(self.start_path[1][0] + x_pad), int(self.start_path[0][0] + y_pad)),
                    (int(self.end_path[1][0] + x_pad), int(self.end_path[0][0] + y_pad)), RED, 1)

        # Posicion del lider virtual, centro de la formacion y proyeccion de la formacion
        pos_v, slope = self.param_p.path(self.param_p.theta, True)
        pos_c = np.mean(np.stack([agent.x[:2] for agent in self.agents]), axis=0)
        pos_p = self.param_p.projection(pos_c)

        # draw these points (dibuja cada agente como un poligono rojo y un circulo amarillo en la posicion esperada de cada agente)
        for asv in self.agents:
            asv_center = (asv.x[1][0] + x_pad, asv.x[0][0] + y_pad)
            points = np.array([asv_center,
                               (asv.x[1][0] + x_pad + 4*np.cos(asv.x[2][0]),
                                asv.x[0][0] + y_pad - 4*np.sin(asv.x[2][0])),
                                (asv.x[1][0] + x_pad + 10*np.sin(asv.x[2][0]),
                                  asv.x[0][0] + y_pad + 10*np.cos(asv.x[2][0])),
                                (asv.x[1][0] + x_pad - 4*np.cos(asv.x[2][0]),
                                asv.x[0][0] + y_pad + 4*np.sin(asv.x[2][0])),], dtype=np.int32)

            pos_a_v = asv.expected_position(pos_v, slope)
            cv2.fillPoly(self.canvas, np.int32([points]), RED)
            cv2.circle(self.canvas, (int(pos_a_v[1][0] + x_pad), int(pos_a_v[0][0] + y_pad)),
                       5, (255, 255, 0), -1)

        # Dibuja el lider virtual (circulo verde), el centro promedio de los ASV (circulo amarillo) y la proyeccion del centro promedio (circulo magenta)
        cv2.circle(
            self.canvas, (int(pos_v[1][0] + x_pad), int(pos_v[0][0] + y_pad)), 5, (0, 255, 0), -1)
        cv2.circle(
            self.canvas, (int(pos_c[1][0] + x_pad), int(pos_c[0][0] + y_pad)), 3, (0, 255, 255), -1)
        cv2.circle(
            self.canvas, (int(pos_p[1][0] + x_pad), int(pos_p[0][0] + y_pad)), 5, (255, 0, 255), -1)


        _, x_v, deriv = self._get_obs(with_v=True)
        #Imprime reward y el tipo de dato

        #recompenso mediante rv y rd
        rv = [agent.Rv(x_v,deriv) for agent in self.agents] #deriva (recompensa de velocidad)
        rv = sum(rv)/len(rv) # promedio de la recompensa de velocidad
        rd = [agent.Rd(x_v, deriv) for agent in self.agents] #pos deseada (formation error) (penalizacion basada en el error de formacion)
        rd = sum(rd)/len(rd)
        reward = rv + rd
        # flip up-down
        self.canvas = cv2.flip(self.canvas, 0) # invierte la imagen para que el origen sea en la esquina inferior izquierda
        # zoom in
        self.canvas = cv2.resize(self.canvas, (640, 480), interpolation=cv2.INTER_AREA) # amplia la imagen a 640x480 pixeles

        # Imprime la recompensa y el error de formación en la ventana de visualización
        with np.printoptions(precision=3, suppress=True):
            cv2.putText(self.canvas, f"reward: {rv[0]:0.2f} + {rd:0.2f} =  {reward}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            ate = self.param_p.along_track_error(np.mean(np.stack([agent.x[:2] for agent in self.agents]), axis=0))
            cv2.putText(self.canvas, f"ate: {ate:0.2f}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)


        # if human imprime la imagen en una ventana
        if mode == "human":
            cv2.imshow("Game", self.canvas)
            cv2.waitKey(1)

        # si el modo es rgb_array, devuelve la imagen a color RGB
        elif mode == "rgb_array":
            return self.canvas



# # if __name__ == "__main__":
# #     # cheking env
# #     from stable_baselines3 import PPO

# #     # Create the environment
# #     env = Environment(render_mode="human")

# #     # Create the model PPO
# #     model = PPO("MlpPolicy", env, verbose=1) # Multilayer perceptron policy PPO training model
# #     env.reset() # Reinicia el entorno

# #     # Run the model (inicializa el bucle principal mienrtas running = true)
# #     running = True
# #     observation, _ = env.reset()

# #     # Toma decisiones. Utiliza la obs actual para predecir el siguiente paso
# #     while running:
# #         action, _ = model.predict(observation)

# #         # Ejecuta un paso en el entorno (observation = nuevo estado del entorno, reward = recompensa obtenida, terminated = si el episodio termino, truncated = si el paso fue truncado)
# #         observation, reward, terminated, truncated, info = env.step([0,0,0,0])
# #         # print(observation)

# #         env.render("human")
# #         if (terminated or truncated or env.current_step > 2048):
# #             observation, _ = env.reset()
# #             # running = False


class Environment2(gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    # Configurar el entorno, incluyendo agentes (ASVs), el espacio de acciones y el espacio de observaciones
    def __init__(self, render_mode=None):
        super().__init__()
        #Agents + range
        self.n=2
        #inicializar clase parametrized path
        self.param_p = ParametrizedPath()
        self.current_step=0 #supervisar cuántos steps se han dado (contador)

        #Crear instancias de ASVAgent sin el argumento 'id' y con estado inicial aleatorio
        self.agents = [ASVAgent(id=n) for n in range (self.n)]
        self.beta_m()

        #Esta fórmula garantiza que cada vehículo se posicione en un ángulo
        #uniformemente distribuido alrededor del líder virtual, formando así
        #una formación circular o en función de la geometría deseada ajustando la fórmula de
        #beta_m = 2 * np.pi * i / self.n


        # 2 actionS might take place ai=(Tau ui, Tau vi)
        # Normalized for 3 USVs

        # Limite inferior y superior de las acciones surge y rotation (yaw) * numero de ASV (para 2 ASV [-1,-1,-1,-1] - [1,1,1,1])
        # Espacio de trabajo de empuje (eje x) y rotacion (eje y)
        self.action_space = spaces.Box(
            low=np.array([-1, -1]*self.n),
            high=np.array([1, 1, ]*self.n), dtype=np.float32)

        # posicion (dos primeros) orientacion (tercera), velocidad (cuarta y quinta) y velocidad angular (sexta) de los ASVs. Posicion deseada (dos ultimos)
        obs_space_l = np.array(
        [0, 0, -np.pi, -3.6, -3.6, -np.pi/3, 0, 0]*self.n, dtype=np.float32)

        # Extiende obs_space_l para incluir los límites inferiores de las observaciones relacionadas con el líder virtual:
        obs_space_l = np.hstack(
            [obs_space_l, np.array(
                [0, 0, 0, 0], dtype=np.float32)])#lider virtual pos (dos primeros) y error froma (tercero) y cross track error (cuarto) (minimos valores)

        # Límites superiores de las observaciones (posición y velocidad de los ASVs, posición y derivada de la trayectoria, error de formación y error de seguimiento)
        obs_space_h = np.array(
        [75, 75, np.pi, 3.6, 3.6, np.pi/3, 75, 75]*self.n, dtype=np.float32)

        obs_space_h = np.hstack(
            [obs_space_h, np.array(
                [75, 75, 100, 100], dtype=np.float32)]
        )

        # Example for using image as input (channel-first; channel-last also works):
        self.observation_space = spaces.Box(low=obs_space_l, high=obs_space_h, dtype=np.float32)

        # Render path
        self.render_mode = render_mode
        if self.render_mode == "human":
            thethas = np.linspace(0, 100, 2)
            painted_path = np.array([self.param_p.path(thetha) for thetha in thethas])
            self.start_path = np.array([painted_path[0][0], painted_path[0][1]])
            self.end_path = np.array([painted_path[-1][ 0], painted_path[-1][1]])

    # Asigna un angulo relativo para cada ASV
    def beta_m(self):
        for i, agent in enumerate(self.agents):
            agent.beta_m = 2 * np.pi * i / self.n

    # Devuelve el estado actual del ASV (observacion)
    def _get_obs(self, with_v=False):

        # Posición y velocidad del lider virtual
        pos_v, deriv = self.param_p.path(self.param_p.theta, True)

        # Inicializacion del vector observaciones y error de formación
        state = np.array([])
        error_f = 0

        # Almacena las observaciones y calcula la posicion esperada de cada ASV
        for agent in self.agents:
            if state.size:
                state = np.vstack(
                    [state, agent.x, agent.expected_position(pos_v, deriv)])
            else:
                state = np.vstack(
                    [agent.x, agent.expected_position(pos_v, deriv)]) # agent.x incluye posicion, orientacion y velocidad. expected_position incluye la posicion esperada de cada agente

            # Calcula el error de formación individual de cada ASV
            error_f += agent.error_f(pos_v, deriv)

        # Almacena la posicion del lider virutal, el error de formación promedio y cross-track error promedio (error perpendicular de los agentes respecto a la trayectoria)
        state = np.vstack([state, pos_v[0], pos_v[1], error_f/self.n,
                           self.param_p.cross_track_error(np.mean(
                               np.stack([agent.x[:2] for agent in self.agents]), axis=0))],
                                 dtype=np.float32).reshape(-1)
        if with_v:
            return state, pos_v, deriv
        return state

    # Interaccion entre entorno y ASV en un paso de tiempo
    def step(self, action): #aquí sucede la interacción
        #por cada gente fuerza y giro, observar
        self.current_step += 1 # en cada step estamos contando
        #actions = [Fi, Ti] * self.n
        #Slicing
        # Aplica la accion de cada ASV
        for i, agent in enumerate(self.agents):
            agent.evolve(action[i*2:i*2+2])
        observation, x_v, deriv = self._get_obs(with_v=True)
        #Imprime reward y el tipo de dato

        #recompenso mediante rv y rd
        # Calcula la metrica de direccion de la velocidad de cada ASV respecto a la trayectoria desdeada
        # Promedio que se usa en parte de la recompensa
        rv = [agent.Rv(x_v,deriv) for agent in self.agents] #deriva
        rv = sum(rv)/len(rv)
        # Desviacion de la posicion de cada ASV respecto a la posicion deseada
        rd = [agent.Rd(x_v, deriv) for agent in self.agents] #pos deseada (formation error)
        rd = sum(rd)/len(rd)
        #extrayendo el primer elemento de rv(que es un array) convirtiendolo en consecuencia en un escalar
        rv = rv.item() if isinstance(rv, np.ndarray) and rv.shape == (1,) else rv

        # Recompensa total
        reward = rv + rd

        # Calcula el error de formación promedio de los ASVs basado en la posicion promedio de los ASVs
        ate = self.param_p.along_track_error(np.mean(np.stack([agent.x[:2] for agent in self.agents]), axis=0))
        terminated = False

        # Termina el episodio si el error de formación es menor a 5 y suma 1 a la recompensa
        if (ate < 5):
        #     # self.param_p.theta += 1
            reward += 1
            terminated = True

        # self.param_p.update_theta(np.mean(np.stack([agent.x[:2] for agent in self.agents]), axis=0), 10)

        # Devuelve el estado actual del entorno despues del paso, la recompensa para el paso, si el episodio termino y si el paso fue truncado
        return observation, reward, terminated, False, {}


    # Reinicia el entorno y los agentes devolviendo los agentes a una posicion aleatoria
    def reset(self, seed=None, options=None):

        # for agent in self.agents:
        #     # randomize agents position
        #     agent.x = np.array([[np.random.uniform(0, 75)], [np.random.uniform(0, 75)], [
        #                        np.random.uniform(-np.pi, np.pi)], [0], [0], [0]], dtype=np.float32)
        # self.current_step = 0 #cuando reiniciamos la simulación, decimos que no ocurrió ningún paso del step (reinicio contador)

        # Reinicia el parametro que controla la posicion del lider virtual y la trayectoria
        self.param_p.theta = np.random.uniform(40, 70)  # 30, 50

        # Reinicia la posicion de los ASVs
        for i, agent in enumerate(self.agents):
            agent.x = np.array([[(2 - -1^(i+1))*5],
                                [(2 + -1^(i+1))*5],
                                [np.random.rand()*3],[0], [0], [0]], dtype=np.float32)

            #
            # 14 sin rand
            factor = 0.4 # El 60% de las veces se reinicia la posicion de los ASVs
            if np.random.rand() > factor:
                pos_v, deriv = self.param_p.path(self.param_p.theta, True)

                agent.x[0] =  agent.expected_position(pos_v, deriv)[0] - 50 # Desplegar la formación en un rango de 50
                agent.x[1] =  agent.expected_position(pos_v, deriv)[1] - 50

                x_p1 = agent.expected_position(pos_v, deriv) - agent.x[:2]
                angle = np.arctan2(x_p1[1], x_p1[0]) # Angulo entre la posicion esperada y la posicion actual
                agent.x[2] = angle
                agent.x[3] = np.random.rand() # Velocidad lineal aleatoria dentro del rango

        self.current_step = 0 #en cada step estamos contando (reinicio contador)

        return self._get_obs(), {}


    def render(self, mode="human"):
        # NED framework (inicializacion del entorno)
        self.canvas = np.ones((240, 320, 3), np.uint8) # Crea un lienzo de 240x320 pixeles con 3 canales de color
        RED = (0, 0, 255) # Color rojo para lineas o poligonos
        x_pad = 240/2 # Posicion en x e y del centro del lienzo
        y_pad = 160/2

        # Dibuja la trayectoria deseada
        cv2.line(self.canvas, (int(self.start_path[1][0] + x_pad), int(self.start_path[0][0] + y_pad)),
                    (int(self.end_path[1][0] + x_pad), int(self.end_path[0][0] + y_pad)), RED, 1)

        # Posicion del lider virtual, centro de la formacion y proyeccion de la formacion
        pos_v, slope = self.param_p.path(self.param_p.theta, True)
        pos_c = np.mean(np.stack([agent.x[:2] for agent in self.agents]), axis=0)
        pos_p = self.param_p.projection(pos_c)

        # draw these points (dibuja cada agente como un poligono rojo y un circulo amarillo en la posicion esperada de cada agente)
        for asv in self.agents:
            asv_center = (asv.x[1][0] + x_pad, asv.x[0][0] + y_pad)
            points = np.array([asv_center,
                               (asv.x[1][0] + x_pad + 4*np.cos(asv.x[2][0]),
                                asv.x[0][0] + y_pad - 4*np.sin(asv.x[2][0])),
                                (asv.x[1][0] + x_pad + 10*np.sin(asv.x[2][0]),
                                  asv.x[0][0] + y_pad + 10*np.cos(asv.x[2][0])),
                                (asv.x[1][0] + x_pad - 4*np.cos(asv.x[2][0]),
                                asv.x[0][0] + y_pad + 4*np.sin(asv.x[2][0])),], dtype=np.int32)

            pos_a_v = asv.expected_position(pos_v, slope)
            cv2.fillPoly(self.canvas, np.int32([points]), RED)
            cv2.circle(self.canvas, (int(pos_a_v[1][0] + x_pad), int(pos_a_v[0][0] + y_pad)),
                       5, (255, 255, 0), -1)

        # Dibuja el lider virtual (circulo verde), el centro promedio de los ASV (circulo amarillo) y la proyeccion del centro promedio (circulo magenta)
        cv2.circle(
            self.canvas, (int(pos_v[1][0] + x_pad), int(pos_v[0][0] + y_pad)), 5, (0, 255, 0), -1)
        cv2.circle(
            self.canvas, (int(pos_c[1][0] + x_pad), int(pos_c[0][0] + y_pad)), 3, (0, 255, 255), -1)
        cv2.circle(
            self.canvas, (int(pos_p[1][0] + x_pad), int(pos_p[0][0] + y_pad)), 5, (255, 0, 255), -1)


        _, x_v, deriv = self._get_obs(with_v=True)
        #Imprime reward y el tipo de dato

        #recompenso mediante rv y rd
        rv = [agent.Rv(x_v,deriv) for agent in self.agents] #deriva (recompensa de velocidad)
        rv = sum(rv)/len(rv) # promedio de la recompensa de velocidad
        rd = [agent.Rd(x_v, deriv) for agent in self.agents] #pos deseada (formation error) (penalizacion basada en el error de formacion)
        rd = sum(rd)/len(rd)
        reward = rv + rd
        # flip up-down
        self.canvas = cv2.flip(self.canvas, 0) # invierte la imagen para que el origen sea en la esquina inferior izquierda
        # zoom in
        self.canvas = cv2.resize(self.canvas, (640, 480), interpolation=cv2.INTER_AREA) # amplia la imagen a 640x480 pixeles

        # Imprime la recompensa y el error de formación en la ventana de visualización
        with np.printoptions(precision=3, suppress=True):
            cv2.putText(self.canvas, f"reward: {rv[0]:0.2f} + {rd:0.2f} =  {reward}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            ate = self.param_p.along_track_error(np.mean(np.stack([agent.x[:2] for agent in self.agents]), axis=0))
            cv2.putText(self.canvas, f"ate: {ate:0.2f}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)


        # if human imprime la imagen en una ventana
        if mode == "human":
            cv2.imshow("Game", self.canvas)
            cv2.waitKey(1)

        # si el modo es rgb_array, devuelve la imagen a color RGB
        elif mode == "rgb_array":
            return self.canvas



if __name__ == "__main__":
    # cheking env
    from stable_baselines3 import PPO

    # Create the environment
    env = Environment(render_mode="human")

    # Create the model PPO
    model = PPO("MlpPolicy", env, verbose=1) # Multilayer perceptron policy PPO training model
    env.reset() # Reinicia el entorno

    # Run the model (inicializa el bucle principal mienrtas running = true)
    running = True
    observation, _ = env.reset()

    # Toma decisiones. Utiliza la obs actual para predecir el siguiente paso
    while running:
        action, _ = model.predict(observation)

        # Ejecuta un paso en el entorno (observation = nuevo estado del entorno, reward = recompensa obtenida, terminated = si el episodio termino, truncated = si el paso fue truncado)
        observation, reward, terminated, truncated, info = env.step([0,0,0,0])
        # print(observation)

        env.render("human")
        if (terminated or truncated or env.current_step > 2048):
            observation, _ = env.reset()
            # running = False
