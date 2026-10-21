// ref_scenarios.cpp
//
// Nodo de referencias para los escenarios de comparación de controladores de
// velocidad de avance (u) y velocidad angular (r).
//
// - El escenario se elige con el parámetro "scenario" (1..5). Solo corre ese.
// - El escenario arranca al armar el vehículo (flanco de subida en mavros/state).
// - En cada paso (Ts) publica una ventana de N_ref muestras SIN retraso:
//     índice 0           -> instante actual t
//     índices 1..N_ref-1 -> valores futuros t + i*Ts del escenario
//   con x = u_ref [m/s], y = r_ref [rad/s], z = psi_ref [rad].
// - psi_ref parte del rumbo de la brújula al armar y se integra en lazo abierto
//   con r_ref (Euler hacia adelante). No se envuelve a [-pi, pi].
// - Al terminar el escenario la referencia pasa a (u_fin, r_fin) hasta desarmar.
// - Los parámetros se pueden cambiar en tiempo real solo con el vehículo
//   desarmado. El sufijo numérico indica el escenario al que afectan
//   (p. ej. t_ini5 solo afecta al escenario 5).

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "asv_interfaces/msg/reference_llc.hpp"

namespace
{

// ---------------------------------------------------------------------------
// Configuración. Los valores por defecto son los del documento de escenarios.
// ---------------------------------------------------------------------------
struct ScenarioConfig
{
  // Comunes
  int scenario = 1;       // escenario a ejecutar (1..5)
  double Ts = 0.1;        // periodo de muestreo [s] (el parámetro "Ts" va en ms)
  int N_ref = 30;         // muestras por ventana (actual + N_ref-1 futuras)
  double u_fin = 0.25;    // tramo de cierre [m/s]
  double r_fin = 0.0;     // tramo de cierre [rad/s]

  // Escenario 1: escalón en surge (r = 0)
  double u_min1 = 0.3, u_max1 = 1.0;
  double t_ini1 = 15.0, t_alto1 = 30.0, t_fin1 = 15.0;

  // Escenario 2: rampa en surge (r = 0)
  double u_min2 = 0.3, u_max2 = 1.0;
  double t_ini2 = 5.0, t_rampa2 = 25.0, t_alto2 = 5.0, t_fin2 = 5.0;

  // Escenario 3: escalones en yaw
  double u_cte3 = 0.5, r_max3 = 0.25;
  double t_ini3 = 5.0, t_escalon3 = 20.0, t_cero3 = 15.0, t_fin3 = 10.0;
  int s3 = 1;

  // Escenario 4: rampas en yaw
  double u_cte4 = 0.5, r_max4 = 0.25;
  double t_ini4 = 5.0, t_rampa4 = 15.0, t_hold4 = 5.0, t_fin4 = 5.0;
  int s4 = 1;

  // Escenario 5: senos simultáneos
  double u_min5 = 0.5, u_max5 = 0.8, T_u5 = 10.0;
  double r_amp5 = 0.1, T_r5 = 6.0;
  double t_ini5 = 5.0, t_senos5 = 45.0;
  int s5 = 1;
};

struct DoubleParamDef
{
  const char * name;
  double ScenarioConfig::* field;
  double min;
  double max;
};

struct IntParamDef
{
  const char * name;
  int ScenarioConfig::* field;
  int min;
  int max;
};

// Límites: u en [0.3, 1.2] m/s (mínimo de operación y saturación del vehículo),
// r en [0, 0.5] rad/s, tiempos en segundos.
const std::vector<DoubleParamDef> & doubleParams()
{
  static const std::vector<DoubleParamDef> defs = {
    // Comunes
    {"u_fin", &ScenarioConfig::u_fin, 0.0, 1.2},
    {"r_fin", &ScenarioConfig::r_fin, -0.5, 0.5},
    // Escenario 1
    {"u_min1", &ScenarioConfig::u_min1, 0.3, 1.2},
    {"u_max1", &ScenarioConfig::u_max1, 0.3, 1.2},
    {"t_ini1", &ScenarioConfig::t_ini1, 0.0, 300.0},
    {"t_alto1", &ScenarioConfig::t_alto1, 0.0, 300.0},
    {"t_fin1", &ScenarioConfig::t_fin1, 0.0, 300.0},
    // Escenario 2
    {"u_min2", &ScenarioConfig::u_min2, 0.3, 1.2},
    {"u_max2", &ScenarioConfig::u_max2, 0.3, 1.2},
    {"t_ini2", &ScenarioConfig::t_ini2, 0.0, 300.0},
    {"t_rampa2", &ScenarioConfig::t_rampa2, 1.0, 300.0},
    {"t_alto2", &ScenarioConfig::t_alto2, 0.0, 300.0},
    {"t_fin2", &ScenarioConfig::t_fin2, 0.0, 300.0},
    // Escenario 3
    {"u_cte3", &ScenarioConfig::u_cte3, 0.3, 1.2},
    {"r_max3", &ScenarioConfig::r_max3, 0.0, 0.5},
    {"t_ini3", &ScenarioConfig::t_ini3, 0.0, 300.0},
    {"t_escalon3", &ScenarioConfig::t_escalon3, 1.0, 300.0},
    {"t_cero3", &ScenarioConfig::t_cero3, 0.0, 300.0},
    {"t_fin3", &ScenarioConfig::t_fin3, 0.0, 300.0},
    // Escenario 4
    {"u_cte4", &ScenarioConfig::u_cte4, 0.3, 1.2},
    {"r_max4", &ScenarioConfig::r_max4, 0.0, 0.5},
    {"t_ini4", &ScenarioConfig::t_ini4, 0.0, 300.0},
    {"t_rampa4", &ScenarioConfig::t_rampa4, 1.0, 300.0},
    {"t_hold4", &ScenarioConfig::t_hold4, 0.0, 300.0},
    {"t_fin4", &ScenarioConfig::t_fin4, 0.0, 300.0},
    // Escenario 5
    {"u_min5", &ScenarioConfig::u_min5, 0.3, 1.2},
    {"u_max5", &ScenarioConfig::u_max5, 0.3, 1.2},
    {"T_u5", &ScenarioConfig::T_u5, 1.0, 60.0},
    {"r_amp5", &ScenarioConfig::r_amp5, 0.0, 0.5},
    {"T_r5", &ScenarioConfig::T_r5, 1.0, 60.0},
    {"t_ini5", &ScenarioConfig::t_ini5, 0.0, 300.0},
    {"t_senos5", &ScenarioConfig::t_senos5, 1.0, 300.0},
  };
  return defs;
}

const std::vector<IntParamDef> & intParams()
{
  static const std::vector<IntParamDef> defs = {
    {"scenario", &ScenarioConfig::scenario, 1, 5},
    {"N_ref", &ScenarioConfig::N_ref, 1, 200},
    {"s3", &ScenarioConfig::s3, -1, 1},
    {"s4", &ScenarioConfig::s4, -1, 1},
    {"s5", &ScenarioConfig::s5, -1, 1},
  };
  return defs;
}

// Comprueba rangos y relaciones entre parámetros. Devuelve "" si todo está bien.
std::string checkConfig(const ScenarioConfig & c)
{
  char buf[160];

  if (c.Ts < 0.05 - 1e-9 || c.Ts > 1.0 + 1e-9) {
    return "Ts: double en [50, 1000] ms";
  }
  for (const auto & d : doubleParams()) {
    const double v = c.*(d.field);
    if (v < d.min || v > d.max) {
      std::snprintf(buf, sizeof(buf), "%s: double en [%.2f, %.2f]", d.name, d.min, d.max);
      return buf;
    }
  }
  for (const auto & d : intParams()) {
    const int v = c.*(d.field);
    if (v < d.min || v > d.max) {
      std::snprintf(buf, sizeof(buf), "%s: entero en [%d, %d]", d.name, d.min, d.max);
      return buf;
    }
  }
  if (c.s3 == 0 || c.s4 == 0 || c.s5 == 0) {
    return "s3, s4 y s5 deben ser +1 o -1";
  }
  if (c.u_min1 >= c.u_max1) {return "u_min1 debe ser menor que u_max1";}
  if (c.u_min2 >= c.u_max2) {return "u_min2 debe ser menor que u_max2";}
  if (c.u_min5 >= c.u_max5) {return "u_min5 debe ser menor que u_max5";}
  return "";
}

// Tramo lineal del perfil: valores al inicio (u0, r0) y al final (u1, r1).
struct Segment
{
  double dur;
  double u0, u1;
  double r0, r1;
};

}  // namespace

// ---------------------------------------------------------------------------
// Nodo
// ---------------------------------------------------------------------------
class RefScenariosNode : public rclcpp::Node
{
public:
  RefScenariosNode()
  : Node("ref_scenarios")
  {
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    const std::string my_id = declare_parameter<std::string>("my_id", "ASV0", read_only);

    cfg_.Ts = declare_parameter<double>("Ts", cfg_.Ts * 1000.0) / 1000.0;
    for (const auto & d : doubleParams()) {
      cfg_.*(d.field) = declare_parameter<double>(d.name, cfg_.*(d.field));
    }
    for (const auto & d : intParams()) {
      cfg_.*(d.field) = static_cast<int>(
        declare_parameter<int64_t>(d.name, static_cast<int64_t>(cfg_.*(d.field))));
    }

    const std::string err = checkConfig(cfg_);
    if (!err.empty()) {
      RCLCPP_FATAL(get_logger(), "Configuración inválida: %s", err.c_str());
      throw std::invalid_argument(err);
    }

    sub_compass_ = create_subscription<std_msgs::msg::Float64>(
      "/" + my_id + "/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
      std::bind(&RefScenariosNode::compassCallback, this, std::placeholders::_1));
    sub_state_ = create_subscription<mavros_msgs::msg::State>(
      "/" + my_id + "/mavros/state", 1,
      std::bind(&RefScenariosNode::stateCallback, this, std::placeholders::_1));
    publisher_ref_ = create_publisher<asv_interfaces::msg::ReferenceLlc>(
      "/" + my_id + "/control/reference_llc", 1);

    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&RefScenariosNode::paramCallback, this, std::placeholders::_1));

    buildProfile();
    startTimer();

    RCLCPP_INFO(
      get_logger(),
      "Nodo de referencias en %s iniciado. Escenario %d (%.1f s), Ts = %.0f ms, ventana de %d muestras.",
      my_id.c_str(), cfg_.scenario, T_total_, cfg_.Ts * 1000.0, cfg_.N_ref);
  }

private:
  // -------------------------------------------------------------------------
  // Cambio de parámetros en tiempo real (solo desarmado, todo o nada)
  // -------------------------------------------------------------------------
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    auto reject = [&](const std::string & log_msg, const std::string & reason_msg) {
        RCLCPP_WARN(get_logger(), "%s", log_msg.c_str());
        result.successful = false;
        result.reason = reason_msg;
        return result;
      };

    std::lock_guard<std::mutex> lock(mutex_);

    if (armed_) {
      return reject("No se pueden cambiar parámetros", "ARMED: cambios de parámetros bloqueados");
    }

    // Se aplican todos los cambios sobre una copia y se valida el conjunto,
    // así las relaciones entre parámetros (u_min < u_max) se comprueban juntas.
    ScenarioConfig cand = cfg_;
    bool ts_changed = false;

    for (const auto & p : params) {
      const std::string & name = p.get_name();

      if (name == "Ts") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          return reject("No se pudo cambiar Ts", "Ts: double en [50, 1000] ms");
        }
        cand.Ts = p.as_double() / 1000.0;
        ts_changed = true;
        continue;
      }

      bool known = false;
      for (const auto & d : doubleParams()) {
        if (name == d.name) {
          known = true;
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            return reject(
              "No se pudo cambiar " + name, name + ": debe ser double (p. ej. 15.0)");
          }
          cand.*(d.field) = p.as_double();
          break;
        }
      }
      if (known) {continue;}

      for (const auto & d : intParams()) {
        if (name == d.name) {
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
            return reject("No se pudo cambiar " + name, name + ": debe ser entero");
          }
          cand.*(d.field) = static_cast<int>(p.as_int());
          break;
        }
      }
      // Otros parámetros (p. ej. use_sim_time) no se gestionan aquí.
    }

    const std::string err = checkConfig(cand);
    if (!err.empty()) {
      return reject("Cambio de parámetros rechazado: " + err, err);
    }

    cfg_ = cand;
    buildProfile();
    if (ts_changed) {
      startTimer();
    }
    for (const auto & p : params) {
      RCLCPP_INFO(
        get_logger(), "Parámetro %s = %s", p.get_name().c_str(), p.value_to_string().c_str());
    }
    return result;
  }

  // -------------------------------------------------------------------------
  // Perfil del escenario
  // -------------------------------------------------------------------------
  void buildProfile()
  {
    const auto & c = cfg_;
    segments_.clear();

    switch (c.scenario) {
      case 1:
        segments_ = {
          {c.t_ini1, c.u_min1, c.u_min1, 0.0, 0.0},
          {c.t_alto1, c.u_max1, c.u_max1, 0.0, 0.0},
          {c.t_fin1, c.u_min1, c.u_min1, 0.0, 0.0},
        };
        break;

      case 2:
        segments_ = {
          {c.t_ini2, c.u_min2, c.u_min2, 0.0, 0.0},
          {c.t_rampa2, c.u_min2, c.u_max2, 0.0, 0.0},
          {c.t_alto2, c.u_max2, c.u_max2, 0.0, 0.0},
          {c.t_rampa2, c.u_max2, c.u_min2, 0.0, 0.0},
          {c.t_fin2, c.u_min2, c.u_min2, 0.0, 0.0},
        };
        break;

      case 3: {
          const double u = c.u_cte3;
          const double r = c.s3 * c.r_max3;
          segments_ = {
            {c.t_ini3, u, u, 0.0, 0.0},
            {c.t_escalon3, u, u, r, r},
            {c.t_cero3, u, u, 0.0, 0.0},
            {c.t_escalon3, u, u, -r, -r},
            {c.t_fin3, u, u, 0.0, 0.0},
          };
          break;
        }

      case 4: {
          const double u = c.u_cte4;
          const double r = c.s4 * c.r_max4;
          segments_ = {
            {c.t_ini4, u, u, 0.0, 0.0},
            {c.t_rampa4, u, u, 0.0, r},
            {c.t_hold4, u, u, r, r},
            {c.t_rampa4, u, u, r, 0.0},
            {c.t_hold4, u, u, 0.0, 0.0},
            {c.t_rampa4, u, u, 0.0, -r},
            {c.t_hold4, u, u, -r, -r},
            {c.t_rampa4, u, u, -r, 0.0},
            {c.t_fin4, u, u, 0.0, 0.0},
          };
          break;
        }

      default:  // escenario 5: se evalúa de forma analítica en evaluate()
        break;
    }

    if (c.scenario == 5) {
      T_total_ = c.t_ini5 + c.t_senos5;
    } else {
      T_total_ = 0.0;
      for (const auto & s : segments_) {
        T_total_ += s.dur;
      }
    }
  }

  // Referencia (u, r) en el instante t [s] desde el armado.
  void evaluate(double t, double & u, double & r) const
  {
    const auto & c = cfg_;

    if (c.scenario == 5) {
      const double u_mid = 0.5 * (c.u_min5 + c.u_max5);
      const double u_amp = 0.5 * (c.u_max5 - c.u_min5);
      if (t < c.t_ini5) {
        u = u_mid;
        r = 0.0;
        return;
      }
      if (t < c.t_ini5 + c.t_senos5) {
        const double tau = t - c.t_ini5;
        u = u_mid + u_amp * std::sin(2.0 * M_PI * tau / c.T_u5);
        r = c.s5 * c.r_amp5 * std::sin(2.0 * M_PI * tau / c.T_r5);
        return;
      }
    } else {
      double t0 = 0.0;
      for (const auto & s : segments_) {
        if (t < t0 + s.dur) {  // un tramo de duración 0 nunca se cumple
          const double a = (t - t0) / s.dur;
          u = s.u0 + (s.u1 - s.u0) * a;
          r = s.r0 + (s.r1 - s.r0) * a;
          return;
        }
        t0 += s.dur;
      }
    }

    // Tramo de cierre: se mantiene hasta que el operador desarma.
    u = c.u_fin;
    r = c.r_fin;
  }

  // -------------------------------------------------------------------------
  // Publicación periódica
  // -------------------------------------------------------------------------
  void startTimer()
  {
    if (timer_) {
      timer_->cancel();
    }
    const auto period = std::chrono::milliseconds(
      static_cast<int64_t>(std::lround(cfg_.Ts * 1000.0)));
    timer_ = create_wall_timer(period, std::bind(&RefScenariosNode::timerCallback, this));
  }

  void timerCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!armed_) {
      return;
    }

    const double t = static_cast<double>(k_) * cfg_.Ts;

    if (!finished_ && t >= T_total_) {
      finished_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Escenario %d terminado (t = %.1f s). Cierre con u = %.2f m/s, r = %.2f rad/s hasta desarmar.",
        cfg_.scenario, t, cfg_.u_fin, cfg_.r_fin);
    }

    asv_interfaces::msg::ReferenceLlc msg;
    msg.references.reserve(static_cast<size_t>(cfg_.N_ref));

    double psi = psi_act_;
    double r_now = 0.0;
    for (int i = 0; i < cfg_.N_ref; ++i) {
      double u = 0.0, r = 0.0;
      evaluate(t + i * cfg_.Ts, u, r);
      if (i == 0) {
        r_now = r;
      }
      geometry_msgs::msg::Vector3 v;
      v.x = u;
      v.y = r;
      v.z = psi;
      msg.references.push_back(v);
      psi += r * cfg_.Ts;
    }

    publisher_ref_->publish(msg);

    psi_act_ += r_now * cfg_.Ts;
    ++k_;
  }

  // -------------------------------------------------------------------------
  // MAVROS
  // -------------------------------------------------------------------------
  void compassCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    init_psi_ = msg->data * M_PI / 180.0;
    compass_ok_ = true;
  }

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (msg->armed && !armed_) {
      k_ = 0;
      psi_act_ = init_psi_;
      finished_ = false;
      buildProfile();
      if (!compass_ok_) {
        RCLCPP_WARN(get_logger(), "Armado sin dato de brújula: psi inicial = 0");
      }
      RCLCPP_INFO(
        get_logger(), "Escenario %d iniciado: duración %.1f s, psi inicial = %.3f rad",
        cfg_.scenario, T_total_, psi_act_);
    } else if (!msg->armed && armed_) {
      RCLCPP_INFO(get_logger(), "Desarmado: se detiene la publicación de referencias");
    }

    armed_ = msg->armed;
  }

  // -------------------------------------------------------------------------
  // Estado
  // -------------------------------------------------------------------------
  std::mutex mutex_;
  ScenarioConfig cfg_;
  std::vector<Segment> segments_;
  double T_total_ = 0.0;

  bool armed_ = false;
  bool finished_ = false;
  bool compass_ok_ = false;
  double init_psi_ = 0.0;
  double psi_act_ = 0.0;
  int64_t k_ = 0;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_compass_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_state_;
  rclcpp::Publisher<asv_interfaces::msg::ReferenceLlc>::SharedPtr publisher_ref_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RefScenariosNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
