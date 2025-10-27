#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace casadi;

// Funzione per convertire un vettore CasADi DM in std::vector<double>
std::vector<double> DM_to_std_vector(const DM& dm) {
    std::vector<double> result(dm.numel());
    for (int i = 0; i < dm.numel(); ++i) {
        result[i] = static_cast<double>(dm(i));
    }
    return result;
}

// Funzione per il coniugato di un quaternione CasADi SX
// q è assunto essere [qx, qy, qz, qw]
SX casadi_quat_conj(const SX& q) {
    return SX::vertcat({-q(0), -q(1), -q(2), q(3)});
}

// Funzione per la moltiplicazione di due quaternioni CasADi SX
// q1 ⊗ q2 --- Hamilton Product
// q1, q2 sono assunti essere [qx, qy, qz, qw]
SX casadi_quat_mult(const SX& q1, const SX& q2) {
    SX x1 = q1(0), y1 = q1(1), z1 = q1(2), w1 = q1(3);
    SX x2 = q2(0), y2 = q2(1), z2 = q2(2), w2 = q2(3);

    SX x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    SX y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    SX z = w1*z2 + x1*y2 - y1*x2 + z1*w2;
    SX w = w1*w2 - x1*x2 - y1*y2 - z1*z2;

    return SX::vertcat({x, y, z, w});
}

// ----------------------------------------------------------------------------------------------

void updateObstacleEstimate(const Eigen::Vector3d& o_k_measured,
                          double ts_obs, double alpha1_obs, double alpha2_obs, double alpha3_obs,
                          Eigen::Vector3d& o_hat_curr,
                          Eigen::Vector3d& v_hat_curr,
                          Eigen::Vector3d& a_hat_curr,
                          bool& is_initialized)
{
    // Blocco di inizializzazione: eseguito solo alla prima chiamata valida (in teoria t=0)
    if (!is_initialized) {
        // La prima stima della posizione è la misura attuale
        o_hat_curr = o_k_measured;
        // Si assume velocità e accelerazione iniziali nulle non avendo informazioni precedenti
        v_hat_curr.setZero();
        a_hat_curr.setZero();
        is_initialized = true;
        return;
    }

    // Per le chiamate successive alla prima:
    // Le variabili "_curr" passate per riferimento contengono le stime del ciclo precedente (es. o_hat_{k|k-1}).
    Eigen::Vector3d o_hat_prev_local = o_hat_curr;
    Eigen::Vector3d v_hat_prev_local = v_hat_curr;
    Eigen::Vector3d a_hat_prev_local = a_hat_curr;
    // Calcola l'errore di stima della posizione: differenza tra la misura attuale e la stima precedente.
    Eigen::Vector3d position_estimation_error = o_k_measured - o_hat_prev_local;

    // Applica le equazioni di aggiornamento del GPIO per calcolare le stime correnti (es. o_hat_{k|k}).
    // Le variabili o_hat_curr, v_hat_curr, a_hat_curr vengono sovrascritte con i nuovi valori.
    o_hat_curr = o_hat_prev_local + ts_obs * (v_hat_prev_local + alpha1_obs * position_estimation_error);
    v_hat_curr = v_hat_prev_local + ts_obs * (a_hat_prev_local + alpha2_obs * position_estimation_error);
    a_hat_curr = a_hat_prev_local + ts_obs * (alpha3_obs * position_estimation_error);
}

std::vector<Eigen::Vector3d> predictObstacleTrajectory(const Eigen::Vector3d& o_hat_now,
                                                     const Eigen::Vector3d& v_hat_now,
                                                     const Eigen::Vector3d& a_hat_now,
                                                     int N_horizon, double ts_obs)
{
    std::vector<Eigen::Vector3d> predicted_o_trajectory;
    predicted_o_trajectory.reserve(N_horizon + 1);                                                                  // MODIFICATO

    // AggiungO la stima attuale come primo elemento della traiettoria predetta (corrisponde a o_hat_{k|k})     // AGGIUNTO
    predicted_o_trajectory.push_back(o_hat_now);

    Eigen::Vector3d o_pred_j = o_hat_now;
    Eigen::Vector3d v_pred_j = v_hat_now;
    Eigen::Vector3d a_pred_j = a_hat_now;

    // Loop per N passi di predizione
    for (int j = 0; j < N_horizon; ++j) {
        // Modello di predizione ostacolo (accelerazione costante)
        Eigen::Vector3d o_next = o_pred_j + ts_obs * v_pred_j;
        Eigen::Vector3d v_next = v_pred_j + ts_obs * a_pred_j;
        Eigen::Vector3d a_next = a_pred_j;

        predicted_o_trajectory.push_back(o_next); // Sto predicendo o_{j+1} rispetto all'inizio dell'orizzonte

        // Aggiorna lo stato per la prossima iterazione di predizione
        o_pred_j = o_next;
        v_pred_j = v_next;
        a_pred_j = a_next;
    }
    return predicted_o_trajectory;
}

// --------------------------------------------------------------------------------------------^

int main(int argc, char *argv[]) {
    // Inizializza ROS 2
    rclcpp::init(argc, argv);

    // Crea il nodo ROS 2
    auto const node = std::make_shared<rclcpp::Node>(
        "mpc_controller",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // msg per velocità dei giunti
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub;

    // Logger
    auto const logger = rclcpp::get_logger("mpc_controller");

    // Esecutore ROS 2 per elaborare i messaggi in background
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Inizializza il publisher per la velocità dei giunti
    // velocity_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 10);
    velocity_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/niryo_velocity_controller/commands", 10);

    // Interfaccia MoveGroup per il manipolatore
    using moveit::planning_interface::MoveGroupInterface;
    auto arm_group_interface = MoveGroupInterface(node, "niryo_arm");

    // Ottieni la posa corrente dell'end-effector
    auto current_pose = arm_group_interface.getCurrentPose();
    Eigen::Vector3d position(current_pose.pose.position.x,
                             current_pose.pose.position.y,
                             current_pose.pose.position.z);
    Eigen::Quaterniond orientation(current_pose.pose.orientation.w,
                                   current_pose.pose.orientation.x,
                                   current_pose.pose.orientation.y,
                                   current_pose.pose.orientation.z);

    // Ottieni le posizioni correnti delle giunture
    std::vector<double> current_joint_values = arm_group_interface.getCurrentJointValues();

    // Costruisci lo stato iniziale completo
    DM x_e_current = DM::vertcat({
        position.x(), position.y(), position.z(),
        orientation.x(), orientation.y(), orientation.z(), orientation.w()
    });
    std::vector<casadi::DM> joints_dm(current_joint_values.size());
    std::transform(current_joint_values.begin(), current_joint_values.end(), joints_dm.begin(),
                   [](double val) { return casadi::DM(val); });
    DM theta_current = casadi::DM::vertcat(joints_dm);

    DM x_current = casadi::DM::vertcat({x_e_current, theta_current});

    std::vector<double> joint_lower = {
        -2.9998, -1.8325, -1.3400, -2.0900, -1.9200, -2.5300
    };

    std::vector<double> joint_upper = {
         2.9998,  0.6101,  1.5700,  2.0900,  1.9228,  2.5300
    };

    std::vector<double> joint_velocity_max = {
       10.0, 8.0, 8.0, 2.0, 2.0, 2.0
    //   1.0, 0.8, 0.8, 0.2, 0.2, 0.2
    };

    // Per calcolare lo jacobiano
    const moveit::core::RobotModelConstPtr& robot_model = arm_group_interface.getRobotModel();
    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("niryo_arm");

    const int N = 2;     // Orizzonte
    const double dt = 0.1; // ---------CAMBIATO-------- PRIMA AVEVO Timestep (25Hz)                   // max_timesteps=100 PRIMA
    const int max_timesteps = 100; // Durata massima simulazione (Oss: La simulazione con chomp dura 77 timestep, circa 3s, avendo considerato dt=40ms )

    // reference traiettoria ee letto da file
    std::vector<std::vector<double>> reference_ee_traj;

    std::ifstream infile("/home/francesco/lab_ROS2/ws_moveit2/src/niryo_moveit2_config/cpp_script/ref_ee_traj_100ms.txt");
    if (!infile.is_open()) {
        RCLCPP_ERROR(node->get_logger(), "Errore nell'aprire il file ref_ee_traj_100ms.txt");
        return 1;
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        std::vector<double> pose(7);
        for (int i = 0; i < 7; ++i) {
            iss >> pose[i];
        }
        reference_ee_traj.push_back(pose);  // 77x7 (Oss: Considerando un tempo di campionamento di 40ms -> La simulazione con chomp dura 77 timestep , circa 3s )
    }
    infile.close();

    // Padding finale: ripeti l'ultima pose per garantire che ci siano almeno max_timesteps + N - 1 elementi
    int required_length = max_timesteps + N - 1;
    while (reference_ee_traj.size() < required_length) {
        reference_ee_traj.push_back(reference_ee_traj.back());
    }


    // Debug: stampa tutta la matrice letta
    for (size_t i = 0; i < reference_ee_traj.size(); ++i) {
        std::ostringstream oss;
        oss << "EE_ref[" << i << "]: ";
        for (double val : reference_ee_traj[i]) {
            oss << val << " ";
        }
        RCLCPP_INFO(node->get_logger(), "%s", oss.str().c_str());
    }

    // ------------------------------------------------------------------------------------

    // (1) Definizione variabili per posa ostacolo
    geometry_msgs::msg::PoseStamped latest_obstacle_pose_msg; // Memorizza l'intero messaggio PoseStamped
    std::mutex obstacle_pose_mutex;                           // Protegge l'accesso a latest_obstacle_pose_msg
    bool new_obstacle_data_available = false;                 // Flag per segnalare nuovi dati

    // (2) Subscriber e Callback
    auto obstacle_pose_callback =
        [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(obstacle_pose_mutex);
        latest_obstacle_pose_msg = *msg;
        new_obstacle_data_available = true; // Segnala che un nuovo dato è arrivato
        RCLCPP_DEBUG(node->get_logger(), "Posizione ostacolo aggiornata: [%.3f, %.3f, %.3f]",
                     msg->pose.position.x,
                     msg->pose.position.y,
                     msg->pose.position.z);
    };

    auto obstacle_pose_subscriber = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/obstacle_pose", 10, obstacle_pose_callback);

    // ----------------------------------------------------------------------------------^

    // VARIABILI DI STATO PER L'OSSERVATORE DELL'OSTACOLO
    // Queste memorizzano lo stato stimato dell'ostacolo tra i cicli MPC.
    Eigen::Vector3d observer_o_hat;         // Posizione stimata corrente [ox, oy, oz]
    Eigen::Vector3d observer_v_hat;         // Velocità stimata corrente [vx, vy, vz]
    Eigen::Vector3d observer_a_hat;         // Accelerazione stimata corrente [ax, ay, az]
    bool observer_is_initialized = false;   // Flag per la prima inizializzazione

    // L’osservatore lavora con lo stesso timestep del controller MPC
    const double obs_ts = dt;

    // Guadagni dell'osservatore valutati su Matlab
    const double obs_alpha1 = 2.0;
    const double obs_alpha2 = 1.6;
    const double obs_alpha3 = 0.128;


    // === Resto del codice MPC ===
    // === Simboli ===
    SX Jp_sym = SX::sym("Jp", 3, 6); // Jacobiano per velocità lineare
    SX Jo_sym = SX::sym("Jo", 3, 6); // Jacobiano per velocità angolare
    SX S_ref_flat = SX::sym("S_ref_flat", 7 * N);  // reference desiderata EE (vettore)
    SX S_ref= reshape(S_ref_flat, 7, N);           // reference desiderata EE (matrice)
    SX x0 = SX::sym("x0", 13);            // Stato iniziale (xee + theta)

    // === Variabili decisionali ===
    SX U = SX::sym("U", 6, N);            // Comandi di controllo
    SX X = SX::sym("X", 13, N + 1);       // Stati predetti: x_0 ... x_N

    SX gamma_ee_k_sym = SX::sym("gamma_ee_k", N); // Tasso di decadimento gamma_k per l'EE (Nx1)            AGGIUNTO

    // Parametro Simbolico per la TRAIETTORIA PREDETTA dell'ostacolo (3(N+1) x 1)
    SX O_pred_flat_sym = SX::sym("O_pred_flat", 3 * (N+1));  // (vettore)                                        MODIFICATO
    SX O_pred_matrix_sym = reshape(O_pred_flat_sym, 3, N+1); // (matrice) Utile per l'accesso nel loop           AGGIUNTO

    // === Costanti peso ===
    SX Q = SX::diag(SX::vertcat({SX::ones(3)*2000, SX::ones(3)*100}));
    SX R = SX::diag(SX::ones(6) * 5);

    SX P_gamma_ee = SX::diag(SX::ones(1) * 200);                                                          // AGGIUNTO

    // === Costo e vincoli ===
    SX J = SX::zeros();
    std::vector<SX> g;  // Vincoli
    std::vector<double> lbg_vec;
    std::vector<double> ubg_vec;

    // Stato iniziale noto
    g.push_back(X(Slice(), 0) - x0);  // X(Slice(), 0) - x0 = 0
    for (int i = 0; i < 13; ++i) {
        lbg_vec.push_back(0.0);
        ubg_vec.push_back(0.0);
    }


    // PARAMETRI PER CBF (m)                                                                                     AGGIUTO (VEDI DOPO)
    double d_min_cbf = 0.001;  // Distanza minima che si vuole mantenere tra EE e la superficie dell'ostacolo.
    double Ro_cbf = 0.0866;    // Raggio della sfera circoscritta dell'ostacolo
    // double delta_cbf = 0.08;   // Limite superiore per la norma dell'errore di stima iniziale dell'osservatore ||E(0)|| ≤ delta (da definire)
    // double phi0_cbf = 2.43;   //  Coeff. che scala l'incertezza iniziale dell'errore di stima. (calcolato con script MATLAB)
    // double rd_cbf = delta_cbf * phi0_cbf;  // Margine di sicurezza aggiuntivo per compensare gli errori di stima dell'osservatore
    double rd_cbf = 0.02;
    double r_safe_cbf = d_min_cbf + Ro_cbf + rd_cbf;  // Distanza di sicurezza ( 0.001 + 0.0866 + 0.02 = 0.1076 m = 10,76 cm )


    for (int i = 0; i < N; ++i) {
        SX x_i = X(Slice(), i);
        SX u_i = U(Slice(), i);
        SX p_i = x_i(Slice(0, 3)); // Posizione predetta
        SX q_i = x_i(Slice(3, 7)); // Quaternione predetto [qx,qy,qz,qw]
        SX theta_i = x_i(Slice(7, 13));

        SX gamma_ee_i = gamma_ee_k_sym(i); // Tasso di decadimento per l'EE per questo step i                     AGGIUNTO

        // Predizione angoli giunti
        SX theta_next = theta_i + dt * u_i;

        // Predizione posizione ee
        SX p_dot_i = mtimes(Jp_sym, u_i);
        SX p_next = p_i + dt * p_dot_i;

        // Predizione orientamento ee
        // 1. Calcola la velocità angolare omega_i (3x1)
        SX omega_i = mtimes(Jo_sym, u_i); // Usa Jo_sym

        // 2. Forma un quaternione puro da omega_i per la moltiplicazione.
        //    Per la convenzione [qx,qy,qz,qw], omega_pure è [omegax,omegay,omegaz,0]
        SX omega_pure_i = SX::vertcat({omega_i(0), omega_i(1), omega_i(2), 0.0});

        // 3. Calcola il termine (omega_pure_i ⊗ q_i) usando la funzione casadi_quat_mult
        //    q_i è [qxi, qyi, qzi, qwi]
        SX q_derivative_term = casadi_quat_mult(omega_pure_i, q_i);

        // 4. Integra il quaternione: q_next = q_i + 0.5 * dt * ( omega_pure_i ⊗ q_i )
        SX q_next = q_i + 0.5 * dt * q_derivative_term;

        // 5. Normalizza simbolicamente q_next
        SX sq_norm_q_next = dot(q_next, q_next); // Norma quadrata (dot=prodotto scalare)
        SX q_next_normalized = q_next / sqrt(sq_norm_q_next + 1e-12); // Epsilon piccolo dentro sqrt per evitare un ipotetico sqrt(0).

        // Ricomponi lo stato EE e lo stato totale
        SX x_e_next_final = SX::vertcat({p_next, q_next_normalized});
        SX x_next_final = SX::vertcat({x_e_next_final, theta_next});

        // Vincolo di dinamica: x_{i+1} = x_next_final
        g.push_back(X(Slice(), i + 1) - x_next_final);
        for (int j = 0; j < 13; ++j) {
            lbg_vec.push_back(0.0);
            ubg_vec.push_back(0.0);
        }

        // Vincoli sulla posizione e velocità dei giunti
        for (int j = 0; j < 6; ++j) {
            // Vincoli: theta_next >= lower
            g.push_back(theta_next(j) - joint_lower[j]);
            lbg_vec.push_back(0.0);          // >= 0
            ubg_vec.push_back(casadi::inf);

            // Vincoli: theta_next <= upper
            g.push_back(joint_upper[j] - theta_next(j));
            lbg_vec.push_back(0.0);          // >= 0
            ubg_vec.push_back(casadi::inf);

            // u_i(j) >= -max_velocity
            g.push_back(u_i(j) + joint_velocity_max[j]);
            lbg_vec.push_back(0.0);         // >= 0
            ubg_vec.push_back(casadi::inf);

            // u_i(j) <= +max_velocity
            g.push_back(joint_velocity_max[j] - u_i(j));
            lbg_vec.push_back(0.0);         // >= 0
            ubg_vec.push_back(casadi::inf);
        }


        // Vincoli per CBF (0 < gamma_ee_i <= 1)                                                 AGGIUNTO
        double epsilon_gamma = 1e-6; // Piccolo valore per > 0

        g.push_back(gamma_ee_i - epsilon_gamma);  // gamma_ee_i >= epsilon_gamma (invece di fare >0)
        lbg_vec.push_back(0.0);
        ubg_vec.push_back(casadi::inf);

        g.push_back(1.0 - gamma_ee_i);            // gamma_ee_i <= 1
        lbg_vec.push_back(0.0);
        ubg_vec.push_back(casadi::inf);


        // --- AGGIUNTA DEL VINCOLO DI SICUREZZA CBFSC PER L'EE ---                             AGGIUNTO

        // Estraggo le posizioni predette dell'ostacolo allineate temporalmente con p_i e p_next.
        SX o_obstacle_current_simb = O_pred_matrix_sym(Slice(), i);     // Posizione ostacolo ô_{i|k}
        SX o_obstacle_next_simb    = O_pred_matrix_sym(Slice(), i + 1); // Posizione ostacolo ô_{i+1|k}

        // Calcola H_attuale = H(x_e,i|k, o_hat_i|k)
        SX H_ee_current = norm_2(p_i - o_obstacle_current_simb) - r_safe_cbf;

        // Calcola H_successiva = H(x_e,i+1|k, o_hat_i+1|k)
        SX H_ee_next = norm_2(p_next - o_obstacle_next_simb) - r_safe_cbf;

        // Formula completa del vincolo CBFSC: H_next - (1 - gamma_ee_i) * H_current >= 0
        g.push_back(H_ee_next - (1 - gamma_ee_i) * H_ee_current);
        lbg_vec.push_back(0.0);         // Limite inferiore del vincolo (>= 0)
        ubg_vec.push_back(casadi::inf); // Limite superiore del vincolo (infinito)


         // === Calcolo Errore ===
        SX p_ref_i = S_ref(Slice(0,3), i);
        SX q_ref_i = S_ref(Slice(3,7), i);

        // 1. Gestisci ambiguità q vs -q
        SX q_pred_aligned = if_else(dot(q_ref_i, q_i) < 0, -q_i, q_i);

        // 2. Calcola q_err = q_ref * q_pred_aligned_conjugate
        SX q_pred_conj = casadi_quat_conj(q_pred_aligned);
        SX q_err = casadi_quat_mult(q_ref_i, q_pred_conj);

        // 3. Estrai errore 3D
        SX err_orient_vec = q_err(Slice(0,3)) * 2; // Parte vettoriale di q_err * 2

        // 4. Errore di posizione
        SX err_pos_vec = p_ref_i - p_i;

        // 5. Vettore errore totale 6D
        SX e_total_i = SX::vertcat({err_pos_vec, err_orient_vec});
        J += mtimes(mtimes(e_total_i.T(), Q), e_total_i) + mtimes(mtimes(u_i.T(), R), u_i) + P_gamma_ee * pow(gamma_ee_i, 2);    // AGGIUNTO
    }


    // === Concatenazione variabili decisionali ===
    SX OPT_VARS = SX::vertcat({reshape(U, 6 * N, 1), reshape(X, 13 * (N + 1), 1), reshape(gamma_ee_k_sym, N, 1)});               // AGGIUNTO

    // === Concatenazione vincoli ===
    SX G = SX::vertcat(g);

    // === NLP ===
    casadi::SXDict nlp;
    nlp["x"] = OPT_VARS;
    nlp["f"] = J;
    nlp["g"] = G;
    nlp["p"] = SX::vertcat({x0,                         // 13x1       Parametri: stato iniziale,
                        reshape(Jp_sym, 18, 1),         // 3x6                   Jacobiano posizione,
                        reshape(Jo_sym, 18, 1),         // 3x6                   Jacobiano orientamneto,
                        S_ref_flat,                     // 7N x 1                target,
                        O_pred_flat_sym});              // 3(N+1) x 1            traiettoria predetta ostacolo.

    Function solver = nlpsol("nlp", "ipopt", nlp);

    RCLCPP_INFO(logger, "NLP pronto per essere risolto con IPOPT");

    // === Loop MPC ===                                                                                AGGIUNTO
    int num_U_vars = 6 * N;
    int num_X_vars = 13 * (N + 1);
    int num_gamma_ee_vars = N;
    int total_decision_vars = num_U_vars + num_X_vars + num_gamma_ee_vars;

    // Variabile per salvare la soluzione del ciclo precedente
    DM previous_solution = DM::zeros(total_decision_vars); // Inizializzazione iniziale                 AGGIUNTO

    // Inizializza con stati iniziali e velocità iniziali (nulle)
    for (int i = 0; i < N + 1; ++i) {
        for (int j = 0; j < 13; ++j) {
           previous_solution(num_U_vars + i * 13 + j) = x_current(j); // Usa x_current per gli stati
        }
    }

    // Inizializza gamma                                                                                    AGGIUNTO
    for (int i = 0; i < N; ++i) {
        // La parte di gamma inizia dopo U e X
        previous_solution(num_U_vars + num_X_vars + i) = 0.001;
    }

    // (1) Inizializza il clock monotono
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    rclcpp::Time iteration_start = steady_clock.now();

    bool first_run = true; // Flag per la prima esecuzione
    casadi::DM last_u = DM::zeros(6, 1); // u_{k-1}, inizializzato a zero

    for (int t = 0; t < max_timesteps; ++t) {

        if (first_run) {
            first_run = false; // Usa lo stato iniziale già definito
        } else {

            // Costruisco msg u_{n-1}
            std_msgs::msg::Float64MultiArray vel_msg;
            vel_msg.data.resize(6);
            for (int i = 0; i < 6; ++i) {
                vel_msg.data[i] = static_cast<double>(last_u(i));
            }

            // Verifico se sono passati 100ms oppure aspetto
            if (t >= 2) {
                auto iteration_end = steady_clock.now();
                auto duration = iteration_end - iteration_start;
                auto elapsed_ms = duration.nanoseconds() / 1'000'000;
                if (elapsed_ms < 100) {                                                                             // ---CAMBIATO---
                    std::this_thread::sleep_for(std::chrono::milliseconds(100 - elapsed_ms));                       // ---CAMBIATO---
                } else {
                    RCLCPP_WARN(logger, "Cycle %d, Tempo ciclo maggiore di 100ms, precisamente: %ld ms", t, elapsed_ms);
                }
            }

            // Applico u_{n-1}
            velocity_pub->publish(vel_msg);
            iteration_start = steady_clock.now();

            // Log azione di controllo
            RCLCPP_INFO(logger, "Azione di Controllo:");
            for (size_t i = 0; i < vel_msg.data.size(); ++i) {
                RCLCPP_INFO(logger, "  u[%zu]: %f", i, vel_msg.data[i]);
            }

            // Aggiorna x_current
            auto pose_start = steady_clock.now();    // DEBUG 1
            auto current_pose = arm_group_interface.getCurrentPose();
            auto pose_end = steady_clock.now();      // DEBUG 1
            RCLCPP_INFO(logger, "Tempo getCurrentPose(): %ld ms", (pose_end - pose_start).nanoseconds() / 1'000'000);  // DEBUG 1
            Eigen::Vector3d position(current_pose.pose.position.x,
                                     current_pose.pose.position.y,
                                     current_pose.pose.position.z);
            Eigen::Quaterniond orientation(current_pose.pose.orientation.w,
                                           current_pose.pose.orientation.x,
                                           current_pose.pose.orientation.y,
                                           current_pose.pose.orientation.z);

            auto joint_start = steady_clock.now();  // DEBUG 2
            std::vector<double> current_joint_values = arm_group_interface.getCurrentJointValues();
            auto joint_end = steady_clock.now();    // DEBUG 2
            RCLCPP_INFO(logger, "Tempo getCurrentJointValues(): %ld ms", (joint_end - joint_start).nanoseconds() / 1'000'000); // DEBUG 2

            x_e_current = DM::vertcat({
                position.x(), position.y(), position.z(),
                orientation.x(), orientation.y(), orientation.z(), orientation.w()
            });

            joints_dm.clear(); // Pulisci il vettore esistente
            for (double val : current_joint_values) {
                joints_dm.push_back(casadi::DM(val)); // Popola con i nuovi valori
            }
            theta_current = casadi::DM::vertcat(joints_dm);

            x_current = casadi::DM::vertcat({x_e_current, theta_current});
        }

        auto state_start = steady_clock.now();  // DEBUG 3
        // === Calcolo dinamico dello Jacobiano geometrico ===
        moveit::core::RobotStatePtr current_state = arm_group_interface.getCurrentState();

        std::vector<double> current_joint_values = DM_to_std_vector(theta_current);
        // Aggiorna lo stato del robot
        current_state->setJointGroupPositions(joint_model_group, current_joint_values);
        current_state->update();

        // Calcolo dello Jacobiano geometrico (6x6)
        Eigen::MatrixXd J_ee(6, joint_model_group->getVariableCount());
        current_state->getJacobian(
            joint_model_group,
            current_state->getLinkModel("tool_link"),
            Eigen::Vector3d::Zero(),
            J_ee,
            false //6x6
        );
        auto state_end = steady_clock.now();  // DEBUG 3
        RCLCPP_INFO(logger, "Tempo update stato + Jacobiano: %ld ms", (state_end - state_start).nanoseconds() / 1'000'000);  // DEBUG 3

        // auto jacobian_convert_start = steady_clock.now();  // DEBUG 4

        // Estrai Jp (prime 3 righe) e Jo (ultime 3 righe) da J_ee
        Eigen::MatrixXd Jp_ee = J_ee.topRows(3);     // 3x6
        Eigen::MatrixXd Jo_ee = J_ee.bottomRows(3);  // 3x6

        // Converti Jp_ee in casadi::DM Jp_live_ee
        DM Jp_live_ee = DM::zeros(3, joint_model_group->getVariableCount());
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < joint_model_group->getVariableCount(); ++c) {
                Jp_live_ee(r, c) = Jp_ee(r, c);
            }
        }

        // Converti Jo_ee in casadi::DM Jo_live_ee
        DM Jo_live_ee = DM::zeros(3, joint_model_group->getVariableCount());
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < joint_model_group->getVariableCount(); ++c) {
                Jo_live_ee(r, c) = Jo_ee(r, c);
            }
        }

        // auto jacobian_convert_end = steady_clock.now();   // DEBUG 4
        // RCLCPP_INFO(logger, "Tempo conversione Jacobiano in DM: %ld ms", (jacobian_convert_end - jacobian_convert_start).nanoseconds() / 1'000'000);   // DEBUG 4

        // Log per verifica
        // RCLCPP_INFO_STREAM(logger, "Jp_live_ee:\n" << Jp_live_ee);
        // RCLCPP_INFO_STREAM(logger, "Jo_live_ee:\n" << Jo_live_ee);

        // auto ref_start = steady_clock.now();  // DEBUG 5
        // Dinamicamente seleziona solo N pose a partire da t
        std::vector<double> flattened_ref;
        for (int i = t; i < t + N; ++i) {
            const auto& pose = reference_ee_traj[i];
            flattened_ref.insert(flattened_ref.end(), pose.begin(), pose.end());
        }
        DM reference_ee_traj_dm = DM(flattened_ref); // vettore colonna con 7xN elementi
        // auto ref_end = steady_clock.now();   // DEBUG 5
        // RCLCPP_INFO(logger, "Tempo costruzione riferimento MPC: %ld ms", (ref_end - ref_start).nanoseconds() / 1'000'000);   // DEBUG 5


        // --- SEZIONE OSSERVATORE E PREDIZIONE OSTACOLO ---
        Eigen::Vector3d o_k_measured_for_observer;
        bool use_this_measurement = false;

        { // Blocco per limitare lo scope del lock_guard
            std::lock_guard<std::mutex> lock(obstacle_pose_mutex); // Usa il mutex definito in main
            if (new_obstacle_data_available) {                     // Flag definito in main e aggiornato dal callback
                o_k_measured_for_observer << latest_obstacle_pose_msg.pose.position.x,
                                             latest_obstacle_pose_msg.pose.position.y,
                                             latest_obstacle_pose_msg.pose.position.z;
                new_obstacle_data_available = false;
                use_this_measurement = true;
            } else if (observer_is_initialized) {           // Usa le variabili di stato dell'osservatore definite in main
                o_k_measured_for_observer = observer_o_hat; // Fallback: usa l'ultima stima come "misura"
                RCLCPP_DEBUG(logger, "Ciclo MPC t=%d: No new obstacle pose, using last estimate for observer input.", t);
                use_this_measurement = true;                // Ho comunque un input per l'osservatore
            } else {
                RCLCPP_WARN(logger, "Ciclo MPC t=%d: Observer not initialized and no obstacle data! Setting far.", t);
                o_k_measured_for_observer = Eigen::Vector3d(100.0, 100.0, 100.0); // Fallback di emergenza
                use_this_measurement = true;                // Bisogna comunque inizializzare l'osservatore
            }
        }

        // Aggiorna le stime dell'osservatore se abbiamo un input valido
        if (use_this_measurement || !observer_is_initialized) { // Chiama sempre se non inizializzato per il primo set
          updateObstacleEstimate(o_k_measured_for_observer,
                                 obs_ts, obs_alpha1, obs_alpha2, obs_alpha3,
                                 observer_o_hat, observer_v_hat, observer_a_hat, observer_is_initialized);
        }
        // Ora observer_o_hat, observer_v_hat, observer_a_hat contengono le stime più recenti

        // Predico la traiettoria dell'ostacolo per l'orizzonte N
        std::vector<Eigen::Vector3d> predicted_o_horizon_eigen =
            predictObstacleTrajectory(observer_o_hat, observer_v_hat, observer_a_hat, N, obs_ts);

        // Converto la traiettoria predetta dell'ostacolo in un casadi::DM appiattito
        std::vector<double> flattened_o_pred_vec;
        flattened_o_pred_vec.reserve(3 * (N+1));
        if (observer_is_initialized && !predicted_o_horizon_eigen.empty()) { // Mi assicuro che ci sia qualcosa da convertire
            for (const auto& pos_o : predicted_o_horizon_eigen) {
                flattened_o_pred_vec.push_back(pos_o.x());
                flattened_o_pred_vec.push_back(pos_o.y());
                flattened_o_pred_vec.push_back(pos_o.z());
            }
        } else { // Se l'osservatore non è inizializzato o la predizione è vuota, invia posizioni "lontane"
            RCLCPP_WARN(logger, "Ciclo MPC t=%d: Observer not ready or empty prediction, sending far obstacle positions.", t);
            for(int i=0; i<N+1; ++i) {
                flattened_o_pred_vec.push_back(100.0); // ox_far
                flattened_o_pred_vec.push_back(100.0); // oy_far
                flattened_o_pred_vec.push_back(100.0); // oz_far
            }
        }
        casadi::DM O_pred_flat_dm(flattened_o_pred_vec);

        // Log per VERIFICARE l'osservatore e la predizione (MOLTO IMPORTANTE)
        RCLCPP_INFO(logger, "Ciclo t=%d: o_misurato_usato_da_obs [%.3f, %.3f, %.3f]", t,
                    o_k_measured_for_observer.x(), o_k_measured_for_observer.y(), o_k_measured_for_observer.z());
        RCLCPP_INFO(logger, "         o_stimato_attuale [%.3f, %.3f, %.3f]",
                    observer_o_hat.x(), observer_o_hat.y(), observer_o_hat.z());
        RCLCPP_INFO(logger, "         v_stimato_attuale [%.3f, %.3f, %.3f]",
                    observer_v_hat.x(), observer_v_hat.y(), observer_v_hat.z());
        RCLCPP_INFO(logger, "         a_stimato_attuale [%.3f, %.3f, %.3f]",
                    observer_a_hat.x(), observer_a_hat.y(), observer_a_hat.z());
        if (!predicted_o_horizon_eigen.empty()){
            RCLCPP_INFO(logger, "         o_pred_orizzonte[1] (per t+dt) [%.3f, %.3f, %.3f]",
                        predicted_o_horizon_eigen[1].x(), predicted_o_horizon_eigen[1].y(), predicted_o_horizon_eigen[1].z());
        }

        //--------------------------------LOG---------------------------------------------------------------

        // 1. Posizione di riferimento (x_ref, y_ref, z_ref) per il PRIMO passo dell'orizzonte
        std::vector<double> p_ref_primo_passo_vec = {
            reference_ee_traj[t][0], reference_ee_traj[t][1], reference_ee_traj[t][2]
        };
        RCLCPP_INFO(logger, "p_ref (attuale, t=%d): [%.4f, %.4f, %.4f]", t,
            p_ref_primo_passo_vec[0], p_ref_primo_passo_vec[1], p_ref_primo_passo_vec[2]);

        // 2. Posizione attuale del robot (predetta per il PRIMO passo)
        std::vector<double> p_pred_attuale_vec(3);
        for(int j=0; j<3; ++j) {
            p_pred_attuale_vec[j] = static_cast<double>(x_e_current(j));
        }
        RCLCPP_INFO(logger, "p_pred (attuale del robot): [%.4f, %.4f, %.4f]",
            p_pred_attuale_vec[0], p_pred_attuale_vec[1], p_pred_attuale_vec[2]);

        // 3. Quaternione di riferimento per il PRIMO passo dell'orizzonte (i=0 simbolico)
        std::vector<double> q_ref_primo_passo_vec = {
            reference_ee_traj[t][3], reference_ee_traj[t][4],
            reference_ee_traj[t][5], reference_ee_traj[t][6]
        };
        RCLCPP_INFO(logger, "q_ref (attuale, t=%d): [%.4f, %.4f, %.4f, %.4f]", t,
            q_ref_primo_passo_vec[0], q_ref_primo_passo_vec[1],
            q_ref_primo_passo_vec[2], q_ref_primo_passo_vec[3]);

        // 4. Quaternione attuale del robot (predetto per il PRIMO passo, i=0 simbolico)
        std::vector<double> q_pred_attuale_vec(4);
        for(int j=0; j<4; ++j) {
            q_pred_attuale_vec[j] = static_cast<double>(x_e_current(3+j));
        }
        RCLCPP_INFO(logger, "q_pred (attuale del robot): [%.4f, %.4f, %.4f, %.4f]",
            q_pred_attuale_vec[0], q_pred_attuale_vec[1],
            q_pred_attuale_vec[2], q_pred_attuale_vec[3]);

        //--------------------------------------------------------------------------------------------------^

        std::map<std::string, DM> arg;
        arg["x0"] = previous_solution;  // Usa la soluzione del ciclo precedente come punto di partenza
        arg["p"] = DM::vertcat({x_current,          // 13x1
                        reshape(Jp_live_ee, 18, 1), // 18x1 avendo 6 DoF
                        reshape(Jo_live_ee, 18, 1), // 18x1 avendo 6 DoF
                        reference_ee_traj_dm,       // 7N x 1
                        O_pred_flat_dm});           // 3(N+1) x 1 TRAIETTORIA PREDETTA DELL'OSTACOLO PASSATA COME PARAMETRO
        arg["lbg"] = DM(lbg_vec);
        arg["ubg"] = DM(ubg_vec);

        try {
            auto solver_start = steady_clock.now();   // DEBUG
            auto result = solver(arg);
            auto solver_end = steady_clock.now();     // DEBUG
            auto solver_duration_ms = (solver_end - solver_start).nanoseconds() / 1'000'000;   // DEBUG

            RCLCPP_INFO(logger, "Tempo solver: %ld ms", solver_duration_ms); // log tempo solver

            DM sol = result.at("x");
            previous_solution = sol;  // Salva la soluzione corrente per il prossimo ciclo - warm-starting
            DM u0 = reshape(sol(Slice(0, 6)), 6, 1); // u_k
            last_u = u0; // salva per il prossimo ciclo

            // Stampa il primo valore di gamma dell'orizzonte (gamma_{0|k})
            RCLCPP_INFO(logger, " gamma_ee_optimale[0] (per t+dt): %.4f", static_cast<double>(sol(num_U_vars + num_X_vars)));  // AGGIUNTO

            // Logica di arresto
            if (t == max_timesteps - 1) {
                // Azzera velocità
                std_msgs::msg::Float64MultiArray zero_vel_msg;
                zero_vel_msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                velocity_pub->publish(zero_vel_msg);
                RCLCPP_INFO(logger, "Velocità nulle pubblicate. Robot fermato.");
                RCLCPP_WARN(logger, "Massimo numero di timestep raggiunto (%d). Arresto MPC.", max_timesteps);
            }
        } catch (std::exception &e) {
            RCLCPP_ERROR(logger, "Ottimizzazione fallita al timestep %d: %s", t, e.what());
            RCLCPP_ERROR(logger, "Parametri del solver durante il fallimento:");
            for (int i = 0; i < arg["p"].numel(); ++i) {
                RCLCPP_ERROR(logger, "  p[%d] = %.3f", i, static_cast<double>(arg["p"](i)));
            }
            // Azzera velocità
            std_msgs::msg::Float64MultiArray zero_vel_msg;
            zero_vel_msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            velocity_pub->publish(zero_vel_msg);
            RCLCPP_INFO(logger, "Velocità nulle pubblicate. Robot fermato.");
            break;
        }
    }

    // Attendi che il thread spinner finisca
    spinner.join();

    return 0;
}
