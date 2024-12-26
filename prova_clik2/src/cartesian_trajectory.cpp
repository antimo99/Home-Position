#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <Eigen/Dense>

//Quintico
#include "prova_clik2/quintic.h"

// tf2
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

Eigen::Quaterniond quaternionContinuity(const Eigen::Quaterniond& q, const Eigen::Quaterniond& oldQ)
{
  auto tmp = q.vec().transpose() * oldQ.vec();
  if (tmp < -0.01)
  {
    Eigen::Quaterniond out(q);
    out.vec() = -out.vec();
    out.w() = -out.w();
    return out;
  }
  return q;
}

class CartesianTraj : public rclcpp::Node
{
    protected:
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;



        // oldQ usato per la continuità del quaternione, Inizializzato all'identità.
        Eigen::Quaterniond oldQuaternion_ = Eigen::Quaterniond::Identity();

        //trasformata terna racchetta in terna flangia
        Eigen::Matrix4d T_7_racchetta;
        Eigen::Matrix4d b_T_rDesiderata;

        //Definisco il vettore per la posizione desiderata    
        Eigen::Vector3d position_des_;
        Eigen::Quaterniond quaternion_des_;

        rclcpp::TimerBase::SharedPtr timer_temp;

        //tf2
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener; 


    public:
            CartesianTraj(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("CartesianTraj", opt)
            { 
                using namespace std::placeholders;
               
                T_7_racchetta<< 1, 0, 0, -0.028, 
                                0, 1, 0, 0,
                                0, 0, 1, 0.3045,
                                0, 0, 0, 1.0; 
                
                // Il publisher per i punti della traiettoria su /desired_pose
                pub_=this->create_publisher<geometry_msgs::msg::PoseStamped>("/desired_pose",1);

                //Leggo una sola volta da joint_states, poi fermo il timer
                timer_temp = this->create_wall_timer(std::chrono::milliseconds((int)(3000)),
                                                    std::bind(&CartesianTraj::timer_temp_cb, this));
                
                //istanzio elementi per l'uso del topic tf
	            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
                
            }

            ~CartesianTraj() = default;

    protected:
            void timer_temp_cb()
            { 

                timer_temp->cancel(); 
                using namespace std::chrono_literals;

                sensor_msgs::msg::JointState joint_current;
                if (!rclcpp::wait_for_message<sensor_msgs::msg::JointState>(joint_current, shared_from_this(), "joint_states", 10s))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to get current joint state");
                    return;
                }


 

                tf2_msgs::msg::TFMessage trasformata; 

                trasformata.transforms.resize(1);

                //calcolo la trasformata
                try
                {
                    trasformata.transforms[0] = tf_buffer->lookupTransform("panda_link0","panda_link7", tf2::TimePointZero, 5s);
                }
                catch (tf2::TransformException& ex)
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "ERRORE NEL CALCOLO DELLA 1° TRASFORMATA " << ex.what());
                }
                
                Eigen::Quaterniond quat_tmp(trasformata.transforms[0].transform.rotation.w,trasformata.transforms[0].transform.rotation.x,trasformata.transforms[0].transform.rotation.y,trasformata.transforms[0].transform.rotation.z);
                quat_tmp.normalize();
                Eigen::Matrix3d b_R_7=quat_tmp.toRotationMatrix();
                Eigen::Vector3d b_P_7(trasformata.transforms[0].transform.translation.x,trasformata.transforms[0].transform.translation.y,trasformata.transforms[0].transform.translation.z);

                Eigen::Matrix4d b_T_7_matrix;
                b_T_7_matrix.block<3,3>(0,0)=b_R_7;
                b_T_7_matrix.block<3,1>(0,3)=b_P_7;
                b_T_7_matrix.block<1,4>(3,0)=Eigen::Vector4d(0,0,0,1);

                Eigen::Matrix4d T_b_r=b_T_7_matrix*T_7_racchetta;
                Eigen::Matrix3d b_R_rinit=T_b_r.block<3,3>(0,0);
                Eigen::Quaterniond b_q_rinit(b_R_rinit);
                b_q_rinit.normalize();
                Eigen::Vector3d b_P_rDesiderata=T_b_r.block<3,1>(0,3);

                //Costruisco la matrice di rotazione racchetta deisderata in tena base che sarà quella di init
                Eigen::Matrix3d b_R_rDesiderata;
                b_R_rDesiderata << 0, 0, -1,
                                  0, -1, 0,
                                  -1, 0, 0;

                //Salvo queste informazioni nella variabile (matrice) globale b_T_rDesiderata per usarla nel clik
                b_T_rDesiderata.block<3,3>(0,0)=b_R_rDesiderata;
                b_T_rDesiderata.block<3,1>(0,3)=b_P_rDesiderata;
                b_T_rDesiderata.block<1,4>(3,0)=Eigen::Vector4d(0,0,0,1);

                // Estraggo la posizione
                position_des_ = b_T_rDesiderata.block<3,1>(0,3);

                // Estraggo il quaternione
                Eigen::Matrix3d rotation_des_ = b_T_rDesiderata.block<3,3>(0,0);
                quaternion_des_=Eigen::Quaterniond(rotation_des_);
                quaternion_des_.normalize();

                //Genero la traiettoria cartesiana in orientamento tra l'orientamento di partenza e quello desiderato
                rclcpp::Time t0 = this->now();
                rclcpp::Duration t(0, 0);  // inizializzo t=0;

                // La nostra traiettoria sarà pubblicata con una certa frequenza.
                // Qui ho impostato 1000 Hz
                rclcpp::Rate loop_rate(1000.0); //va bene 1000 Hz?

                // Estraggo la duration in secondi
                double traj_duration = 5.0;

                Eigen::Quaterniond old=b_q_rinit;
                while (rclcpp::ok() && t <= rclcpp::Duration::from_seconds(5.0))
                {
                    // calcolo il nuovo t
                    t = this->now() - t0;

                    geometry_msgs::msg::PoseStamped out_msg; 
                    Eigen::Quaterniond deltaQ;

                    //Q(t)=Qi*deltaQ(t) dove deltaQ ha componenti generate con chiamate del quintico ed è tale per cui
                    //all'istante t=0 vale I (ossia il quaternione [1,0,0,0]) e all'istante tf vale Qi^-1*Qf;
                    //quaternion_des_=b_q_rinit;
                    
                    Eigen::Quaterniond quat_temp=b_q_rinit.inverse()*quaternion_des_;
    
                    deltaQ.w()=quintic(t.seconds(), 1, quat_temp.w(),  traj_duration);
                    deltaQ.x()=quintic(t.seconds(), 0, quat_temp.x(),  traj_duration);
                    deltaQ.y()=quintic(t.seconds(), 0, quat_temp.y(),  traj_duration);
                    deltaQ.z()=quintic(t.seconds(), 0, quat_temp.z(),  traj_duration);

                    Eigen::Quaterniond quat_finale=b_q_rinit*deltaQ;
                    quat_finale=quaternionContinuity(quat_finale,old);
                    old=quat_finale;
                    out_msg.pose.orientation.w = quat_finale.w();
                    out_msg.pose.orientation.x = quat_finale.x();
                    out_msg.pose.orientation.y = quat_finale.y();
                    out_msg.pose.orientation.z = quat_finale.z();

                    // la parte di posizione la pongo pari a quella init.
                    out_msg.pose.position.x = position_des_(0);
                    out_msg.pose.position.y = position_des_(1);
                    out_msg.pose.position.z = position_des_(2);

                    // sto usando la versione Stamped del messaggio posa. Sarebbe opportuno riempire l'header.stamp (il tempo attuale
                    // del messaggio)
                    out_msg.header.stamp = this->now();

                    // publico il comando in cartesiano
                    pub_->publish(out_msg);

                    // sleep sul rate
                    loop_rate.sleep();
                }
            }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianTraj>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}