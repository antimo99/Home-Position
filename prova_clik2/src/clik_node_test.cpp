#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <Eigen/Dense>
#include <stdio.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// tf2
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/float64_multi_array.hpp>

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

class CLIKNode : public rclcpp::Node
{
    protected:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
        rclcpp::TimerBase::SharedPtr clik_timer_;
        rclcpp::TimerBase::SharedPtr timer_temp;
        // Variabili di controllo
        double Ts_ = 0.001;             // <-- periodo di campionamento del clik
        double clik_gain_ = 500; //1/Ts_; //1.0 / Ts_;  // <-- guadagno del clik

        std::string ROBOT_MODEL_GROUP = "panda_arm";
        std::string EE_LINK_ = "panda_link7";

        // il RobotModelLoader serve per caricare il modello da URDF con la libreria MoveIt!
        // è usato per costruire l'oggetto RobotState kinematic_state_
        std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
        rclcpp::Node::SharedPtr robot_loader_node_;

        const moveit::core::JointModelGroup* joint_model_group_;
        const moveit::core::LinkModel* last_link_;

        // lo stato cinematico del robot, possiamo leggerlo come lo stato dell'algoritmo CLIK (le q - variabili di giunto)
        moveit::core::RobotStatePtr kinematic_state_;

        // oldQ usato per la continuità del quaternione, Inizializzato all'identità.
        Eigen::Quaterniond oldQuaternion_ = Eigen::Quaterniond::Identity();

        Eigen::Vector3d reference_point_position_ = Eigen::Vector3d::Zero();  

        //trasformata terna racchetta in terna flangia
        Eigen::Matrix4d T_7_racchetta;
        Eigen::Matrix4d b_T_rDesiderata;

        //Definisco il vettore per la posizione desiderata    
        Eigen::Vector3d position_des_;
        Eigen::Quaterniond quaternion_des_;

        //subscriber per il topic /desired_pose
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;

        //variabile in cui salvo i vari punti della traiettoria 
        geometry_msgs::msg::PoseStamped pose_desired;
        bool gestore,var;

        //tf2
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener; 


    public:
            CLIKNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("clik", opt)
            { 
                using namespace std::placeholders;
                robot_loader_node_ = std::make_shared<rclcpp::Node>(
                    "robot_model_loader", rclcpp::NodeOptions(opt).automatically_declare_parameters_from_overrides(true));
                robot_model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(robot_loader_node_);
                const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader_->getModel();
                RCLCPP_INFO(this->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());
                joint_model_group_ = kinematic_model->getJointModelGroup(ROBOT_MODEL_GROUP);
                kinematic_state_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(kinematic_model));
                last_link_ = kinematic_state_->getLinkModel(EE_LINK_);

                T_7_racchetta<< 1, 0, 0, -0.028, 
                                0, 1, 0, 0,
                                0, 0, 1, 0.3045,
                                0, 0, 0, 1.0; 
        

                // Il publisher per i comandi in giunto
                cmd_pub_=this->create_publisher<sensor_msgs::msg::JointState>("cmd/joint_position",1);
                vel_pub_=this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_cmd",1);
                // timer of the main control loop
                // uso un timer che garantisce la giusto frequenza al clik
                rclcpp::Duration period = rclcpp::Duration::from_seconds(Ts_);
                clik_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1)),
                                                    std::bind(&CLIKNode::clik_control_cycle, this));
                
                clik_timer_->cancel();

                gestore=false;
                var=false;
                
                //Subscriber per il topic /desired_pose
                subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                                 "/desired_pose", 10, std::bind(&CLIKNode::sub_callback, this, _1));
                
                //istanzio elementi per l'uso del topic tf
	              tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	              tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
                
                RCLCPP_INFO(this->get_logger(), "CLIK Node started");

            }

            ~CLIKNode() = default;

    protected:
           void sub_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
            {
                using namespace std::chrono_literals;
                pose_desired=*msg;
                //std::cout<<"POSA LETTA: "<<pose_desired.pose.orientation.w<<","<<pose_desired.pose.orientation.x<<","<<pose_desired.pose.orientation.y<<", "<<pose_desired.pose.orientation.z<<std::endl;
                if(gestore==false)
                {
                    gestore=true;

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
                    
                    Eigen::Vector3d translation(trasformata.transforms[0].transform.translation.x,trasformata.transforms[0].transform.translation.y,trasformata.transforms[0].transform.translation.z);
                    Eigen::Quaterniond quate(trasformata.transforms[0].transform.rotation.w,trasformata.transforms[0].transform.rotation.x,trasformata.transforms[0].transform.rotation.y,trasformata.transforms[0].transform.rotation.z);
                    //std::cout<<" INIZIALE: Translation: "<<translation<<" Rotation: "<<quate<<std::endl;
                    Eigen::Matrix3d b_R_7=quate.toRotationMatrix();
                    Eigen::Matrix4d b_T_7;
                    b_T_7.block<3,3>(0,0)=b_R_7;
                    b_T_7.block<3,1>(0,3)=translation;
                    b_T_7.block<1,4>(3,0)=Eigen::Vector4d(0,0,0,1);
                    Eigen::Matrix4d b_T_rinit=b_T_7*T_7_racchetta;
                    std::cout<<"T_INI: "<<b_T_rinit<<std::endl;
                    std::cout<<std::endl;

                    clik_timer_->reset();

                }
            }

            void clik_control_cycle()
            {
              using namespace std::chrono_literals;
              if(var==false)
              {
                var=true;
                sensor_msgs::msg::JointState joint_current;
                if (!rclcpp::wait_for_message<sensor_msgs::msg::JointState>(joint_current, shared_from_this(), "joint_states", 10s))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to get current joint state");
                    return;
                }

                // aggiorno la variabile globale kinematic_state con i valori letti delle variabili di giunto
                for (size_t i=0; i<7; ++i)
                {
                    kinematic_state_->setJointPositions(joint_current.name[i], &joint_current.position[i]);
                }
              }
                const Eigen::Isometry3d& b_T_7 = kinematic_state_->getGlobalLinkTransform(last_link_);
                
                Eigen::Vector3d position_temp = b_T_7.translation();
                Eigen::Matrix3d rotation_temp_ = b_T_7.rotation();

                Eigen::Matrix4d b_T_7_matrix;
                b_T_7_matrix.block<3,3>(0,0)=rotation_temp_;
                b_T_7_matrix.block<3,1>(0,3)=position_temp;
                b_T_7_matrix.block<1,4>(3,0)=Eigen::Vector4d(0,0,0,1);

                Eigen::Matrix4d T_b_r=b_T_7_matrix*T_7_racchetta;

                // Estraggo la posizione
                Eigen::Vector3d position = T_b_r.block<3,1>(0,3);
                // Estraggo il quaternione
                Eigen::Matrix3d rotation = T_b_r.block<3,3>(0,0);
                Eigen::Quaterniond quaternion(rotation);

                // Assicuro la continuità del quaternione
                quaternion = quaternionContinuity(quaternion, oldQuaternion_);
                //std::cout<<"QUESTO: "<<quaternion<<std::endl;
                oldQuaternion_ = quaternion;  // <-- per il prissimo ciclo

                Eigen::Matrix<double, 6, 1> error;
                Eigen::Vector3d p_des(pose_desired.pose.position.x,pose_desired.pose.position.y,pose_desired.pose.position.z);
                error.block<3, 1>(0, 0) = p_des - position;

                // Errore in orientamento è la parte vettoriale di Qd*inv(Q)
                Eigen::Quaterniond Qd(pose_desired.pose.orientation.w,pose_desired.pose.orientation.x,pose_desired.pose.orientation.y,pose_desired.pose.orientation.z);
                Eigen::Quaterniond deltaQ = Qd * quaternion.inverse();

                // errore in orientamento (setto il blocco di dimensioni 3x1 che parte dalla posizione (3,0))
                error.block<3, 1>(3, 0) = deltaQ.vec();

                std::cout<<"ERRORE: ["<<error(0,0)<<", "<<error(1,0)<<", "<<error(2,0)<<", "<<error(3,0)<<", "<<error(4,0)<<", "<<error(5,0)<<std::endl;
                std::cout<<std::endl;

                //vel_e è il termine (v_des + gamma*error)
                Eigen::Matrix<double, 6, 1> vel_e =clik_gain_ * error;

                Eigen::MatrixXd jacobian;
                kinematic_state_->getJacobian(joint_model_group_, last_link_, reference_point_position_, jacobian);

                // Calcolo pinv(jacobian)*vel_e, ovvero q_dot
                Eigen::VectorXd q_dot = jacobian.completeOrthogonalDecomposition().solve(vel_e);
                std_msgs::msg::Float64MultiArray vel;
                vel.data.resize(7);
                for(int i=0; i<(int)vel.data.size();i++)
                {
                  vel.data[i]=q_dot[i];
                }

                //std::cout<<"Q_DOT: "<<q_dot<<std::endl;

                Eigen::VectorXd q;
                kinematic_state_->copyJointGroupPositions(joint_model_group_, q);
                //std::cout<<"Q0: "<<q<<std::endl;
                q = q + q_dot * Ts_; 
                //std::cout<<"Q1: "<<q<<std::endl;
                kinematic_state_->setJointGroupPositions(joint_model_group_, q);

                sensor_msgs::msg::JointState out_msg;
                kinematic_state_->copyJointGroupPositions(joint_model_group_, out_msg.position);
                // infine copio i nomi dei giunti dal joint_model_group (PASSAGGIO FONDAMENTALE)
                out_msg.name = joint_model_group_->getActiveJointModelNames();

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
                    
                    Eigen::Vector3d translation2(trasformata.transforms[0].transform.translation.x,trasformata.transforms[0].transform.translation.y,trasformata.transforms[0].transform.translation.z);
                    Eigen::Quaterniond quate2(trasformata.transforms[0].transform.rotation.w,trasformata.transforms[0].transform.rotation.x,trasformata.transforms[0].transform.rotation.y,trasformata.transforms[0].transform.rotation.z);
                    
                    Eigen::Matrix3d b_R_7fin=quate2.toRotationMatrix();
                    Eigen::Matrix4d b_T_7fin;
                    b_T_7fin.block<3,3>(0,0)=b_R_7fin;
                    b_T_7fin.block<3,1>(0,3)=translation2;
                    b_T_7fin.block<1,4>(3,0)=Eigen::Vector4d(0,0,0,1);
                    Eigen::Matrix4d b_T_rfin=b_T_7fin*T_7_racchetta;
                    std::cout<<"T_FIN: "<<b_T_rfin<<std::endl;
                    std::cout<<std::endl;

                cmd_pub_->publish(out_msg);
                vel_pub_->publish(vel);


            }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CLIKNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


