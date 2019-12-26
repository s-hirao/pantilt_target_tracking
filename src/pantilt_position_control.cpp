//ヘッダ header 
#ifndef INCLUDE_SYNCRO_IMAGE_CLASS
#define INCLUDE_SYNCRO_IMAGE_CLASS
//rosを使って制御するため
#include <ros/ros.h>
//？
#include <ros/callback_queue.h>

//時間同期したビーゴとパンチルトの命令値とdepth画像を受け取るため
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
//opencvのdepth画像をrosで使えるようにするため
// #include <cv_bridge/cv_bridge.h>

//ros msg header パンチルト指令値型
#include <std_msgs/UInt16MultiArray.h>
//ros msg header ビーゴ指令値型
// #include <beego_control/beego_encoder.h>
//ros msg header depth画像型
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
// ros msg 
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
//dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
//cfg
#include <pantilt_position_control/goal_coefficient_paramConfig.h>
//行列ライブラリ
#include <Eigen/Dense>
// selfmsg
#include <pantilt_position_control/pantiltArray.h>
// 別のcppファイル検出プログラム用
#include <obstacle_detection_2019/ClassificationVelocityData.h>
#include <obstacle_detection_2019/ClassificationElement.h>
// クラス class
class pantiltPositionControlClass{
    private:
    //nodehandle
	ros::NodeHandle nodehandle_publisher, nodehandle_pantilt_publisher, nodehandle_beego_publisher , nodehandle_subscriber ;
	//publisher
	ros::Publisher pantilt_publisher, pantilt_time_publisher , beego_publisher ;
    //subscriber
	ros::Subscriber position_control_subscriber ;

	// message_filters::Subscriber<sensor_msgs::Image> depth_subscriber;
	// 今回使わないmessage_filters::Subscriber<nav_msgs::Odometry> camera_odometry_subscriber;
	// message_filters::Subscriber<beego_control::beego_encoder> beego_encoder_subscriber;
	// message_filters::Subscriber<pantilt_position_control::pantiltArray> pantilt_order_time_subscriber;
    //message_filters
	// typedef message_filters::sync_policies::ApproximateTime</*sensor_msgs::Image,nav_msgs::Odometry,*/beego_control::beego_encoder,pantilt_position_control::pantiltArray> MySyncPolicy ;
	// message_filters::Synchronizer<MySyncPolicy> sync ;
	
	//rqt_reconfigure系
    bool rqt_reconfigure ;
    dynamic_reconfigure::Server<pantilt_position_control::goal_coefficient_paramConfig > server ;
    dynamic_reconfigure::Server<pantilt_position_control::goal_coefficient_paramConfig >::CallbackType f ;
	// msg 変数 
	geometry_msgs::Twist beegoMessage;//ビーゴ命令用
	std_msgs::UInt16MultiArray panTiltMessage;//ラテパンダ命令用
	pantilt_position_control::pantiltArray pantiltTimeMessage;//時間付きpwm角速度
	obstacle_detection_2019::ClassificationVelocityData copyTargetGravityCenterMessage;//カメラ座標からの対象物体の重心コピー用
	geometry_msgs::Point targetGravityCenterMessage;//カメラ座標からの対象物体の重心
	ros::Time time ;
	//コントローラ設計時の変数
	// 各成分
	// スタートのパンチルト姿勢
	int start_pan_angular ;
	int start_tilt_angular;
	// 係数λ
	double lambda ;
	// 目標距離rqt設定
	double distance_x ;
	double distance_y ;
	double distance_z ;
	// 実際にPWMとして入力する回転速度
	int pan_velocity; 
	int tilt_velocity;
	// 実際に指令値として入力するパン 誤差分だけ目標角度パンチルト
	int pan_angular ;
	int tilt_angular; 
	// 2-2行列
	// 二重行列
	// std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > J;
	// std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> >J_Inverse ;
	// 2-1行列
	// 二重行列
	// std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > error;
	// 2-1行列
	// std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > goal;
	// std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > output_y;
	// std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > input_u;
	// 単位行列
	Eigen::Matrix3d I;
	// error
	Eigen::Vector3d error_difference  ;//偏差 観測データと目標値の差距離
	Eigen::Vector3d error_theta;//偏差 観測データと目標値の差角度
	// goal
	Eigen::Vector3d t_g_c;//カメラ座標の対象の目標位置
	Eigen::Vector3d phi_g_c ;//カメラ座標目標回転ベクトルヨー角パンチルト命令値に使用
	// output_y
	Eigen::Vector3d t_o_c;//カメラ座標の対象の三次元位置
	Eigen::Vector3d psi_o_c ;//カメラ座標の対象の三次元姿勢回転ベクトルヨー角のみ三次元よりarctanで
	// input_u
	Eigen::Vector3d vel_c_t;//命令速度今は直進のみ
	Eigen::Vector3d omega_c_psi;//命令角度パンチルトのみ
	// rqt確認用
	// bool check;
    public:
    //コンストラクタ：クラス定義に呼び出されるメソッド
    pantiltPositionControlClass();
    //デストラクタ：クラスが消滅するときに呼びだされるメソッド
    ~pantiltPositionControlClass();
	//コントローラの係数λと目標位置と角度の設定 動的パラメータ変更
	void Config_Callback_Function(pantilt_position_control::goal_coefficient_paramConfig &config, uint32_t level);
	//メッセージフィルタよりdepthとbeegoエンコーダとcameraオドメトリとpantilt指令値を受け取る
	// void DepthBeegoCameraPan(/*const sensor_msgs::Image::ConstPtr& depthMsg,const nav_msgs::Odometry& cameraMsg, const beego_control::beego_encorder& beegoMsg,*/ const pantilt_position_control::pantiltArray& pantiltMsg);
	// 対象位置のコールバック関数
	void TargetPointCallback(const obstacle_detection_2019::ClassificationVelocityData::ConstPtr& msg);
	void calculationProcess();
	void angularCopyAndCheck();
	void goalSet();
	void sensorDataGet();
	void deviation();
	// void inverse();
	void input();
	void beegoVelocityOrder();
	void RotationToPWM();
	void plusTime();
	void publishJudgment();
	void publishBeegoData();
	void publishPantiltData();
	void publishPantiltTimeData();
};
#endif

// コールバック関数 callback_function
void pantiltPositionControlClass::Config_Callback_Function(pantilt_position_control::goal_coefficient_paramConfig &config, uint32_t level)
{
	// 係数λ
	lambda = config.p_control ;
	// スタート位置
	start_pan_angular  = config.pan_angular ;
	start_tilt_angular = config.tilt_angular ;
	// 目標三次元位置
	distance_x	 = config.distance_x ;
	distance_y	 = config.distance_y ;
	distance_z	 = config.distance_z ;
	ROS_INFO("reconfigure_ok lambda = %f" , lambda);
	ROS_INFO("reconfigure_ok start_pan_angular  = %d",start_pan_angular );
	ROS_INFO("reconfigure_ok start_tilt_angular = %d",start_tilt_angular);
	ROS_INFO("reconfigure_ok distance_x = %f",distance_x );
	ROS_INFO("reconfigure_ok distance_x = %f",distance_y );
	ROS_INFO("reconfigure_ok distance_x = %f",distance_z );
	// check = true ;
}

// void pantiltPositionControlClass::DepthBeegoCameraPan(/*const sensor_msgs::Image::ConstPtr& depthMsg,const nav_msgs::Odometry& cameraMsg,*/ const beego_control::beego_encorder& beegoMsg, const pantilt_position_control::pantiltArray& pantiltMsg);
// {
    // try{        
        // bridgeDepth = cv_bridge::toCvCopy(depthMsg,sensor_msgs::image_encodings::TYPE_32FC1);
        // ROS_INFO("callBack");
    // }
    // catch(cv_bridge::Exception& e) 
	// {
    	//エラー処理        
        // ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",        
	// depthMsg->encoding.c_str());
        // return ;
    // } 	
	// depth = bridgeDepth->image.clone();
	// ROS_INFO("depthcallBack_function");	 
	// ROS_INFO("odometry_callback_ok");
	// camera_odometry = *cameraMsg ;
	//ビーゴの速度データ角度データ
    // beego = *beegoMag ;
	// ROS_INFO("beego_callback_ok");

	// pan_angular = pantiltMsg.data[0];
	// tilt_angular = pantiltMsg.data[1];
	// pan_velocity = pantiltMsg.data[2];
	// tilt_velocity = pantiltMsg.data[3];
	// ROS_INFO("pantilt_callback_ok");
	// 関数強制脱出
	// if(check==false) return;
	// 計算プロセス
	// calculationProcess();
// }

// 関数 function
void pantiltPositionControlClass::TargetPointCallback(const obstacle_detection_2019::ClassificationVelocityData::ConstPtr& msg)
{
	// コピー
	copyTargetGravityCenterMessage = *msg ;
	// 関数強制脱出
	// if(check==false) return;
	// 計算プロセス
	calculationProcess();
}

// 計算プロセス
void pantiltPositionControlClass::calculationProcess()
{
	// targetGravityCenterMessage = copyTargetGravityCenterMessage.data[クラスタ番号].gc;//重心
	targetGravityCenterMessage = copyTargetGravityCenterMessage.data[].gc;//重心
	goalSet();
	sensorDataGet();
	deviation();
	// inverse();
	input();
	beegoVelocityOrder();
	RotationToPWM();
	angularCopyAndCheck();
	plusTime();
	publishJudgment();
} 

// 目標値設定
void pantiltPositionControlClass::goalSet()
{
	//目標値設定 対象までの相対位置と相対姿勢
	// リサイズ
	// t_g_c.resize(3) ;
	// phi_g_c .resize(3);
	// 各データの入力
	// 目標距離
	t_g_c(0) = distance_x ;
	t_g_c(1) = distance_y ;
	t_g_c(2) = distance_z ;
	//目標角度は0
	phi_g_c = Eigen::Vector3d::Zero();
	ROS_INFO("goal_position_set_ok");
}

// 出力センサデータの受け取り
void pantiltPositionControlClass::sensorDataGet()
{
	//観測データの格納 対象までの相対位置と相対姿勢
	// リサイズ
	// t_o_c.resize(3);
	// psi_o_c.resize(3);
	// 並進方向 ビーゴ
	t_o_c(0) = targetGravityCenterMessage.x ;
	t_o_c(1) = targetGravityCenterMessage.y ;
	t_o_c(2) = targetGravityCenterMessage.z ;
	// 回転方向 とれないので計算する 
	// ロールとピッチは本来0であるが誤差があるため
	// 実験よりロボットと対象が動いてない時のパンチルト動作の誤差を見て仮に0に近似する
	psi_o_c(0) = 0 ;
	psi_o_c(1) = 104;
	// 目標までのズレ
	psi_o_c(2) = atan2(t_o_c(0) , t_o_c(1));
	ROS_INFO("receive_target_position_ok");
}

// 誤差eの作成
void pantiltPositionControlClass::deviation()
{
	//偏差 観測データと目標値の差
	// error_difference.resize(3);
	// error_theta.resize(3);
	error_difference  = t_g_c - t_o_c ;
	error_theta 	  = phi_g_c - psi_o_c ;
	ROS_INFO("error_value_ok");
}

// 逆行列J-1の作成
// void pantiltPositionControlClass::inverse()
// {
	// リサイズ
	// J.resize(6,6);
	// J_Inverse.resize(6,6);
	// 行列Jに各小行列に値を入れる
	// J.block(0,0,3,3) = Eigen::Matrix3d::Identity();
	// J.block(3,1,3,3) = Eigen::Matrix3d::Identity();
	// J.block(0,1,3,1) = t_o_c; 
	// J.block(3,0,3,1) = Eigen::Matrix3d::Zero();
	// 逆行列の作成
	// J_Inverse.block(0,0,3,3) = Eigen::Matrix3d::Identity();
	// J_Inverse.block(3,1,3,3) = Eigen::Matrix3d::Identity();
	// J_Inverse.block(0,1,3,1) = t_g_c;
	// J_Inverse.block(3,0,3,1) = Eigen::Matrix3d::Zero();
	// ROS_INFO("J_Inverse_ok");
// }

// 入力uの作成
void pantiltPositionControlClass::input()
{
	// リサイズ
	// vel_c_t.resize(3) ;
	// omega_c_psi.resize(3) ;
	// -λ逆行列J-1誤差e
 	vel_c_t = -1*lambda*(I*error_difference + t_g_c.cross(error_theta));
 	omega_c_psi = -1*lambda*( Eigen::Matrix3d::Zero()*error_difference + I*error_theta);
	ROS_INFO("input_u_ok");
}

void pantiltPositionControlClass::beegoVelocityOrder()
{
	// ビーゴ命令値
	beegoMessage.linear.x = vel_c_t(0);
	beegoMessage.linear.y = vel_c_t(1);
	beegoMessage.linear.z = vel_c_t(2);
}

void pantiltPositionControlClass::RotationToPWM()
{
	// 回転ベクトルからpwmに変換及び検問
	// 回転式導出後変更する
	panTiltMessage.data[2] = pan_velocity ;
	panTiltMessage.data[3] = tilt_velocity;
	ROS_INFO("pantilt_PWM_velocity_ok");
}

void pantiltPositionControlClass::angularCopyAndCheck()
{
	// 四捨五入
	pan_angular = (int)(error_theta(2)+ 0.5); 
	// 指令値用に目標値角度コピーかつ検問停止用
    pantiltTimeMessage.pantiltArray.data.resize(4);
	// 対象との角度差左右の時場合分けをする0から180に変換
	// 右
    if(pan_angular >= 5 && pan_angular <= 90 ){	
		pan_angular += 90 ;
		pantiltTimeMessage.pantiltArray.data[0] = pan_angular;
	}
	// 左
	else if(pan_angular <= 5 && pan_angular >= -90){
		pan_angular += 90 ;
		pantiltTimeMessage.pantiltArray.data[0] = pan_angular;
	}
	// 正面
	else if(pan_angular < 5 && pan_angular >= -5 ){
		// check = false ;
	}
	// それ以外
	else {
		return ;
	}
	pantiltTimeMessage.pantiltArray.data[1] = (int)(error_theta(1)+ 0.5);
	tilt_angular = (int)(error_theta(1)+ 0.5);
	ROS_INFO("pan_angular_ok");
}

//パンチルト角度角速度のデータコピー及びタイムスタンプ付与
void pantiltPositionControlClass::plusTime()
{
// 実行した後Rvizでtfを見たものが、下記
	pantiltTimeMessage.header.frame_id = "pantilt_command" ;
	pantiltTimeMessage.header.stamp = ros::Time::now() ;
	pantiltTimeMessage.pantiltArray.data.resize(4);
	pantiltTimeMessage.pantiltArray.data[0] = pan_angular ;//入力角度値 z軸
	pantiltTimeMessage.pantiltArray.data[1] = tilt_angular;//入力角度値 y軸
	pantiltTimeMessage.pantiltArray.data[2] = panTiltMessage.data[2];//入力回転速度値 z軸
	pantiltTimeMessage.pantiltArray.data[3] = panTiltMessage.data[3];//入力回転速度値 y軸
	ROS_INFO("pantilt_time_copy_ok");
}

//publish判定を行う
void  pantiltPositionControlClass::publishJudgment()
{
    // if(check==false) return;//関数強制脱出
	publishBeegoData();
    publishPantiltData();//パンチルト指令値publish
	publishPantiltTimeData();//パンチルトとタイムスタンプをpublishする
}

void pantiltPositionControlClass::publishBeegoData()
{
	ROS_INFO("beego_publish_ok");
	beego_publisher.publish(beegoMessage);//ビーゴ命令値をpublishする
}

//パンチルト角度角速度publish
void pantiltPositionControlClass::publishPantiltData()
{
    ROS_INFO("pantilt_publish_ok");
	pantilt_publisher.publish(panTiltMessage);//パンチルト指令値をpublishする
}

//タイムスタンプ付きパンチルト角度角速度publish
void pantiltPositionControlClass::publishPantiltTimeData()
{
    ROS_INFO("pantilt_edit_publish_ok");
	pantilt_time_publisher.publish(pantiltTimeMessage);//パンチルトとタイムスタンプをpublishする
}

//constructor
pantiltPositionControlClass::pantiltPositionControlClass()
:/*depth_subscriber(nodehandle_subscriber, "/zed/zed_node/depth/depth_registered", 1)
,camera_odometry_subscriber(nodehandle_subscriber,"/zed/zed_node/odom",1)
,pantilt_subscriber(nodehandle_subscriber,"PanTiltPlusTimeData",1)
,sync(MySyncPolicy(10),rgb_image_subscriber, depth_image_subscriber, pantilt_subscriber)
,check(false)
,*/lambda(0.0),distance_x(0.0),distance_y(0.0),distance_z(0.0) 
,start_pan_angular(90),start_tilt_angular(104)
,pan_angular(90), tilt_angular(104),pan_velocity(0),tilt_velocity(0)
,time(ros::Time::now())
{
	position_control_subscriber = nodehandle_subscriber.subscribe("classificationDataEstimateVelocity", 1, &pantiltPositionControlClass::TargetPointCallback,this);
	pantilt_publisher = nodehandle_pantilt_publisher.advertise<std_msgs::UInt16MultiArray>("PanTilt",1);
	pantilt_time_publisher = nodehandle_publisher.advertise<pantilt_position_control::pantiltArray>("PanTiltPlusTimeData",1);
	beego_publisher = nodehandle_beego_publisher.advertise<geometry_msgs::Twist>("beego/cmd_vel",1);

	// sync.registerCallback(boost::bind(&, this,_1, _2, _3));//, _4));
	
	f = boost::bind(&pantiltPositionControlClass::Config_Callback_Function, this,  _1, _2);
	server.setCallback(f);

	// J = std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> >::Zero(2,2);
	// J_Inverse = std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> >::Zero(2,2);
	// error = std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> >::Zero(2,1);
	// goal = std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> >::Zero(2,1);
	// output_y = std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> >::Zero(2,1);
	// input_u = std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> >::Zero(2,1);
	t_g_c = Eigen::Vector3d::Zero(3);
	phi_g_c = Eigen::Vector3d::Zero(3);
	t_o_c = Eigen::Vector3d::Zero(3);
	psi_o_c = Eigen::Vector3d::Zero(3);
	vel_c_t = Eigen::Vector3d::Zero(3);
	omega_c_psi = Eigen::Vector3d::Zero(3);	
	I = Eigen::MatrixXd::Identity(3,3); 
}

pantiltPositionControlClass::~pantiltPositionControlClass()
{
}

//main関数
int main(int argc, char **argv)
{
	ros::init(argc,argv,"pantilt_position_control_node");
	pantiltPositionControlClass pantilt_position_control_class_declaration; // pantiltPositionControlClassを新しい型として宣言
ros::Rate rate(60);
while(ros::ok())
	{
		pantilt_position_control_class_declaration.publishBeegoData();
		pantilt_position_control_class_declaration.publishPantiltData();
		pantilt_position_control_class_declaration.publishBeegoData();
		ros::spinOnce();//ros spin １回分
		rate.sleep();
	}	
	return 0;
}