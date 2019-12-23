
//ヘッダ　header 
#ifndef INCLUDE_SYNCRO_IMAGE_CLASS
#define INCLUDE_SYNCRO_IMAGE_CLASS
//rosを使って制御するため
#include <ros/ros.h>
//？
#include <ros/callback_queue.h>
//時間同期したビーゴとパンチルトの命令値とdepth画像を受け取るため
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//opencvのdepth画像をrosで使えるようにするため
#include <cv_bridge/cv_bridge.h>
//ros msg header パンチルト指令値型
#include <std_msgs/UInt16MultiArray.h>
//ros msg header ビーゴ指令値型
#include <beego_control/beego_encoder.h>
//ros msg header depth画像型
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
//dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
//念の為　行列ライブラリ
// #include <eigen3/Eigen/Dense>
// selfmsg
#include <pantilt_position_control/pantiltArray.h>

// クラス　class
class pantiltPositionControlClass{
    private:
    //nodehandle
	ros::NodeHandle nodehandle_publisher , nodehandle_subscriber;
	//publisher
	ros::Publisher pantilt_publisher ,beego_publisher ;
    //subscriber
	message_filters::Subscriber<sensor_msgs::Image> depth_subscriber;
	// 今回使わないmessage_filters::Subscriber<nav_msgs::Odometry> camera_odometry_subscriber;
	message_filters::Subscriber<beego_control::beego_encoder> beego_encoder_subscriber;
	message_filters::Subscriber<pantilt_position_control::pantiltArray> pantilt_order_time_subscriber;
    //message_filters
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,/*nav_msgs::Odometry,*/beego_control::beego_encoder,pantilt_position_control::pantiltArray> MySyncPolicy ;
	message_filters::Synchronizer<MySyncPolicy> sync ;
	// msg 変数 
	ビーゴ命令値の型わかんない　beegoMessage;//ビーゴ命令用
	std_msgs::UInt16MultiArray panTiltMessage;//ラテパンダ命令用
	pantilt_position_control::pantiltArray pantiltTimeMessage;//時間付きpwm角速度
	対象の三次元位置わからない　targetMsg;//

	//コントローラ設計時の変数
	// 2-2行列
	// 二重行列
	std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > J;
	std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> >J_Inverse ;
	// 2-1行列
	// 二重行列
	std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > error;
	// 2-1行列
	std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > goal;
	std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > output_y;
	std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > input_u;
	// 単位行列
	// Eigen::MatrixXd I;
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
	// 誤差分だけ目標角度パンチルト

	int pan_angular ;
	int tilt_angular; 
	int pan_velocity; 
	int tilt_velocity;
	// 実際に指令値として入力するパン
	int order_pan ;
	// goal
	Eigen::VectorXd t_g_c;//目標位置
	Eigen::VectorXd phi_g_c ;//目標回転ベクトルヨー角パンチルト命令値に使用
	// output_y
	Eigen::VectorXd t_o_c;//三次元位置
	Eigen::VectorXd psi_o_c ;//三次元姿勢回転ベクトルヨー角のみ三次元よりarctanで
	// input_u
	Eigen::VectorXd vel_c_t;//命令速度今は直進
	Eigen::VectorXd omega_c_psi;//命令角度パンチルトのみ
	// rqt確認用
	bool rqt_check;
    public:
    //コンストラクタ：クラス定義に呼び出されるメソッド
    pantiltPositionControlClass();
    //デストラクタ：クラスが消滅するときに呼びだされるメソッド
    ~pantiltPositionControlClass();
	//コントローラの係数λと目標位置と角度の設定　動的パラメータ変更
	void Config_Callback_Function(pantilt_position_control::pantilt_position_reconfigure&config, uint32_t level);
	//メッセージフィルタよりdepthとbeegoエンコーダとcameraオドメトリとpantilt指令値を受け取る
	void DepthBeegoCameraPan(const sensor_msgs::Image::ConstPtr& depthMsg,/*const nav_msgs::Odometry& cameraMsg,*/ const beego_control::beego_encorder& beegoMsg, const pantilt_position_control::pantiltArray& pantiltMsg);
	void calculationProcess();
	void copyPantiltData();
	void goalSet();
	void sensorDataGet();
	void deviation();
	void inverse();
	void input();
	void RotationToPWM();
	void publishJudgment();
	void publishBeegoData();
	void publishPantiltData();
	void publishPantiltTimeData();
}
#endif

// コールバック関数　callback_function
void pantiltPositionControlClass::Config_Callback_Function(pantilt_position_control::pantilt_position_reconfigure &config, uint32_t level)
{
	lambda 		 = config.p_control ;
	start_pan_angular  = config.pan_angular ;
	start_tilt_angular = config.tilt_angular ;
	distance_x	 = config.distance_x ;
	distance_y	 = config.distance_y ;
	distance_z	 = config.distance_z ;
	ROS_INFO("reconfigure_ok lambda = %f" , lambda);
	ROS_INFO("reconfigure_ok start_pan_angular  = %f",start_pan_angular );
	ROS_INFO("reconfigure_ok start_tilt_angular = %f",start_tilt_angular);
	ROS_INFO("reconfigure_ok distance_x = %f",distance_x );
	ROS_INFO("reconfigure_ok distance_x = %f",distance_y );
	ROS_INFO("reconfigure_ok distance_x = %f",distance_z );
	rqt_check = true ;
}

void pantiltPositionControlClass::DepthBeegoCameraPan(const sensor_msgs::Image::ConstPtr& depthMsg,/*const nav_msgs::Odometry& cameraMsg,*/ const beego_control::beego_encorder& beegoMsg, const pantilt_position_control::pantiltArray& pantiltMsg);
{
    try{        
        bridgeDepth = cv_bridge::toCvCopy(depthMsg,sensor_msgs::image_encodings::TYPE_32FC1);
        ROS_INFO("callBack");
    }
    catch(cv_bridge::Exception& e) 
	{
    	//エラー処理        
        ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",        
	depthMsg->encoding.c_str());
        return ;
    } 	
	depth = bridgeDepth->image.clone();
	ROS_INFO("depthcallBack_function");	 

	// ROS_INFO("odometry_callback_ok");
	// camera_odometry = *cameraMsg ;
	//ビーゴの速度データ角度データ
    beego = *beegoMag ;
	ROS_INFO("beego_callback_ok");
	pan_angular = pantiltMsg.data[0];
	tilt_angular = pantiltMsg.data[1];
	pan_velocity = pantiltMsg.data[2];
	tilt_velocity = pantiltMsg.data[3];
	ROS_INFO("pantilt_callback_ok");
	
	//関数強制脱出
	if(rqt_check==false) return;
	// 計算プロセス
	calculationProcess();
	
}

void pantiltPositionControlClass::calculationProcess()
{
	copyPantiltData();
	goalSet();
	sensorDataGet()
	deviation();
	inverse();
	input();
	RotationToPWM();
	publishJudgment();
} 

// 関数　function
// 目標値設定
void pantiltPositionControlClass::goalSet()
{
	//目標値設定 対象までの相対位置と相対姿勢
	// リサイズ
	t_g_c.resize(3) ;
	phi_g_c .resize(3);
	// 各データの入力
	// 目標距離
	t_g_c(0) = distance_x ;
	t_g_c(1) = distance_y ;
	t_g_c(2) = distance_z ;
	//目標角度は0
	phi_g_c = VectorXd::Zero();
	ROS_INFO("goal_position_set_ok");
}

// 出力センサデータの受け取り
void pantiltPositionControlClass::sensorDataGet()
{
	//観測データの格納　対象までの相対位置と相対姿勢
	// リサイズ
	t_o_c.resize(3);
	psi_o_c.resize(3);
	// 並進方向
	t_o_c(0) =　わからない型
	t_o_c(1) =　わからない型
	t_o_c(2) =　わからない型
	// 回転方向 とれないので計算する　
	// ロールとピッチは本来0であるが誤差があるため
	// 実験よりロボットと対象が動いてない時のパンチルト動作の誤差を見て0に近似する
	psi_o_c(0) = VectorXd::Zero();
	psi_o_c(1) = VectorXd::Zero();
	// 目標までのズレ
	psi_o_c(2) = atan2(t_o_c(0) , t_o_c(1));
	ROS_INFO("receive_target_position_ok");
}

// 誤差eの作成
void pantiltPositionControlClass::deviation()
{
	//偏差 観測データと目標値の差
	// 並進　位置がどっちが短いかによって
	if(t_g_c - t_o_c > 0){
	error.block<3,1>(0,0) = t_g_c ;
	error.block<3,1>(0,0) = t_g_c ;
	}
	else if(t_g_c - t_o_c < 0){
	error.block<3,1>(0,0) = t_g_c ;
	error.block<3,1>(0,0) = t_g_c ;
	}
	else{
	error.block<3,1>(0,0) = t_g_c ;
	error.block<3,1>(0,0) = t_g_c ;
	}
	// 回転
	if(phi_g_c - psi_g_c > 0){
	error.block<3,1>(1,0) = t_g_c ;
	error.block<3,1>(1,0) = t_g_c ;
	}
	else if(phi_g_c - psi_g_c < 0){
	error.block<3,1>(1,0) = t_g_c ;
	error.block<3,1>(1,0) = t_g_c ;
	}
	else{
	error.block<3,1>(1,0) = t_g_c ;
	error.block<3,1>(1,0) = t_g_c ;
	}
	ROS_INFO("error_value_ok");
}

// 逆行列J-1の作成
void pantiltPositionControlClass::inverse()
{
	// リサイズ
	J.resize(2,2);
	J_Inverse.resize(2,2);
	// 行列Jに各小行列に値を入れる
	J.block<3,3>(0,0) = MatrixXd::Identity(3,3);
	J.block<3,3>(1,1) = MatrixXd::Identity(3,3);
	J.block<3.1>(0,1) = 対象三次元位置; 
	J.block<3,1>(1,0) = MatrixXd::Zero(3,3);
	// 逆行列の作成
	J_Inverse.block<3,3>(0,0) = MatrixXd::Identity(3,3);
	J_Inverse.block<3,3>(1,1) = MatrixXd::Identity(3,3);
	J_Inverse.block<3.1>(0,1) = t_g_c;
	J_Inverse.block<3,1>(1,0) = MatrixXd::Zero(3,3);
	ROS_INFO("J_Inverse_ok");
}

// 入力uの作成
void pantiltPositionControlClass::input()
{
	// リサイズ
	input_u.resize(2,1);
	vel_c_t.resize(3) ;
	omega_c_psi.resize(3) ;
	// -λ逆行列J-1誤差e
	input_u.block<3,1>(0,0) = -1*lambda(J_Inverse.block<3,3>(0,0)*error.block<3,1>(0,0) + J_Inverse.block<3.1>(0,1).cross(error.block<3,1>(1,0)));
	input_u.block<3,1>(1,0) = -1*lambda(J_Inverse.block<3,1>(1,0)*error.block<3,1>(0,0) + J_Inverse.block<3,3>(1,1)*error.block<3,1>(1,0));
	// 入力uの成分取り出し
	vel_c_t = input_u.block<3,1>(0,0);
	omega_c_psi = input_u.block<3,1>(1,0);
	ROS_INFO("input_u_ok");
}

void pantiltPositionControlClass::RotationToPWM()
{
	// 回転ベクトルからpwmに変換及び検問
	回転式導出後変更する
	panTiltMessage[2];
	panTiltMessage[3];
	ROS_INFO("pantilt_PWM_velocity_ok");
}

void pantiltPositionControlClass::copyPantiltData()
{
	// 指令値用に目標値角度コピーかつ検問
    pantiltTimeMsg.pantiltArray.data.resize(4);
    if(psi_o_c(2) => 5 && psi_o_c(2) =<90 ){
		order_pan = psi_o_c(2) - psi_o_c(2)%1;
		order_pan += 90 ;
		pantiltTimeMsg.pantiltArray.data[0] = order_pan;
	}
	else if(psi_o_c(2) < 5 && psi_o_c(2) => -90){
		order_pan = psi_o_c(2) - psi_o_c(2)%1;
		order_pan += 90 ;
		pantiltTimeMsg.pantiltArray.data[0] = order_pan;
	}
	else if(psi_o_c(2) < 5.0 && psi_o_c(2) => -5.0 ){
		rqt_check = false ;
	}
	else {
		// 再計算

	}
	pantiltTimeMsg.pantiltArray.data[1] = psi_o_c(1) - psi_o_c(1)%1 ;
	ROS_INFO("pan_order_ok");
}

//パンチルト角度角速度のデータコピー及びタイムスタンプ付与
void pantiltPositionControlClass::plusTime()実行した後Rvizでtfを見たものが、下記
	pantiltTimeMessage.header.frame_id = "pantilt_command" ;
	pantiltTimeMessage.header.stamp = ros::Time::now() ;
	pantiltTimeMessage.pantiltArray.data.resize(4);
	pantiltTimeMessage.pantiltArray.data[0] = ordr_pan;//入力角度値 z軸
	pantiltTimeMessage.pantiltArray.data[1] = psi_o_c(1) - psi_o_c(1)%1;//入力角度値 y軸
	pantiltTimeMessage.pantiltArray.data[2] = panTiltMessage[2];//入力回転速度値 z軸
	pantiltTimeMessage.pantiltArray.data[3] = panTiltMessage[3];//入力回転速度値 y軸
	ROS_INFO("pantilt_time_copy_ok");
}

//publish判定を行う
void  pantiltPositionControlClass::publishJudgment()
{
    if(rqt_check==false) return;//関数強制脱出
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
:depth_subscriber(nodehandle_subscriber, "/zed/zed_node/depth/depth_registered", 1)
/*,camera_odometry_subscriber(nodehandle_subscriber,"/zed/zed_node/odom",1)*/
,pantilt_subscriber(nodehandle_subscriber,"PanTiltPlusTimeData",1)
,sync(MySyncPolicy(10),rgb_image_subscriber, depth_image_subscriber, pantilt_subscriber)
,rqt_check(false)
,lambda(0.0),distance_x(0.0),distance_y(0.0),distance_z(0.0) 
,start_pan_angular(90),start_tilt_angular(104),order_pan(0),pan_angular(90), tilt_angular(104),pan_velocity(0),tilt_velocity(0)

{
	pantilt_publisher = nodehandle_publisher.advertise<std_msgs::UInt16MultiArray>("PanTilt", 1);
	pantilt_time_publisher = nodehandle_publisher.advertise<pantilt_position_contorol::pantiltArray>("PanTiltPlusTimeData", 1);
	beego_publisher = nodehandle_publisher.advertise<>("",1);

	sync.registerCallback(boost::bind(&, this,_1, _2, _3));//, _4));
	{
		f = boost::bind(&pantiltPositionControlClass::Config_Callback_Function, this,  _1, _2);
		server.setCallback(f);
	}

	J = std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> >::Zero(2,2);
	J_Inverse = std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> >::Zero(2,2);
	error = std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> >::Zero(2,1);
	goal = std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> >::Zero(2,1);
	output_y = std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> >::Zero(2,1);
	input_u = std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> >::Zero(2,1);
	
	t_g_c = Eigen::VectorXd::Zero(3);
	phi_g_c = Eigen::VectorXd::Zero(3);
	t_o_c = Eigen::VectorXd::Zero(3);
	phi_o_c = Eigen::VectorXd::Zero(3);
	vel_c_t = Eigen::VectorXd::Zero(3);
	omega_c_psi = Eigen::VectorXd::Zero(3);	
	// I = Eigen::MatrixXd::Identity(3,3); 
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