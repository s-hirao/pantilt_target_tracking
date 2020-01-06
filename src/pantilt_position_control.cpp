//ヘッダ header 
#ifndef INCLUDE_SYNCRO_IMAGE_CLASS
#define INCLUDE_SYNCRO_IMAGE_CLASS
//rosを使って制御するため
#include <ros/ros.h>
//？
#include <ros/callback_queue.h>
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
	ros::NodeHandle nodehandle_publisher, nodehandle_pantilt_publisher, nodehandle_beego_publisher , nodehandle_target_publisher , nodehandle_subscriber ;
	//publisher
	ros::Publisher pantilt_publisher, pantilt_time_publisher , beego_publisher ,target_publisher ;
    //subscriber
	ros::Subscriber position_control_subscriber ;
	//rqt_reconfigure系
    bool rqt_reconfigure ;
    dynamic_reconfigure::Server<pantilt_position_control::goal_coefficient_paramConfig > server ;
    dynamic_reconfigure::Server<pantilt_position_control::goal_coefficient_paramConfig >::CallbackType f ;
	// msg 変数 
	geometry_msgs::Twist beegoMessage;//ビーゴ命令用
	std_msgs::UInt16MultiArray panTiltMessage;//ラテパンダ命令用
	pantilt_position_control::pantiltArray pantiltTimeMessage;//時間付きpwm角速度
	obstacle_detection_2019::ClassificationVelocityData copyTargetGravityCenterMessage;//カメラ座標からの対象物体の重心コピー用
	geometry_msgs::Twist targetGravityCenterMessage;//カメラ座標からの対象物体の重心
	ros::Time time ;
	// センサーデータの幅を求めるための変数
	// for文用
	int i ,k ,l;
	double gcMin;
	double gcCluster;
	double width ;//対象の幅x,y成分
	double widthMax ;//幅の距離
	double length ; //奥行き
	double ellipse ;//楕円
	double minRotation , maxRotation  ;// 区間
	double eps ;// 打ち切り精度
	double solution ;// 解 
	double pwm; //pwm回転速度
	double pwmCenter,center ;//
	int limit ;// 打ち切り回数
	//コントローラ設計時の変数
	// 各成分
	// スタートのパンチルト姿勢
	int start_pan_angular ;
	int start_tilt_angular;
	// pwm回転速度値
	double pwmRotationYaw;
    double pwmRotationPitch;
	// 係数λ
	double lambda ;
	// 目標距離rqt設定
	double distance_x ;
	double distance_y ;
	double distance_z ;
	// 実際にPWMとして入力する回転速度
	// int pan_velocity; 
	// int tilt_velocity;
	// 実際に指令値として入力するパン 誤差分だけ目標角度パンチルト
	// int pan_angular ;
	// int tilt_angular; 
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
	// 確認用
	bool check;
    public:
    //コンストラクタ：クラス定義に呼び出されるメソッド
    pantiltPositionControlClass();
    //デストラクタ：クラスが消滅するときに呼びだされるメソッド
    ~pantiltPositionControlClass();
	//コントローラの係数λと目標位置と角度の設定 動的パラメータ変更用コールバック関数
	void Config_Callback_Function(pantilt_position_control::goal_coefficient_paramConfig &config, uint32_t level);
	// 検出システムからのコールバック関数
	void TargetPointCallback(const obstacle_detection_2019::ClassificationVelocityData::ConstPtr& msg);
	void calculationProcess();
	void targetDecision();
	void stop();
	void angularCopy();
	void goalSet();
	void sensorDataGet();
	void deviation();
	// void inverse();
	void input();
	void beegoVelocityOrder();
	void bisectionMethod(double x ,double c,double y);
	void RotationToPWM();
	void plusTime();
	void publishJudgment();
	void publishBeegoData();
	void publishPantiltData();
	void publishPantiltTimeData();
};
#endif
// rqt_reconfigureコールバック関数 callback_function
void pantiltPositionControlClass::Config_Callback_Function(pantilt_position_control::goal_coefficient_paramConfig &config, uint32_t level)
{
	// 係数λ
	lambda = config.p_control ;
	// 目標三次元位置 実験ではy=1にする
	distance_x	 = config.distance_x ;
	distance_y	 = config.distance_y ;
	distance_z	 = config.distance_z ;
	ROS_INFO("reconfigure_ok lambda = %f" , lambda);
	ROS_INFO("reconfigure_ok distance_x = %f",distance_x );
	ROS_INFO("reconfigure_ok distance_x = %f",distance_y );
	ROS_INFO("reconfigure_ok distance_x = %f",distance_z );
	goalSet();
}
// 目標値設定
void pantiltPositionControlClass::goalSet()
{
	//目標値設定 対象までの相対位置と相対姿勢
	// リサイズ
	// t_g_c.resize(3) ;
	// phi_g_c .resize(3);
	// 各データの入力
	// 目標距離 自分で設定
	t_g_c(0) = distance_x ;
	t_g_c(1) = distance_y ;
	t_g_c(2) = distance_z ;
	// 目標角度は0 
	phi_g_c = Eigen::Vector3d::Zero();
	ROS_INFO("goal_position_set_ok");
}
// コールバック関数 callbackfunction
void pantiltPositionControlClass::TargetPointCallback(const obstacle_detection_2019::ClassificationVelocityData::ConstPtr& msg)
{
	// コピー
	copyTargetGravityCenterMessage = *msg ;
	ROS_INFO("target_position_copy_ok");
	// 計算プロセス
	calculationProcess();
	if(check==false)
	{
		// 目標角度値とビーゴ速度0をずっと入力
		publishJudgment();
		ROS_INFO("stop");
	}
	ROS_INFO("process_finish");
}
// 計算プロセス
void pantiltPositionControlClass::calculationProcess()
{
	targetDecision();
	stop();
	sensorDataGet();
	deviation();
	RotationToPWM();
	angularCopy();
	publishJudgment();
} 
void pantiltPositionControlClass::targetDecision()
{
	// 重心の幅求める
	width = 0 ;
	widthMax = 0 ;
	// 各セルごとの重心位置とクラスタ全体の重心との距離を比較し最大距離を求めその円の半径を対象の幅とする
	// 目標の重心距離の決定
		goalDistance = sqrt(distance_x*distance_x
					+distance_y*distance_y
					+distance_z*distance_z);
	// クラスタデータの数,要素を求める
	// 全クラスタの中の重心がロボットとの距離と一番近いのを追跡対象にする
	for(l = 0; l <  最大クラスタ要素数 ; l++)
	{
		// クラスタ距離の計算
		gcCluster = sqrt(copyTargetGravityCenterMessage.data[l].gc.x*copyTargetGravityCenterMessage.data[l].gc.x
						+copyTargetGravityCenterMessage.data[l].gc.y*copyTargetGravityCenterMessage.data[l].gc.y
						+copyTargetGravityCenterMessage.data[l].gc.z*copyTargetGravityCenterMessage.data[l].gc.z );
		// 最初の検出した障害物
		if(l == 0 && gcMin >= goalDistance){gcMin = gcCluster};
			// 一番距離が近い障害物をgcMinに入れる
			if(gcMin >= gcCluster && gcCluster >= goalDistance)
			{
				gcMin = gcCluster ;
				// 特定したクラスタを障害物として幅計算
 				for(k = 1; k <= クラスタ内の３次元データの最大要素数 ; k++)
				{
					width = sqrt(
								 ((copyTargetGravityCenterMessage.data[l].pt[k].x - copyTargetGravityCenterMessage.data[l].gc.x)
								* (copyTargetGravityCenterMessage.data[l].pt[k].x - copyTargetGravityCenterMessage.data[l].gc.x))
								+((copyTargetGravityCenterMessage.data[l].pt[k].y - copyTargetGravityCenterMessage.data[l].gc.y)
								* (copyTargetGravityCenterMessage.data[l].pt[k].y - copyTargetGravityCenterMessage.data[l].gc.y))
								);
								// +((copyTargetGravityCenterMessage.data[l].pt[k].z - copyTargetGravityCenterMessage.data[l].gc.z)
								// * (copyTargetGravityCenterMessage.data[l].pt[k].z - copyTargetGravityCenterMessage.data[l].gc.z))
    				// 全体の重心から幅が広い
					if(width >= widthMax)
					{
    				    widthMax = width;
    				}
    			}
				// センサー誤差 速度推定によりカルマンフィルタが使用されている
				// センサデータそのまま障害物位置確定
				targetGravityCenterMessage.linear.x = copyTargetGravityCenterMessage.data[l].gc.y;//重心
				targetGravityCenterMessage.linear.y = copyTargetGravityCenterMessage.data[l].gc.y;//重心
				targetGravityCenterMessage.linear.z = copyTargetGravityCenterMessage.data[l].gc.z;//重心
				ROS_INFO("x= %f" ,targetGravityCenterMessage.linear.x);
				ROS_INFO("y= %f" ,targetGravityCenterMessage.linear.y);
				ROS_INFO("z= %f" ,targetGravityCenterMessage.linear.z);
			}
	}
}
void pantiltPositionControlClass::stop()git push origin master
{	
	// 対象の幅とパンチルトの揺れを考慮した距離の範囲 0近ければ終了この時の命令値と距離から対象姿勢を算出
	// 奥行き設定 とりあえず円
	length = widthMax;
	ellipse = ( (distance_x- targetGravityCenterMessage.linear.x) * (distance_x- targetGravityCenterMessage.linear.x) / widthMax) 
			+ ( (distance_y - targetGravityCenterMessage.linear.y) * (distance_y- targetGravityCenterMessage.linear.y) / length) ;
	if(ellipse <= 1 )
	{
		// 止める用
		vel_c_t = Eigen::Vector3d::Zero();
		// 対象までの角度 障害物姿勢のこと
		targetGravityCenterMessage.angular.z = atan2(targetGravityCenterMessage.linear.x , targetGravityCenterMessage.linear.y);
		panTiltMessage.data[0] = targetGravityCenterMessage.angular.z;
		panTiltMessage.data[1] = start_tilt_angular;
		publishJudgment();
		// 精度を見る用
		target_publisher.publish(targetGravityCenterMessage);//パンチルト指令値をpublishする
		ROS_INFO("stop");
		check = false ;		
		// 関数強制脱出
		return ;
	}
}
// 出力センサデータの受け取り
void pantiltPositionControlClass::sensorDataGet()
{
	//観測データの格納 対象までの相対位置と相対姿勢
	// 並進方向 ビーゴ
	t_o_c(0) = targetGravityCenterMessage.linear.x ;
	t_o_c(1) = targetGravityCenterMessage.linear.y ;
	t_o_c(2) = targetGravityCenterMessage.linear.z ;
	// 回転方向 とれないので計算する 
	// ロールとピッチは本来0であるが誤差があるため
	// 実験よりロボットと対象が動いてない時のパンチルト動作の誤差を見て仮に0に近似する
	psi_o_c(0) = 0 ;
	psi_o_c(1) = 0 ;
	// 目標までのズレ 対象の幅分考慮 
	psi_o_c(2) = atan2(t_o_c(0) , t_o_c(1));
	ROS_INFO("receive_target_position_ok");
}
// 誤差eの作成
void pantiltPositionControlClass::deviation()
{
	//偏差 観測データと目標値の差
	// 逆かも
	error_difference  = t_g_c - t_o_c ;
	error_theta 	  = phi_g_c - psi_o_c ;
	ROS_INFO("error_value_ok");
	// inverse();
	input();
}
// 入力uの作成
void pantiltPositionControlClass::input()
{
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
void pantiltPositionControlClass::bisectionMethod(double x ,double c, double y)
{
	pwm = -2.2730205129 + 3.0743433753*x - 0.0231440897*x*x + 5.00167336716448E-05*x*x*x - y;
	pwmCenter = -2.2730205129 + 3.0743433753*c - 0.0231440897*c*c + 5.00167336716448E-05*c*c*c - y;
}
void pantiltPositionControlClass::RotationToPWM()
{
	// 回転ベクトルからpwmに変換
	// 回転式導出 最小二乗法 実際の回転速度から
	solution = omega_c_psi(2);
	if(solution < 120)
	for(i = 1; i<=limit ; i++)
	{
		center = (minRotation + maxRotation) / 2;
		bisectionMethod(maxRotation,center, solution); 
		if (pwm*pwmCenter > 0)
		{maxRotation = center;}
    	else
        {minRotation  = center;}
		if ( fabs(maxRotation - minRotation)  < eps) 
		{
			pwmRotationYaw = pwm ;
			break ;
		}
	}
	if (i > limit)
	{ROS_INFO("no_solution") ;}
	pwmRotationPitch = omega_c_psi(1);
	// pwm値四捨五入
	panTiltMessage.data[2] = (int)(pwmRotationYaw+ 0.5);
	panTiltMessage.data[3] = (int)(pwmRotationPitch+ 0.5);
	ROS_INFO("pantilt_PWM_velocity_ok");
}
void pantiltPositionControlClass::angularCopy()
{
	pantiltTimeMessage.pantiltArray.data.resize(4);
	// 指令値用に目標値角度コピー
	// 四捨五入 int出ないと送れない
	// 対象との角度差正面90度が0度であり
	// パンは正面が90度であるため0から180に変換
	panTiltMessage.data[0] = (int)(error_theta(2)+ 0.5) + 90; 
	pantiltTimeMessage.pantiltArray.data[0] = panTiltMessage.data[0];
	// チルトコピー用
	panTiltMessage.data[1] = (int)(error_theta(1)+ 0.5);
	pantiltTimeMessage.pantiltArray.data[1] = panTiltMessage.data[1];
	ROS_INFO("pan_angular_goal_ok");
}
//パンチルト角度角速度のデータコピー及びタイムスタンプ付与
void pantiltPositionControlClass::plusTime()
{
// 実行した後Rvizでtfを見たものが、下記
	pantiltTimeMessage.pantiltArray.data.resize(4);
	pantiltTimeMessage.header.frame_id = "pantilt_command" ;
	pantiltTimeMessage.header.stamp = ros::Time::now() ;
	pantiltTimeMessage.pantiltArray.data[0] = panTiltMessage.data[0] ;//入力角度値 z軸
	pantiltTimeMessage.pantiltArray.data[1] = panTiltMessage.data[1];//入力角度値 y軸
	pantiltTimeMessage.pantiltArray.data[2] = panTiltMessage.data[2];//入力回転速度値 z軸
	pantiltTimeMessage.pantiltArray.data[3] = panTiltMessage.data[3];//入力回転速度値 y軸
	ROS_INFO("pantilt_time_copy_ok");
}
//publish判定を行う
void  pantiltPositionControlClass::publishJudgment()
{
	publishBeegoData();
    publishPantiltData();//パンチルト指令値publish
	publishPantiltTimeData();//パンチルトとタイムスタンプをpublishする
}
void pantiltPositionControlClass::publishBeegoData()
{
	beegoVelocityOrder();
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
	plusTime();
    ROS_INFO("pantilt_edit_publish_ok");
	pantilt_time_publisher.publish(pantiltTimeMessage);//パンチルトとタイムスタンプをpublishする
}
//constructor
pantiltPositionControlClass::pantiltPositionControlClass()
:check(true),lambda(0.0),distance_x(0.0),distance_y(0.0),distance_z(0.0) 
,start_pan_angular(90),start_tilt_angular(104)
,time(ros::Time::now())
,width(0),widthMax(0),length(0),i(0),k(0),l(0)
,pwmRotationYaw(0),pwmRotationPitch(0),gcMin(0),gcCluster(0)
,minRotation(1),maxRotation(118),eps(0.0001),solution(0),pwm(0),pwmCenter(0),center(0)
{
	position_control_subscriber = nodehandle_subscriber.subscribe("/robot2/classificationDataEstimateVelocity", 1, &pantiltPositionControlClass::TargetPointCallback,this);
	pantilt_publisher = nodehandle_pantilt_publisher.advertise<std_msgs::UInt16MultiArray>("PanTilt",1);
	pantilt_time_publisher = nodehandle_publisher.advertise<pantilt_position_control::pantiltArray>("PanTiltPlusTimeData",1);
	beego_publisher = nodehandle_beego_publisher.advertise<geometry_msgs::Twist>("beego/cmd_vel",1);
	target_publisher = nodehandle_target_publisher.advertise<geometry_msgs::Twist>("TargetPosition",1);
	
	f = boost::bind(&pantiltPositionControlClass::Config_Callback_Function, this,  _1, _2);
	server.setCallback(f);

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
	ros::spin();
	return 0;
}
