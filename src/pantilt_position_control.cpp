//ヘッダ header 
#ifndef INCLUDE_SYNCRO_IMAGE_CLASS
#define INCLUDE_SYNCRO_IMAGE_CLASS
//rosを使って制御するため
#include <ros/ros.h>
#include <ros/callback_queue.h>
//ros msg header パンチルト指令値型
#include <std_msgs/UInt16MultiArray.h>
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
// 別のcppファイル検出プログラム
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
	obstacle_detection_2019::ClassificationVelocityData copyTargetGravityCenterMessage;//カメラ座標からの対象物体の重心コピー用
	geometry_msgs::Twist targetGravityCenterMessage;//カメラ座標からの対象物体の重心
	geometry_msgs::Twist beegoMessage;//ビーゴ命令用
	std_msgs::UInt16MultiArray panTiltMessage;//ラテパンダ命令用
	pantilt_position_control::pantiltArray pantiltTimeMessage;//時間付きpwm角速度
	ros::Time time ;
	// 各変数の使用場所
	double distance_x ,distance_y ,distance_z ;// 目標距離rqt設定
	double lambda ;// 係数λ
	bool check;// 停止判断用
	int k ,l ;// for文用カウンタ変数
	double goalDistance ;// 目標の重心距離
	double gcMin;//クラスタ最小距離
	double gcCluster;//クラスタ距離
	double width ;//対象の幅距離
	double widthMax ;//幅の最大距離
	double length ; //奥行き
	double ellipse ;//楕円
	double eps ;// 打ち切り精度
	int i ,limit ;// カウンタ変数と打ち切り回数
	double minRotation , maxRotation  ;// 区間最大は256,最小は1
	double solution ,center; //命令回転速度と中間区間の命令回転速度 
	double maxSolution ,pwmCenter;//pwm命令回転速度
	double pwmRotationYaw ,pwmRotationPitch;//pwm回転速度値
	int pan_velocity ,tilt_velocity;// 実際にPWMとして入力する回転速度
	int pan_angular ,tilt_angular;//実際に指令値として入力するパン 誤差分だけ目標角度
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
    public:
    //コンストラクタ：クラス定義に呼び出されるメソッド
    pantiltPositionControlClass();
    //デストラクタ：クラスが消滅するときに呼びだされるメソッド
    ~pantiltPositionControlClass();
	//コントローラの係数λと目標位置と目標角度の設定 動的パラメータ変更
	void Config_Callback_Function(pantilt_position_control::goal_coefficient_paramConfig &config, uint32_t level);
	// 対象位置のコールバック関数
	void TargetPointCallback(const obstacle_detection_2019::ClassificationVelocityData::ConstPtr& msg);
	void goalSet();
	void calculationProcess();
	void targetDecision();
	void stop();
	void angularVelocityCopy();
	void sensorDataGet();
	void deviation();
	void input();
	void beegoVelocityOrder();
	void bisectionMethod();
	void approximateExpression(double x ,double c, double y);
	void RotationToPWM();
	void plusTime();
	void publishJudgment();
	void publishBeegoData();
	void publishPantiltData();
	void publishPantiltTimeData();
};
#endif
// 関数
//コントローラの係数λと目標位置と目標角度の設定 動的パラメータ変更コールバック関数 callback_function
void pantiltPositionControlClass::Config_Callback_Function(pantilt_position_control::goal_coefficient_paramConfig &config, uint32_t level)
{
	
	lambda = config.p_control ;// 係数λ設定
	// 目標三次元位置 実験ではy=1にする
	distance_x	 = config.distance_x ;
	distance_y	 = config.distance_y ;
	distance_z	 = config.distance_z ;
	ROS_INFO("reconfigure_ok lambda = %f" , lambda);
	ROS_INFO("reconfigure_ok distance_x = %f",distance_x );
	ROS_INFO("reconfigure_ok distance_y = %f",distance_y );
	ROS_INFO("reconfigure_ok distance_z = %f",distance_z );
}
// 検出された位置情報のコールバック関数
void pantiltPositionControlClass::TargetPointCallback(const obstacle_detection_2019::ClassificationVelocityData::ConstPtr& msg)
{
	goalSet();
	copyTargetGravityCenterMessage = *msg ;// コピー
	ROS_INFO("target_position_get_ok");
	calculationProcess();// 計算プロセス
	// 停止判定
	if(check==false)
	{
		stop();// 目標角度値とビーゴ速度0を入力
	}
	ROS_INFO("process_finish");
}
// 目標値設定
void pantiltPositionControlClass::goalSet()
{
	//目標値自分で設定 
	// 目標距離　対象までの相対位置
	t_g_c(0) = distance_x ;
	t_g_c(1) = distance_y ;
	t_g_c(2) = distance_z ;
	ROS_INFO("goal_distance= %f" ,t_g_c(0));
	ROS_INFO("goal_distance= %f" ,t_g_c(1));
	ROS_INFO("goal_distance= %f" ,t_g_c(2));
	// 目標角度は0　 対象までの相対姿勢
	phi_g_c = Eigen::Vector3d::Zero();
	ROS_INFO("goal_position_set_ok");
}
// 計算プロセス
void pantiltPositionControlClass::calculationProcess()
{
	targetDecision();
	sensorDataGet();
	deviation();
	RotationToPWM();
	publishJudgment();
} 
// 検出された障害物の追跡対象とその重心位置の決定方法
void pantiltPositionControlClass::targetDecision()
{
	// copyTargetGravityCenterMessage.data[クラスタ番号].gc;クラスタの重心
	// 目標の重心距離の決定
	goalDistance = sqrt(distance_x*distance_x
				       +distance_y*distance_y);
	ROS_INFO("goal_distance = %f",goalDistance);		   
	// 各セルごとの重心位置とクラスタ全体の重心との距離を比較し最大距離を求めその円の半径を対象の幅とする
	// 全クラスタの中の重心がロボットとの距離と一番近いのを追跡対象にする
	for(l = 0; l < (int)copyTargetGravityCenterMessage.data.size() ; l++)
	{
		// クラスタ距離の計算 高さは考慮しない
		gcCluster = sqrt(copyTargetGravityCenterMessage.data[l].gc.x*copyTargetGravityCenterMessage.data[l].gc.x
						+copyTargetGravityCenterMessage.data[l].gc.y*copyTargetGravityCenterMessage.data[l].gc.y
						);
		ROS_INFO("gc_cluster[%d] = %f",l,gcCluster);
		// 一番最初の検出した障害物
		if(l == 0 && gcCluster >= goalDistance)
			{
				gcMin = gcCluster ;
				ROS_INFO("first_gravity_center");
			// センサー誤差は速度推定によりカルマンフィルタが使用されているためセンサデータそのままで障害物位置確定
			targetGravityCenterMessage.linear.x = copyTargetGravityCenterMessage.data[l].gc.y;//重心
			targetGravityCenterMessage.linear.y = copyTargetGravityCenterMessage.data[l].gc.y;//重心
			targetGravityCenterMessage.linear.z = copyTargetGravityCenterMessage.data[l].gc.z;//重心
			}
		// 一番距離が近い障害物をgcMinに入れる
		if(gcMin > gcCluster && gcCluster >= goalDistance)
		{
			gcMin = gcCluster ;
			//重心の幅求める
			widthMax = 0 ;
			//特定したクラスタを障害物としてその重心からの幅計算
 			for(k = 0; k < (int)copyTargetGravityCenterMessage.data[l].pt.size() ; k++)
			{
				width = sqrt(
							 ((copyTargetGravityCenterMessage.data[l].pt[k].x - copyTargetGravityCenterMessage.data[l].gc.x)
							* (copyTargetGravityCenterMessage.data[l].pt[k].x - copyTargetGravityCenterMessage.data[l].gc.x))
							+((copyTargetGravityCenterMessage.data[l].pt[k].y - copyTargetGravityCenterMessage.data[l].gc.y)
							* (copyTargetGravityCenterMessage.data[l].pt[k].y - copyTargetGravityCenterMessage.data[l].gc.y))
							);
    			// 全体の重心から最も広い幅をwidthMaxに入れる
				if(width >= widthMax)
				{
    			    widthMax = width;
    			}
				width = 0 ;
    		}
		// センサー誤差は速度推定によりカルマンフィルタが使用されているためセンサデータそのままで障害物位置確定
		targetGravityCenterMessage.linear.x = copyTargetGravityCenterMessage.data[l].gc.y;//重心
		targetGravityCenterMessage.linear.y = copyTargetGravityCenterMessage.data[l].gc.y;//重心
		targetGravityCenterMessage.linear.z = copyTargetGravityCenterMessage.data[l].gc.z;//重心
		}
	}
	ROS_INFO("width = %f",widthMax);
	ROS_INFO("position_x= %f" ,targetGravityCenterMessage.linear.x);
	ROS_INFO("position_y= %f" ,targetGravityCenterMessage.linear.y);
	ROS_INFO("position_z= %f" ,targetGravityCenterMessage.linear.z);
	ROS_INFO("cluster_numbers = %d",(int)copyTargetGravityCenterMessage.data.size());
	// 対象の幅とパンチルトの揺れを考慮した距離の範囲内で終了
	// 奥行き設定 とりあえず円
	length = widthMax;
	ellipse = ( (distance_x- targetGravityCenterMessage.linear.x) * (distance_x- targetGravityCenterMessage.linear.x) / (widthMax*widthMax)) 
			+ ( (distance_y - targetGravityCenterMessage.linear.y) * (distance_y- targetGravityCenterMessage.linear.y) / (length*length)) ;
	ROS_INFO("length  =  %f" ,length );
	ROS_INFO("ellipse =  %f" ,ellipse);
	if(ellipse <= 1 )
	{check = false ;}//停止判定変数	
}
// パンチルトカメラ搭載移動ロボット停止判断
void pantiltPositionControlClass::stop()
{	
		vel_c_t = Eigen::Vector3d::Zero();//ビーゴ停止
		//命令値と距離から対象姿勢（対象までの角度）を算出
		targetGravityCenterMessage.angular.z = atan2(targetGravityCenterMessage.linear.x , targetGravityCenterMessage.linear.y);
		pan_angular = targetGravityCenterMessage.angular.z;
		publishJudgment();
		// 精度を見る用
		target_publisher.publish(targetGravityCenterMessage);//パンチルト指令値をpublishする
		ROS_INFO("stop");
}
// 出力センサデータの受け取り
void pantiltPositionControlClass::sensorDataGet()
{
	// 並進方向対象までの相対位置 ビーゴ
	t_o_c(0) = targetGravityCenterMessage.linear.x ;
	t_o_c(1) = targetGravityCenterMessage.linear.y ;
	t_o_c(2) = targetGravityCenterMessage.linear.z ;
	ROS_INFO("target_distance = %f" ,t_o_c(0));
	ROS_INFO("target_distance = %f" ,t_o_c(1));
	ROS_INFO("target_distance = %f" ,t_o_c(2));
	// 回転方向対象までの相対姿勢 とれないので計算する 
	// ロールとピッチは本来0であるが誤差がある
	// 実験よりロボットと対象が動いてない時のパンチルト動作の誤差を見て仮に0に近似する
	psi_o_c(0) = 0 ;//ロール
	psi_o_c(1) = 0 ;//ピッチ
	psi_o_c(2) = atan2(t_o_c(0) , t_o_c(1));//ヨー 目標までのズレ 相対姿勢　
	ROS_INFO("psi= %f" ,psi_o_c(2));
	ROS_INFO("receive_target_position_ok");
}
// 誤差eの作成
void pantiltPositionControlClass::deviation()
{
	//偏差 観測データと目標値の差
	// 逆かも
	error_difference  = t_g_c - t_o_c ;//並進
	error_theta 	  = phi_g_c - psi_o_c ;//回転
	pan_angular = round(error_theta(2)) ;// pwm値四捨五入
	ROS_INFO_STREAM("to0_pan_ang "<<pan_angular );
	ROS_INFO("diffe= %f" ,error_difference(0));
	ROS_INFO("diffe= %f" ,error_difference(1));
	ROS_INFO("diffe= %f" ,error_difference(2));
	ROS_INFO("theta= %f" ,error_theta(2));
	ROS_INFO("error_value_ok");
	input();
}
// 入力uの作成
void pantiltPositionControlClass::input()
{
	// -λ逆行列J-1誤差e
 	vel_c_t = -1*lambda*(I*error_difference + t_g_c.cross(error_theta));
 	ROS_INFO("vel_x = %f" ,vel_c_t(0));
	ROS_INFO("vel_y = %f" ,vel_c_t(1));
	ROS_INFO("vel_z = %f" ,vel_c_t(2));
	omega_c_psi = -1*lambda*( Eigen::Matrix3d::Zero()*error_difference + I*error_theta);
	ROS_INFO("rot_z = %f" ,omega_c_psi(2));
	ROS_INFO("input_u_ok");
}
// ビーゴ右左合わせての速度publish
void pantiltPositionControlClass::beegoVelocityOrder()
{
	// ビーゴ命令値
	beegoMessage.linear.x = vel_c_t(0);
	beegoMessage.linear.y = vel_c_t(1);
	beegoMessage.linear.z = vel_c_t(2);
}
// 二分法
void pantiltPositionControlClass::bisectionMethod()
{
	for(i = 1; i<=limit ; i++)
		{
			if(i>limit)
			{
				ROS_INFO("no_solution") ;
				break ;	
			}
			pwmCenter = (minRotation + maxRotation) / 2;
			approximateExpression(maxRotation ,pwmCenter ,solution ); 
			if (maxSolution*center < 0)
				{minRotation = pwmCenter;}
    		else
    	    	{maxRotation  = pwmCenter;}
			if ( fabs(maxRotation - minRotation)  < eps) 
			{
				break ;
			}
		}
		pwmRotationYaw = pwmCenter;
		ROS_INFO("pwmYawomega = %f",pwmRotationYaw);
}
//実際の回転速度から最小二乗法より制御用回転制御式　
void pantiltPositionControlClass::approximateExpression(double x ,double c, double y)
{
	maxSolution = -2.2730205129 + 3.0743433753*x - 0.0231440897*x*x + 5.00167336716448E-05*x*x*x - y;
	center = -2.2730205129 + 3.0743433753*c - 0.0231440897*c*c + 5.00167336716448E-05*c*c*c - y;
}
// 回転ベクトルからpwmに変換
void pantiltPositionControlClass::RotationToPWM()
{
	limit = 1000 ;
	solution = omega_c_psi(2);
	ROS_INFO("omega = %f",solution);
	//90度が中心,左右で＋ー
	if(solution < 120 && solution >0)
	{
		bisectionMethod();
		ROS_INFO("right_turn") ;
	}
	else if(solution == 0)
	{
		stop();
		ROS_INFO("robot_stop") ;
	}
	else if (solution < 0 )
	{
		solution = -1 * solution ;
		bisectionMethod();
		pwmRotationYaw = -1 * pwmRotationYaw ;
		ROS_INFO("left_turn") ;
	}
	else 
	{
		pwmRotationYaw = 0 ;
		ROS_INFO("no_solution") ;
	}
	//絶対値を取る
	if(omega_c_psi(1) > 0)
		{pwmRotationPitch = omega_c_psi(1);}
	else
		{pwmRotationPitch = -1 * omega_c_psi(1);}
	ROS_INFO("pwmPitchomega = %f",pwmRotationPitch);
	// pwm値四捨五入
	pan_velocity = round(pwmRotationYaw);
	tilt_velocity = round(pwmRotationPitch);
	//pan_velocity = (int)(pwmRotationYaw+ 0.5);
	//tilt_velocity = (int)(pwmRotationPitch+ 0.5);ROS_INFO_STREAM("to0_pan_ang "<<pan_angular );
	ROS_INFO("pantilt_PWM_velocity_ok");
}
// パンチルト目標角度と指令回転速度値の入力
void pantiltPositionControlClass::angularVelocityCopy()
{
	// 指令値用に目標値角度
	// 対象との角度差正面90度が0度であり
	// パンは正面が90度であるため0から180に変換
	pan_angular = pan_angular + 90;
	tilt_angular = 104 ;
	ROS_INFO_STREAM("pan_ang "<<pan_angular );
	ROS_INFO_STREAM("tilt_ang"<<tilt_angular);
	//4つにresize	
	panTiltMessage.data.resize(4);
	panTiltMessage.data[0] = pan_angular;
	panTiltMessage.data[1] = tilt_angular;
	panTiltMessage.data[2] = pan_velocity ;
	panTiltMessage.data[3] = tilt_velocity;
	ROS_INFO("pan_angular_goal_ok");
}
//パンチルト角度角速度のデータコピー及びタイムスタンプ付与
void pantiltPositionControlClass::plusTime()
{
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
	publishBeegoData();// ビーゴ右左合わせての速度publish
    publishPantiltData();//パンチルト指令値publish
	publishPantiltTimeData();//パンチルトとタイムスタンプをpublishする
}
// ビーゴ右左合わせての速度publish
void pantiltPositionControlClass::publishBeegoData()
{
	beegoVelocityOrder();
	ROS_INFO("beego_publish_ok");
	beego_publisher.publish(beegoMessage);//ビーゴ命令値をpublishする
}
//パンチルト角度角速度publish
void pantiltPositionControlClass::publishPantiltData()
{
	angularVelocityCopy();
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
//コンストラクタ：クラス定義に呼び出されるメソッド
pantiltPositionControlClass::pantiltPositionControlClass()
:lambda(0.0),distance_x(0.0),distance_y(0.0),distance_z(0.0),check(true)
,k(0),l(0),goalDistance(0),gcMin(0),gcCluster(0),width(0),widthMax(0)
,length(0),ellipse(0)
,i(0),limit(0),minRotation(1),maxRotation(256),eps(0.0001)
,maxSolution(0),pwmCenter(0),solution(0),center(0)
,pwmRotationYaw(0),pwmRotationPitch(0)
,pan_velocity(0),tilt_velocity(0),pan_angular(0), tilt_angular(0) 
,time(ros::Time::now())
{
	// サブスクライバ
	position_control_subscriber = nodehandle_subscriber.subscribe("/robot2/classificationDataEstimateVelocity", 1, &pantiltPositionControlClass::TargetPointCallback,this);
	// パブリッシャ
	pantilt_publisher = nodehandle_pantilt_publisher.advertise<std_msgs::UInt16MultiArray>("PanTilt",1);
	pantilt_time_publisher = nodehandle_publisher.advertise<pantilt_position_control::pantiltArray>("PanTiltPlusTimeData",1);
	beego_publisher = nodehandle_beego_publisher.advertise<geometry_msgs::Twist>("beego/cmd_vel",1);
	target_publisher = nodehandle_target_publisher.advertise<geometry_msgs::Twist>("TargetPosition",1);
	// rqt_reconfigure
	if(rqt_reconfigure)
	{
	f = boost::bind(&pantiltPositionControlClass::Config_Callback_Function, this,  _1, _2);
	server.setCallback(f);
	}
	// 行列初期化
	t_g_c = Eigen::Vector3d::Zero(3);
	phi_g_c = Eigen::Vector3d::Zero(3);
	t_o_c = Eigen::Vector3d::Zero(3);
	psi_o_c = Eigen::Vector3d::Zero(3);
	vel_c_t = Eigen::Vector3d::Zero(3);
	omega_c_psi = Eigen::Vector3d::Zero(3);	
	I = Eigen::MatrixXd::Identity(3,3); 
}
//デストラクタ：クラスが消滅するときに呼びだされるメソッド
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