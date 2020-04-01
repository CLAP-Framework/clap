
/*author :zx
date 2020.2.25
function ： test Vehicle mode 
*/
#include"icv_carla_node.h"
#include"icv_carla_control.h"

using namespace std;
using namespace Eigen;

namespace icv
{
    icvCarlaControl::icvCarlaControl(/* args */)
    
  
    {
        controlInit(); //从yaml文件中获取Vehiclemodel和controlparam配置信息
       
    }
    
    icvCarlaControl::~icvCarlaControl()
    {
    }

    template<typename T>
    T getParam(const YAML::Node& node,const string& name,const T& defaultValue)
    {
        T v;
        try 
        {
            v=node[name].as<T>();
            std::cout<<"Found parameter: "<<name<<",\tvalue: "<<v<<std::endl;
        } 
        catch (std::exception e) 
        {
            v=defaultValue;
            std::cout<<"Cannot find parameter: "<<name<<",\tassigning  default: "<<v<<std::endl;
        }
        return v;
    }
    /**初始化话控制参数，调用yaml文件*/
    void icvCarlaControl::controlInit()
    {
        //加载参数文件
        YAML::Node dset_config = YAML::LoadFile("/home/icv/zzz/src/driver/sensors/xiaopeng/icvfollower/config/Controlconfig.yaml");
        //Vehiclemodel,Controlparam two key_value
        YAML::Node Vehiclemodel=dset_config["Vehiclemodel"];
        YAML::Node Controlparam=dset_config["Controlparam"];        
        
        /****VehicleModle*****/
        lf=getParam<float>(Vehiclemodel,"_lf",0);
        lr=getParam<float>(Vehiclemodel,"_lr",0);     
        L=lr+lf;
        m=getParam<float>(Vehiclemodel,"_m",0);     //mass of Vehicle
        Ccf=getParam<float>(Vehiclemodel,"_Ccf",0); //前轮侧偏刚度
        Ccr=getParam<float>(Vehiclemodel,"_Ccr",0); //后轮侧偏刚度
        Clf=getParam<float>(Vehiclemodel,"_Clf",0); //前轮纵向刚度
        Clr=getParam<float>(Vehiclemodel,"_Clr",0); //后轮纵向刚度
        delta_f=getParam<float>(Vehiclemodel,"_delta_f",0); //前轮偏角度
        Sf=getParam<float>(Vehiclemodel,"_Sf",0); //前轮纵向滑移率
        Sr=getParam<float>(Vehiclemodel,"_Sr",0); //后轮纵向滑移率
        I=getParam<float>(Vehiclemodel,"_I",0); //车辆质心转动惯量
        wheel_base=getParam<float>(Vehiclemodel,"_L",0); //车辆轴距
        g=getParam<float>(Vehiclemodel,"_g",0); 
        R_min=getParam<float>(Vehiclemodel,"_Rmin",0); //最小转向半径
        K=getParam<float>(Vehiclemodel,"_K",0); //方向盘传动比
        
       /********Controlparam***************/
        __distance_pre=getParam<float>(Controlparam,"_distance_pre",0);     //mass of Vehicle
        _p1_neg=getParam<float>(Controlparam,"_P1_neg",0);
        _p2_neg=getParam<float>(Controlparam,"_P2_neg",0);
        _p3_neg=getParam<float>(Controlparam,"_P3_neg",0);
        _p1_post=getParam<float>(Controlparam,"_P1_post",0);
        _p2_post=getParam<float>(Controlparam,"_P2_post",0);
        _p3_post=getParam<float>(Controlparam,"_P3_post",0);
        _dis_pre_max=getParam<float>(Controlparam,"_dis_pre_max",0);
        _dis_pre_min=getParam<float>(Controlparam,"_dis_pre_min",0);
        _wheel_angle_degree_max=getParam<float>(Controlparam,"_wheel_angle_degree_max",0);
        _wheel_angle_degree_sec=getParam<float>(Controlparam,"_wheel_angle_degree_sec",0);
        _t_preview=getParam<float>(Controlparam,"_t_preview",2.8);
        _v_pid.Kp=getParam<float>(Controlparam,"_v_pid.Kp",0.15);
        _v_pid.Kd=getParam<float>(Controlparam,"_v_pid.Kd",0);
        _v_pid.Ki=getParam<float>(Controlparam,"_v_pid.Ki",0.01);
        _delta_T=getParam<float>(Controlparam,"_delta_T",0.05);
    }
    /***主程序循环*/////////////////////////////////////////////////////////////////////////////////////////////
    /***********************************************************************************************/
    void icvCarlaControl::run_follower(double *out_steerangle)
    {       
         
        data2file(); 
        clock_t  begin ,end;//clock() 不能计算sleep()占用的时间  
        begin=clock();   
        PathSmoothCheck();            
        delta_f=ControlStr(Current_Point.x,Current_Point.y,Current_Point.theta,Current_Point.v)/15.7/180*PI;//     
        // usleep(200000);   
        *out_steerangle=delta_f;//输出前轮转角弧度
        end=clock();
        sample_time=(end - begin) / CLOCKS_PER_SEC;        
    }   

    

   /**icv跟踪控制主体**/
    double  icvCarlaControl::ControlStr(double _vehicle_x,double _vehicle_y,double _vehicle_heading_theta,double _vehicle_v) 
    // double  icvCarlaControl::ControlStr() 
{    
        float _G;                   //the steering gear ratio (hand wheel angle/road wheel angle)
        float _t;                   //time        
        float _d_0 = 0.5; //起始距离.        
        float _theta_ss;
        float _d_dis;
        float _x_preview;        //预瞄处的x坐标
        float _y_preview;        //预瞄处的y坐标
        float _Oss;              //距离偏差        
        int _keep_steer=0;//zx
        float _transfer_val;
        float _wheel_angle_degree;
        _vehicle_x=_vehicle_x+_vehicle_v*__distance_pre*cos(_vehicle_heading_theta*PI/180);;
        _vehicle_y=_vehicle_y+_vehicle_v*__distance_pre*sin(_vehicle_heading_theta*PI/180);;
        
        if (_keep_steer==0)//判断是够跟踪
        {
            _d_dis = _t_preview * _vehicle_v + _d_0;   //公式分母中的ｄ
            lookahead_distance_=_d_dis;
            if (_d_dis<=5)
            {
                _d_dis=5;
            }
            _x_preview=_vehicle_x+_d_dis * cos(_vehicle_heading_theta * PI/180);
            _y_preview=_vehicle_y+_d_dis * sin(_vehicle_heading_theta * PI/180);
            //RVIZ显示           
            _x_preview_rviz=_x_preview;            
            _y_preview_rviz=_y_preview;
            _Oss=find_min_point(_x_preview,_y_preview,&nextpoint_num);    //寻找最小值,左负右正   
            ctedata[2]=_Oss;//zx数据存储          
            // //TODO:增加曲率计算，进行转弯补偿；
            // geometry_msgs::Point p1,p2,p3;
            // // cout<<nextpoint_num<<endl;
            // p1.x=_path_x[nextpoint_num];
            // p2.x=_path_x[nextpoint_num+1];
            // p3.x=_path_x[nextpoint_num+2];
            // p1.y=_path_y[nextpoint_num];
            // p2.y=_path_y[nextpoint_num+1];
            // p3.y=_path_y[nextpoint_num+2];
            // p3.y=_path_y[nextpoint_num+2];
           
            // double kappa=CalculateCur(p1,p2,p3)+0.00001;                
            // ROS_DEBUG("Kappa= %lf",kappa);   
            // kappa=limitParamChangeUnit(kappa,0.0,0);
            _transfer_val=cal_transfer_val(_d_dis);            
            _theta_ss=_transfer_val*_Oss;//可以近似不变
            if (_theta_ss >= 0)
                _G = _p1_post * (_theta_ss * 180/PI) * (_theta_ss * 180/PI) + _p2_post * (_theta_ss * 180/PI) + _p3_post;//+kappa;
            else
                _G = _p1_neg * (_theta_ss * 180/PI) * (_theta_ss * 180/PI) + _p2_neg * (_theta_ss * 180/PI) + _p3_neg;//+kappa;
            _wheel_angle_degree=_theta_ss*_G*180/PI; 
                    
            if (_flag<30)
                _wheel_angle_degree=limit_wheel_val(_vehicle_wheel_angle,  _wheel_angle_degree);
            else
                _wheel_angle_degree=limit_wheel_val(_wheel_angle_degree_last, _wheel_angle_degree);
            _wheel_angle_degree_last=_wheel_angle_degree;
            _cal_wheel_angle=_wheel_angle_degree;
            _record_cnt=0;
        }       
        else                    //保持当前转角
        {
            if ((_keep_steer==1)&&(_record_cnt<2))            //记录当前转角
            {
                _keep_wheel_angle=_vehicle_wheel_angle;
                _record_cnt++;
            }
            _wheel_angle_degree=limit_wheel_val(_vehicle_wheel_angle, _keep_wheel_angle);  //限制一下方向盘转角上升度数
            _cal_wheel_angle=_wheel_angle_degree;
        }
        _flag=_flag+1;        
    }   
    /**寻找路径最近点*/
    float  icvCarlaControl::find_min_point(float _preview_x_rec,float _preview_y_rec,int *count_min)
    {
        float _preview_x_tmp;
        float _preview_y_tmp;
        float _L_min = 20.0;
        float _L;
        float _x_min = -1.0;
        float _y_min = -1.0;
        float _Oss_tmp;       
        int _count_min;
        _preview_x_tmp=_preview_x_rec;
        _preview_y_tmp=_preview_y_rec;
        float _vehicle_x_ref=Current_Point.x;
        float _vehicle_y_ref=Current_Point.y;       
        for (int _path_ii=0;_path_ii<_path_x.size();_path_ii++)
        {
            
            _L=sqrt(pow((_preview_x_tmp-_path_x[_path_ii]),2)+pow((_preview_y_tmp-_path_y[_path_ii]),2));
            if (_L<=_L_min)
            {
                _L_min = _L;
                _x_min = _path_x[_path_ii];
                _y_min = _path_y[_path_ii];
                _count_min=_path_ii;
            }
            else
            {
                _L_min = _L_min;
                _x_min = _x_min;
                _y_min = _y_min;
                _count_min=_count_min;
            }
        }    
        //TODO:增加最近索引点输出；
        *count_min=  _count_min;  
           if ((_preview_x_tmp - _vehicle_x_ref) * (_y_min - _vehicle_y_ref) - (_preview_y_tmp- _vehicle_y_ref) * (_x_min - _vehicle_x_ref) >= 0)
            _Oss_tmp = _L_min;
 
        else
            _Oss_tmp = -1.0 * _L_min; 
        cout<<"  Cte_d=  "<< _Oss_tmp<<endl;//zx                     
        return _Oss_tmp;
    } 
    /**计算公式系数值？？？*/   
    float  icvCarlaControl::cal_transfer_val(float _d_dis_rec)
    {
        float _m = 1705;            //the mass of vehicle
        float _a = 1.255;           //the distance from centre of mass to front axle
        float _b = 1.535;           //the distance from centre of mass to rear axle
        float _Cf = 116000;         //the front tyre cornering stiffness
        float _Cr = 187000;         //the rear tyre cornering stiffness
        float _denominator;
        float _molecular;
        float _T;        //分母中的参数
        float _d_dis_tmp;
        float _trans_val;
        float _vehicle_v=Current_Point.v;
        _d_dis_tmp=_d_dis_rec;
        _molecular=2 * (_a + _b - (_m * (_a * _Cf - _b * _Cr)) / ((_a + _b) * _Cf * _Cr)); 
        _T=_b - ((_a * _m * (_vehicle_v * _vehicle_v)) / (_Cr * (_a + _b))); 
        _denominator = _d_dis_tmp * (_d_dis_tmp + 2 * _T);  
        _trans_val=_molecular/_denominator;

        return _trans_val;
    }
    
    /*限制方向盘输出上下限*/
        float icvCarlaControl::limit_wheel_val(float _wheel_last_ref_rec, float _wheel_angle_degree_rec)
    {

        float _wheel_now;
        float _wheel_last;

        _wheel_now=_wheel_angle_degree_rec;
        _wheel_last=_wheel_last_ref_rec;
        if(_wheel_now>=_wheel_last)
        {        
            if((_wheel_now-_wheel_last)>_wheel_angle_degree_sec*_delta_T)
                {
                  _wheel_now=  _wheel_last+_wheel_angle_degree_sec*_delta_T;
                }
            else
                {
                  _wheel_now=_wheel_now;
                }
            if(_wheel_now>=_wheel_angle_degree_max)
                {
                    _wheel_now=_wheel_angle_degree_max;
                }
            else
                {
                    _wheel_now=_wheel_now;
                }
        }
        else
        {
            if ((_wheel_last-_wheel_now)>=_wheel_angle_degree_sec*_delta_T)
            {
                _wheel_now=_wheel_last-_wheel_angle_degree_sec*_delta_T;
            }
            else
            {
                _wheel_now=_wheel_now;
            }
            if (_wheel_now<=-_wheel_angle_degree_max)
            {
                _wheel_now=-_wheel_angle_degree_max;
            }
            else
            {
                _wheel_now=_wheel_now;
            }
        }
    return _wheel_now;
    }  


    bool icvCarlaControl::StateConstraint(float v,float angle,float t)
    {
        bool Path_flag=false;//路径安全性
        Beta=((lr-(lf*m*v*v)/(2*Ccf*(lr+lf)))/(lf+lr+(m*v*v*(lr*Ccr-lf*Ccf)/(2*Ccf*Ccr*(lr+lf)))))*angle;  
        v=60;angle=0.1;
        float beita=Beta;  
        float ceita=angle;
        v=2;
        // F_yf=u3;
        // F_yr=u4;
        //模型参数初始值：
        float A0=1.65;
        float A1=-34;
        float A2=1250,A3=3036,A4=12.8,A5=0.00501,A6=-0.02103;
        float A7=0.77394,A8=0.002289,A9=0.013442,A10=0.003709,A11=19.1656,A12=1.21356;
        float A13=6.26206;
        //车辆初始参数：
        //m=1818.2;lf=1.463;lr=1.585;
        float Q=0.3;//车轮外倾角；
        //计算两个轮子的侧偏角和垂向压力值：
        //注意魔术公式轮胎模型的车轮垂直载荷为KN，输出的侧向力为N；
        float af=(beita+lf*r/v-ceita)*180/PI;       
        float ar=(beita-lr*r/v)*180/PI;
        float F_zf=m*9.8*(lr)/(lr+lf)/2000;
        float F_zr=m*9.8*(lf)/(lf+lr)/2000;
        //建立轮胎模型：
        float C=A0;
        float D1=A1*(F_zf*F_zf)+A2*F_zf;
        float B1=A3*sin(2*atan(F_zf/A4))*(1-A5*Q)/(C*D1);
        float E1=A6*F_zf+A7;
        float Sh1=A9*F_zf+A10+A8*Q;
        float Sv1=A11*F_zf*Q+A12*F_zf+A13;
        float X1=(af+Sh1);
        float F_yf=(D1*sin(C*atan(B1*X1-E1*(B1*X1-atan(B1*X1)))))+Sv1;
        
        float D2=A1*(F_zr*F_zr)+A2*F_zr;
        float B2=A3*sin(2*atan(F_zr/A4))*(1-A5*Q)/(C*D2);
        float E2=A6*F_zr+A7;
        float Sh2=A9*F_zr+A10+A8*Q;
        float Sv2=A11*F_zr*Q+A12*F_zr+A13;
        float X2=(ar+Sh2);
        float F_yr=(D2*sin(C*atan(B2*X2-E2*(B2*X2-atan(B2*X2)))))+Sv2;
       
        
        float beita_d=-r+2*F_yf/(m*v)*cos(beita-ceita)+2*F_yr/(m*v)*cos(beita);
        float r_d=2*lf/I*F_yf*cos(ceita)-2*lr/I*F_yr;
        
        // Beta=Beta+beita_d*t;
        r=t*r_d;
        beita=t*beita_d;        
        float EE1=-1/9,EE2=0;
        EE2=abs(beita_d*EE1+Beta);
        if(abs(Beta)>15)        
            Path_flag=false;        
        else        
            Path_flag=true; 
            
       
        return  Path_flag;

    }
    //参数输入质心侧偏角，方向盘转角？？前轮转角？，车速ｖ，横摆角速度的导数ｒ
    float icvCarlaControl::tiremodel(float beita,float cetia,float v,float r)//未验证
    {  
        //模型参数初始值：
        float A0=1.65;
        float A1=-34;
        float A2=1250,A3=3036,A4=12.8,A5=0.00501,A6=-0.02103;
        float A7=0.77394,A8=0.002289,A9=0.013442,A10=0.003709,A11=19.1656,A12=1.21356;
        float A13=6.26206;
        //车辆初始参数：
        //m=1818.2;lf=1.463;lr=1.585;
        float Q=0.3;//车轮外倾角；
        //计算两个轮子的侧偏角和垂向压力值：
        //注意魔术公式轮胎模型的车轮垂直载荷为KN，输出的侧向力为N；
        float af=(beita+lf*r/v-cetia)*180/PI;
        float ar=(beita-lr*r/v)*180/PI;
        float F_zf=m*9.8*(lr)/(lr+lf)/2000;
        float F_zr=m*9.8*(lf)/(lf+lr)/2000;
        //建立轮胎模型：
        float C=A0;
        float D1=A1*(F_zf*F_zf)+A2*F_zf;
        float B1=A3*sin(2*atan(F_zf/A4))*(1-A5*Q)/(C*D1);
        float E1=A6*F_zf+A7;
        float Sh1=A9*F_zf+A10+A8*Q;
        float Sv1=A11*F_zf*Q+A12*F_zf+A13;
        float X1=(af+Sh1);
        float F_yf=(D1*sin(C*atan(B1*X1-E1*(B1*X1-atan(B1*X1)))))+Sv1;

        float D2=A1*(F_zr*F_zr)+A2*F_zr;
        float B2=A3*sin(2*atan(F_zr/A4))*(1-A5*Q)/(C*D2);
        float E2=A6*F_zr+A7;
        float Sh2=A9*F_zr+A10+A8*Q;
        float Sv2=A11*F_zr*Q+A12*F_zr+A13;
        float X2=(ar+Sh2);
    }

    /******数据写出*******/
    void icvCarlaControl::data2file()
    {

        ofstream out;
        out.open("/home/icv/follow_carla/3.26test.txt",std::ios::out | std::ios::app);   
        pathdeal();       
        out<<setiosflags(ios::fixed)<<setprecision(3)<<ctedata[0]<<" "<<ctedata[1]<<" "<<ctedata[2]<<endl;     
      
        out.close();  
    }
    
    // /*函数输入：当前车辆坐标，前轮偏角，车辆速度v，航向角phi，迭代周期T
    // 函数输出：坐标（x,y,theta)*/         
    // Pathpoint icvCarlaControl::Vehiclemodeupdate1(Pathpoint coordinate,float delta_f,float T)//不准确
    // {
    //     //车辆动力学模型
        
    //     Pathpoint coordinate_back;
    //     X=coordinate.x;
    //     Y=coordinate.y;
    //     phi=coordinate.theta/180*PI;
    //     x_dot=coordinate.v*cos(delta_f)+0.00001;
    //     y_dot=coordinate.v*sin(delta_f);    
    //     cout<<"1x_dot  "<<x_dot<<" y_dot "<<y_dot<<" phi_dot "<<phi_dot<<endl;

    //     phi_dot=coordinate.phi_dot;
    //     y_dot=y_dot+T*(-x_dot*phi_dot+2*(Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)+Ccr*(lr*phi_dot-y_dot)/x_dot)/m);//y_dot 初值为0？？
    //     x_dot=x_dot+T*(y_dot*phi_dot+2*(Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/m);//x_dot初值为0？？    
        

    // t+lf*phi_dot)/x_dot)-2*lr*Ccr*(lr*phi_dot-y_dot)/x_dot)/I);//phi_dot 初值为0
    //     phi=phi+T*phi_dot;    
    //     // Y=Y+T*(x_dot*sin(phi)+y_dot*cos(phi));
    //     // X=X+T*(x_dot*cos(phi)-y_dot*sin(phi));
    //     coordinate.theta=coordinate.theta+T*phi_dot;
    //     coordinate.y=coordinate.y+T*(x_dot*sin(phi)+y_dot*cos(phi));
    //     coordinate.x=coordinate.x+T*(x_dot*cos(phi)-y_dot*sin(phi));
    //     // cout<<(y_dot*phi_dot+2*(Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/m)<<endl;
    //     cout<<"2x_dot  "<<x_dot<<" y_dot "<<y_dot<<" phi_dot "<<phi_dot<<endl;
    //     // cout<<(2*(Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/m)<<endl;
    //     // cout<<Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot)<<endl;
    //     coordinate.x_dot=x_dot;
    //     coordinate.y_dot=y_dot;
    //     // coordinate_back.x=X;
    //     // coordinate_back.y=Y;
    //     // coordinate_back.theta=phi;
    //     if(coordinate.theta>180)
    //     {
    //         coordinate.theta=coordinate.theta-360;
    //     }     
    //     return coordinate;

    // }

    // Pathpoint icvCarlaControl::Vehiclemodeupdate2(Pathpoint coordinate,float delta_f,float T)
    // {
    //     // Pathpoint coordinate;
    //     //车辆运动学模型
    //     //函数输入前轮转角，车辆速度v，航向角phi
    //     //函数输出：x，y，headingangle     
    //     coordinate.x= coordinate.x   +coordinate.v*cos((coordinate.theta+Beta)/180*PI)*T;    
    //     coordinate.y= coordinate.y   +coordinate.v*sin((coordinate.theta+Beta)/180*PI)*T;
    //     coordinate.theta=coordinate.theta/180*PI+(cos(Beta)*coordinate.v/(lf+lr))*tan(delta_f)*T;     
    //     coordinate.theta=coordinate.theta*180/PI;
    //     if(coordinate.theta>180)
    //     {
    //         coordinate.theta=coordinate.theta-360;
    //     }     
        
        
    //     return coordinate;

    // }
    // // float FollowPath(Pathpoint coordinate,float v)
    // // {
    // //     //autoware purepsuit  algorithm





    /*******偏差计算********/
    void icvCarlaControl::pathdeal()
    {    
        //"pathdeal to calculte cte_d  cte_a "
        float cte_D=0;
        float cte_A;
        int plo_i=0;//曲线拟合的索引点
        float ctd=100;   
        float dis_a=0;
        int m=_path_x.size();//轨迹点长度   
        float x_len=0;    
        float x_plo_start=0;
        float d_ditance=0;//x的离散补偿
        float sig_d=0;
        int sgn=0;
        /*曲线拟合*/
        VectorXd x_veh(5);
        VectorXd y_veh(5);
        for(int i=0;i<m;i++)
        {
            dis_a=sqrt(pow((Current_Point.x-Waypoints[i].x),2)+(pow((Current_Point.y-Waypoints[i].y),2)));        
            if(dis_a>=ctd)
                { 
                ctd=ctd;
                plo_i=plo_i;
                }
            else
                { 
                ctd=dis_a;
                plo_i=i;
                } 
        }  
        if(plo_i<4)
        {
            
            float plox[5]={Waypoints[0].x,Waypoints[1].x,Waypoints[2].x,Waypoints[3].x,Waypoints[4].x};
            float ploy[5]={Waypoints[0].y,Waypoints[1].y,Waypoints[2].y,Waypoints[3].y,Waypoints[4].y};
            for(int j=0;j<5;j++)
            {
                x_veh[j]=plox[j];
                y_veh[j]=ploy[j];
            }       
            // f1 = np.polyfit(plox, ploy, 3)      
            // p1 = np.poly1d(f1)          
            x_len=Waypoints[3].x-Waypoints[0].x;    
            x_plo_start=Waypoints[0].x;
        }
        else
        {
            float plox[5]={Waypoints[plo_i-2].x,Waypoints[plo_i-1].x, Waypoints[plo_i].x,Waypoints[plo_i+1].x,Waypoints[plo_i+2].x};
            float ploy[5]={Waypoints[plo_i-2].y,Waypoints[plo_i-1].y, Waypoints[plo_i].y,Waypoints[plo_i+1].y,Waypoints[plo_i+2].y};
            for(int j=0;j<5;j++)
            {
                x_veh[j]=plox[j];
                y_veh[j]=ploy[j];
            }      
            // f1 = np.polyfit(plox, ploy, 3)      
            // p1 = np.poly1d(f1)      
            x_len=Waypoints[plo_i+1].x-Waypoints[plo_i-1].y;
            x_plo_start=Waypoints[plo_i-1].x;
        }
        auto coeffs = polyfit(x_veh, y_veh, 3); //拟合三次多项式
        
        for(int i=0;i < coeffs.size();i++)
        {
            // ROS_INFO("coeffs[%d]:%f",i,coeffs[i]);
        }
        if(x_len>0)      
            d_ditance=0.1;     
        else        
            d_ditance=-0.1;  
        float i_plo=0;
        float x_plo=0;
        float y_plo=0;
        float d_plo=0;
        while(abs(i_plo)<abs(x_len))
        {
            x_plo=x_plo_start+i_plo;
            y_plo=polyeval(coeffs,x_plo);            
            d_plo=sqrt(pow((Current_Point.x-x_plo),2)+(pow((Current_Point.y-y_plo),2)));
            if(d_plo>=ctd)            
                ctd=ctd;
            else
                ctd=d_plo;
            i_plo=i_plo+d_ditance;
        }  
        sig_d=((Waypoints[plo_i].x-Current_Point.x)*(Waypoints[plo_i+1].y-Current_Point.y))-((Waypoints[plo_i].y-Current_Point.y)*(Waypoints[plo_i+1].x-Current_Point.x));  
        if(sig_d>=0)    
        {
            if(sig_d>0)            
                sgn=1; 
            else
                sgn=0;             
        }
        else
        {
            sgn=-1;
        }         
        cte_D=ctd*sgn;    
        cte_A=0-(fmod(((Waypoints[plo_i].theta-Current_Point.theta)/180.0*PI+PI),(2*PI))-PI)/PI*180.0;
        ctedata[0]=cte_D;
        ctedata[1]=cte_A;    
    }

    //获取拟合的参数曲线在x时的y值
    double icvCarlaControl::polyeval(Eigen::VectorXd coeffs, double x)     
    {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
    }

    //多项式拟合的一个函数,返回拟合的参数曲线系数
    Eigen::VectorXd icvCarlaControl::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
    {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++) 
    {
        for (int i = 0; i < order; i++) 
        A(j, i + 1) = A(j, i) * xvals(j);
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
    }
    //计算曲率三点求解法
    double icvCarlaControl::CalculateCur(geometry_msgs::Point P1,geometry_msgs::Point P2,geometry_msgs::Point P3)
    {
        
        double curvity_pre=0;//前一时刻曲率
        double curvity=0;//当前曲率
        double curvity_R=0;//当前转向半径
        double speed_pre=0;//前一时刻车速
        double speed_cur=0;//当前车速
        double acc=0;
        double acc_pre=0;//前一时刻加速度    
      
        
        //计算曲率部分，使用正弦定理 a/sinA = 2R
        if(P1.x == P2.x == P3.x)//三点横坐标相同，即共线，直接标记曲率为0
        {
            curvity = 0;
        }
        else
        {
            if(P1.x==P2.x&&P1.y==P2.y||P1.x==P3.x&&P1.y==P3.y||P2.x==P3.x&&P2.y==P3.y)
            {
                // cout<<"path have same point!"<<endl;
            }
            else
            {
                double dis1,dis2,dis3;
                double cosA,sinA,dis;
                dis1 = sqrt((P1.x - P2.x)*(P1.x - P2.x) + (P1.y - P2.y)*(P1.y - P2.y));
                dis2 = sqrt((P1.x - P3.x)*(P1.x - P3.x) + (P1.y - P3.y)*(P1.y - P3.y));
                dis3 = sqrt((P2.x - P3.x)*(P2.x - P3.x) + (P2.y - P3.y)*(P2.y - P3.y));
                dis = dis1*dis1 + dis3*dis3 - dis2*dis2;
                cosA = dis/(2*dis1*dis3)+0.0001;//余弦定理求角度
                if(cosA>1)
                {cosA=1;}
                sinA = sqrt(1 - cosA*cosA);//求正弦
                curvity_R = 0.5*dis2/sinA;//正弦定理求外接圆半径
                curvity = 1/curvity_R;//半径的倒数是曲率，半径越小曲率越大                        
                            
            }
        }  
        return curvity;                      
    }
    //平滑度判断标准1.方向盘：最小半径,曲率变化率
    //速度，点点之间的加速度是否超过最大值，加速度变化率。
    bool icvCarlaControl::PathSmoothCheck()
    {
        geometry_msgs::Point P1;
        geometry_msgs::Point P2;
        geometry_msgs::Point P3;
        double curvity_pre=0;//前一时刻曲率
        double curvity=0;//当前曲率
        double curvity_R=0;//当前转向曲率
        double speed_pre=0;//前一时刻车速
        double speed_cur=0;//当前车速
        double acc=0;
        double acc_pre=0;//前一时刻加速度        
        for(int i=0;i<(Waypoints_size-2);i++)
        {
            
            try
            {   //取点 P1、P2、P3是位置信息
                P1.x = _path_x[i];
                P2.x = _path_x[i+1];
                P3.x = _path_x[i+2];
                P1.y = _path_y[i];
                P2.y = _path_y[i+1];
                P3.y = _path_y[i+2];
                curvity=CalculateCur(P1,P2,P3);
                curvity_R = 1/curvity;//半径的倒数是曲率，半径越小曲率越大                        
                if(curvity_R<R_min)//转向半径小于最小半径时返回错误。
                {
                    ROS_DEBUG("error:Path Curvity is too big!!!");
                    ROS_DEBUG("curvity_R=%lf",curvity_R);
                    return false;  
                }                    
                
            }
            catch(tf::TransformException &ex)//处理意外情况
            {
                ROS_ERROR("%s",ex.what());
                cout<<"三点取点失败\n";
            }
            double cur_dt=curvity-curvity_pre;
            if(cur_dt*L*K>30)
            {
                ROS_DEBUG("error:Path Curvity change too much!!!");
                ROS_DEBUG("cur_dt*L*K =%lf",cur_dt*L*K);
                return false;
            }
            curvity_pre=curvity;
            /***速度平滑度测试**///删除？？
            speed_cur=Waypoints[i].v;
            acc=(speed_cur-speed_pre)/dt;
            if(abs(acc)>Acc_max)
            {
                ROS_DEBUG("error:Acc is too MAX!!!!");
                return false;
            }
            if(abs(acc-acc_pre)>jerk_max)
            speed_pre=speed_cur;
        }
        return true;
    }
    //小鹏速度y预测模块，输入当前车速为km/h,预瞄距离   预瞄速度,返回目标车速
    double icvCarlaControl::Velocitytopredict(float velocity,float distance_pre,float velocity_pre)
        {
            float V[20];//存储速度
            float S[20];//累积位移
            V[0]=velocity*3.6;
            float Acc_G1,out_speed;
            int Np=20;//预测周期20
            int T_G1=0.1;//步长0.1s
            int i_pre=0;
            for(int i=0;i<(Np-1);i++)//速度预测
            {
                Acc_G1=0.0003*pow(V[i],3)-0.016*V[i]*V[i]+0.2547*V[i]-0.59;//G1加速度变化拟合方程
                V[i+1]=V[i]+Acc_G1*T_G1;  
                S[i+1]=(S[i]+V[i+1]*V[i+1]-V[i]*V[i])/(2*Acc_G1);                    
            }
            for(int i=0;i<Np;i++)
            {
                if(distance_pre<S[i])//预瞄距离
                {
                    i_pre=i;
                    break;
                }   
            }
            if(velocity_pre<V[i_pre])//加速工况
            {
                out_speed=velocity_pre;
            }
            else
            {
                ROS_INFO("target speed is too max!!! ");
                out_speed=V[i_pre];
            }
        return out_speed;
        }
    void icvCarlaControl::run_speedcontrol(double *out_throttle,double *out_brake,double tar_v)
    {
        double v_error;
        if(nextpoint_num>_path_x.size())
        {
            v_error=0-Current_Point.v*3.6;
            v_error=limitParamChangeUnit(v_error,0,-4);
            *out_brake= PIDcontrol(v_error,_v_pid);
            *out_brake=limitParamChangeUnit(*out_brake,0.8,0); 
        }
        else
        {
            v_error=tar_v-Current_Point.v*3.6;
            v_error=limitParamChangeUnit(v_error,4,-4);         
            *out_throttle= PIDcontrol(v_error,_v_pid);
            if(!setVehiclestauts_flag)
            {
                *out_throttle=0;
            }
            *out_throttle=limitParamChangeUnit(*out_throttle,0.8,0); 
        }
    }
    double icvCarlaControl::limitParamChangeUnit(double param,double max_param ,double min_param)
    {
        if(param>max_param)
        {param=max_param;}
        else if (param<min_param)
        {
            param=min_param;
        }
        else{
            param=param;
        }

        return param;
    }
    double icvCarlaControl::PIDcontrol(double error,PID pid )
    {            
        error_p = error;
        if(abs(error_p<2))
        { error_i += error_p;}
        else
        {
            error_i=0;
        }
        error_d = error_p - error_dp;
        error_dp = error_p;
        out_insturct= pid.Kp * error_p + pid.Ki * error_i + pid.Kd * error_d;
        return out_insturct;
    }

}//icv

 
  
  
  

  












