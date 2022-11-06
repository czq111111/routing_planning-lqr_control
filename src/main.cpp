#include "read_data.h"
#include "refercenceline_smooth.h"
#include "matching_line.h"
#include "reference_line.h"
#include "extract_point.h"

#include "LQR_control.h"
#include "struct_common.h"

#include <iostream>
#include <memory>


int main() {
//*******************************读取参考线文件里的数据******************************************//
    std::vector<std::pair<double, double>> xy_points;
    auto data = std::make_unique<readDate>();
    std::vector<std::pair<double, double>> xy_points1;
    data->readTxt(xy_points1);
    for(int i=0;i < (int)xy_points1.size();i++)
    {    
        xy_points.push_back(std::make_pair(xy_points1[i].first, xy_points1[i].second));
    }
//******************************计算heading和kappa*******************************************//
    std::vector<double> headings, accumulated_s, kappas, dkappas;
    std::unique_ptr<ReferenceLine> reference_line = std::make_unique<ReferenceLine>(xy_points);
    //访问计算出来的headings,kappas并保存到common.h的结构体中
    reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas,
                                     &dkappas);
//********************************************************************************************//
//**********************把文件在的点坐标xy_points1转换到frenet坐标系下***************************//
#if 1
    MatchingToFrenet fre;
    std::vector<std::pair<double, double>> xy_set;
    xy_set.push_back(std::make_pair(xy_points.at(0).first, xy_points.at(0).second));
    fre.tofrenetreferenceLine(xy_points,xy_set);
    fre.tofrenet(&headings,&kappas);
    
#endif
//**************************提取refercenceline未平滑的初值*************************************//
#if 1
    ExtractPoint ext;
    ext.axtractglobalpath(xy_points);
    int host_match_point_index ;
    host_match_point_index = fre.match_point_index_set(0,0);
    ext.axtractrefercenceline(host_match_point_index);
#endif
//*****************************************************************************************

//**********************把文件在的点坐标xy_points1进行曲线平滑处理*******************************//
#if 1
    std::vector<std::pair<double, double>> refercence_point;
    refercence_point.clear();//清理之前的数据
    for(int j=0;j<(int)ext.referenceline_x_init.size();j++)
    { 
        refercence_point.push_back(std::make_pair(ext.referenceline_x_init[j], ext.referenceline_y_init[j]));
    }
    refercenceline_smooth test;
    test.ReferenceLine(refercence_point);
    test.line_smooth(100, 50, 3, 0.2, 0.2, 0.2, 0.2);
    // //平滑后的曲线
    std::vector<double> x_smooth, y_smooth;
    x_smooth.resize(ext.referenceline_x_init.size());
    y_smooth.resize(ext.referenceline_x_init.size());
    x_smooth.clear();
    y_smooth.clear();
    for(int j=0;j<(int)ext.referenceline_x_init.size();j++)
    {
        x_smooth.push_back(test.ref_x.at(j));
        y_smooth.push_back(test.ref_y.at(j));
    }  
#endif
//******************************LQR_control控制********************************************//
    //初始化
    std::vector<planning_trajectory> Trajectory_;
    planning_trajectory pt;
    LQR_control lqr;
    ControlCmd cmd;
//建立跟踪轨迹(平滑曲线)
//*********************************************************************
    for (int a = 0; a < (int)xy_points.size(); a++)                 //*
    {                                                               //*
            pt.trajectory_x = x_smooth[a];//x_smooth[a];            //*
            pt.trajectory_y = y_smooth[a];///y_smooth[a];           //*
            pt.trajectory_accel = 0;                                //*
            pt.trajectory_speed = 0;                                //*
            pt.trajectory_heading = headings[a];                    //*
            pt.trajectory_kappa = kappas[a];                        //*
            pt.trajectory_time = 0;                                 //*
            Trajectory_.push_back(pt);                              //*
    }                                                               //*
//*********************************************************************
//创建车辆初始位置
//***************************************************
    location_info car;                            //*
    car.host_x = xy_points.at(40).first;//38.83;  //*
    car.host_y = xy_points.at(40).second;//3.15;  //*
    car.host_heading_xy= headings[40];//3.14159;  //*
    car.host_yawrate= 0;                          //*
    car.host_speed= 30;                           //*
    car.host_vx= 0;                               //*
    car.host_vy= 0;                               //*
    car.host_acceleration= 0;                     //*
    car.host_ax= 0;                               //*
    car.host_ay= 0;                               //*
//***************************************************
//建立跟踪轨迹(参考线)
#if  0
//*********************************************************************
    for (int a = 0; a < (int)xy_points.size(); a++)                 //*
    {                                                               //*
            pt.trajectory_x = xy_points[a].first;//x_smooth[a];     //*
            pt.trajectory_y = xy_points[a].second;//y_smooth[a];    //*
            pt.trajectory_accel = 0;                                //*
            pt.trajectory_speed = 0;                                //*
            pt.trajectory_heading = headings[a];                    //*
            pt.trajectory_kappa = kappas[a];                        //*
            pt.trajectory_time = 0;                                 //*
            Trajectory_.push_back(pt);                              //*
    }                                                               //*
//*********************************************************************
#endif
     //自身车辆运动轨迹
    std::vector<double> x0, y0;
    x0.resize(xy_set.size());
    y0.resize(xy_set.size());
    x0.clear();
    y0.clear();
    //获取全局路径
    std::vector<double> x2, y2;
    //全局路径+平滑+LQR控制
    for(int i=0;i<600;i++)
    {
    //**********************全局路径****************************//
        x2.resize(xy_points.size());                          //
        y2.resize(xy_points.size());                          //
        x2.clear();                                           //
        y2.clear();                                           //
        for (int j = 0; j < (int)xy_points.size(); j++) {     //
            x2.push_back(xy_points.at(j).first);              //
            y2.push_back(xy_points.at(j).second);             //
        }                                                     //
        plt::cla();                                           //
        plt::plot(x2, y2,"-.r");                              //
    //********************************************************//
        //*********************lqr*************************************
        double steer = lqr.ComputeControlCommand(car,Trajectory_,cmd);
        car.host_x += car.host_speed *cos(car.host_heading_xy)*0.01;
        car.host_y += car.host_speed*sin(car.host_heading_xy)*0.01;
        car.host_heading_xy = car.host_heading_xy + car.host_speed/2.947*tan(steer)*0.01;
        car.host_yawrate= 0;
        car.host_speed= 30;
        car.host_vx= 0;
        car.host_vy= 0;
        car.host_acceleration= 0;
        car.host_ax= 0;
        car.host_ay= 0;
        //车的状态
        xy_set.push_back(std::make_pair(car.host_x, car.host_y));
        //******************自身车辆运动轨迹***************************************//
        x0.push_back(xy_set.at(i).first);                                       //
        y0.push_back(xy_set.at(i).second);                                      //
        plt::plot(x0, y0,"bo");                                                 //
        //************************************************************************//
        //把文件在的点坐标xy_points1转换到frenet坐标系下
        fre.pre_match_point_index_set.clear();
        fre.tofrenetreferenceLine(xy_points,xy_set);
        fre.tofrenet(&headings,&kappas);
        //*************************************************************************//  
        //提取refercenceline未平滑的初值
        host_match_point_index = fre.pre_match_point_index_set.at(i);
        //cout<<" host_match_point_index: "<< host_match_point_index<<endl;
        ext.axtractglobalpath(xy_points);
        ext.referenceline_x_init.clear();
        ext.referenceline_y_init.clear();
        ext.axtractrefercenceline(host_match_point_index);
        //*************************提取的点****************************************//
        std::vector<double> x4, y4;
        x4.resize(ext.referenceline_x_init.size());
        y4.resize(ext.referenceline_x_init.size());
        x4.clear();
        y4.clear();
        for (int a = 0; a < (int)ext.referenceline_x_init.size(); a++) {
            x4.push_back(ext.referenceline_x_init.at(a));
            y4.push_back(ext.referenceline_y_init.at(a));
            //cout<<"ext.referenceline_x_init:  "<< ext.referenceline_x_init[a]<<endl;
        }
        //提取128个点
        //plt::plot(x4, y4);
        //********************************************************************************//
        //*******************************将提取的180个点进行平滑******************************//
        refercence_point.clear();
        for(int j=0;j<(int)ext.referenceline_x_init.size();j++)
        {
            refercence_point.push_back(std::make_pair(x4.at(j), y4.at(j)));
        }
        //cout<<"refercence_point.size: "<<refercence_point.size()<<endl;
        test.ReferenceLine(refercence_point);
        test.ref_x.clear();
        test.ref_y.clear();
        test.line_smooth(100, 50, 3, 0.2, 0.2, 0.2, 0.2);
        //平滑的曲线
        x_smooth.resize(test.ref_x.size());
        y_smooth.resize(test.ref_y.size());
        x_smooth.clear();
        y_smooth.clear();
        for(int j=0;j<(int)ext.referenceline_x_init.size();j++)
        {
            x_smooth.push_back(test.ref_x.at(j));
            y_smooth.push_back(test.ref_y.at(j));
        }
        plt::plot(x_smooth, y_smooth);//平滑曲线
        //**********************************************************************************//
        plt::pause(0.001);
    } 
    plt::show();
//******************************************************************************************// 

//****************************参考线/自身车辆运动轨迹的曲线**************************//
#if 0
    //自身车辆运动轨迹
    std::vector<double> x0, y0;
    x0.resize(xy_set.size());
    y0.resize(xy_set.size());
    x0.clear();
    y0.clear();
    //参考轨迹
    std::vector<double> x2, y2;
    x2.resize(xy_points.size());
    y2.resize(xy_points.size());
    x2.clear();
    y2.clear();
    for (int i = 0; i < (int)xy_points.size(); i++) {//(int)xy_points.size()

        x2.push_back(xy_points.at(i).first);
        y2.push_back(xy_points.at(i).second);
        plt::cla();
        plt::plot(x2, y2,"-.r");//参考轨迹

        x0.push_back(xy_set.at(i).first);
        y0.push_back(xy_set.at(i).second);
        plt::plot(x0, y0,"bo");//车身运动

        // plt::legend();
        plt::pause(0.01);
    }
        plt::show();
#endif
//******************************************************************************************//
    return 0;
}