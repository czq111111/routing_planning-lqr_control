# routing_planning-lqr_control
第一部分(轨迹生成)
首先导入全局地图
把全局地图的轨迹点在Globa坐标系下投影到frenet坐标系下
再以车辆投影到frenet坐标系下
提取全局地图轨迹点在自身车辆后30个点前150个点(frenet坐标系下)
对提取的全局地图的轨迹点进行平滑(五次多项式)
//***************************************************************************************************************//
#Part I (Track Generation)
First import the global map
The trajectory points of the global map are projected under the global coordinate system to the frenet coordinate system
Then project the vehicle to the frenet coordinate system
Extract 150 points in front of 30 points behind the vehicle (under frenet coordinate system)
Smoothing the trajectory points of the extracted global map (quintic polynomial)
//***************************************************************************************************************//
第二部分(LQR控制)
Part II (LQR control)

第三部分(验证)
使用了matplotlib画图工具
验证跟踪效果
//*********************************//
Part III (Verification)
Matplotlib drawing tool is used
Verify the tracking effect
//********************************//
