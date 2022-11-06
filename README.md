# routing_planning-lqr_control
第一部分(轨迹生成)
首先导入全局地图
把全局地图的轨迹点在Globa坐标系下投影到frenet坐标系下
再以车辆投影到frenet坐标系下
提取全局地图轨迹点在自身车辆后30个点前150个点(frenet坐标系下)
对提取的全局地图的轨迹点进行平滑(五次多项式)

第二部分(LQR控制)

第三部分(验证)
使用了matplotlib画图工具
验证跟踪效果
