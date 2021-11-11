   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid1(new pcl::PointCloud<pcl::PointXYZ>);
  
  float jj = pointmin.x +0.3f;
    for (size_t i = 0; i < (pointmax.x - pointmin.x)/0.001; i++)
    {

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud1);         
        pass.setFilterFieldName("x");      
        pass.setFilterLimits(jj,jj + 0.001);    
        pass.filter(*cloudmid); 
        pcl::getMinMax3D(*cloudmid,p1,p2);
        index.push_back(p1);
        jj = jj + 0.001;
         for (size_t j = 0; j <cloudmid->points.size(); j++)
        {
           if (p1.y == cloudmid->points[j].y)
            {
                cloudmid1->points.push_back(cloudmid->points[j]);
            }
        }
    }
    float k = pointmin.y;
    for (size_t i = 0; i < (pointmax.y - pointmin.y)/0.001; i++)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud1);         
        pass.setFilterFieldName("y");      
        pass.setFilterLimits(k,k + 0.001);    
        pass.filter(*cloudmid); 
        pcl::getMinMax3D(*cloudmid,p1,p2);
        index.push_back(p2);
        k = k + 0.001;
         for (size_t j = 0; j <cloudmid->points.size(); j++)
        {
           if (p2.x == cloudmid->points[j].x)
            {
                cloudmid1->points.push_back(cloudmid->points[j]);
            }
        }
    }