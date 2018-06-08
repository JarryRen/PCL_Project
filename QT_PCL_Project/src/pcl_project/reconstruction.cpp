#include "pcl_project/reconstruction.h"

gp::reconsruction::reconsruction()
{
  get_config();
}

void gp::reconsruction::run(pcl::PointCloud<PointT>::Ptr cloud, pcl::PolygonMesh &mesh)
{
    pcl::PointCloud< PointT >::Ptr cloud_mls(new pcl::PointCloud< PointT >);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    movingLeastSquares(cloud,cloud_mls);
    normlasConsistency(cloud_mls, cloud_with_normals);
    poisson(cloud_with_normals, mesh);
    removeRedundantPolygon(cloud_mls, mesh);
    meshColoring(mesh, cloud);
}

void gp::reconsruction::pcd_to_mesh(pcl::PointCloud< PointT >::Ptr cloud, pcl::PolygonMesh& mesh, int method)
{
    pcl::PointCloud< PointT >::Ptr cloud_mls(new pcl::PointCloud< PointT >);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    movingLeastSquares(cloud,cloud_mls);
    if(method == 1){
        normlasConsistency(cloud_mls, cloud_with_normals);
        poisson(cloud_with_normals, mesh);
        removeRedundantPolygon(cloud_mls, mesh);
        meshColoring(mesh,cloud);
    }
    if(method == 2){

    }
}

void gp::reconsruction::movingLeastSquares(pcl::PointCloud< PointT >::Ptr cloud, pcl::PointCloud< PointT >::Ptr cloud_mls)
{
    pcl::PointCloud<PointT>::Ptr cloud_grid(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(leaf_size,leaf_size,leaf_size);
    grid.setInputCloud(cloud);
    grid.filter(*cloud_grid);

    pcl::PointCloud<PointT>::Ptr cloud_sor_filtered(new pcl::PointCloud<PointT>);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_grid);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_sor_filtered);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_sor_filtered);

    pcl::MovingLeastSquares<pcl::PointXYZRGB,pcl::PointXYZRGB> mls;
    mls.setInputCloud(cloud_sor_filtered);
    mls.setComputeNormals(true);
    mls.setPolynomialFit(true);//启用多项式你拟合
    mls.setPolynomialOrder(2);//3 拟合阶数
    mls.setSearchMethod(tree);
    mls.setSearchRadius(mls_radius);//值越大，平滑范围越大，点越多
    /*
    mls.setUpsamplingMethod(pcl::MovingLeastSquares< pcl::PointXYZRGB, pcl::PointXYZRGB >::VOXEL_GRID_DILATION);
    mls.setDilationVoxelSize(1);
    mls.setDilationIterations(2);
    */
    mls.process(*cloud_mls);
}

void gp::reconsruction::normlasConsistency(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                           pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals)
{
    //设置中心点，法向量指向物体中心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud,centroid);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointT,pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(0.01);
    ne.setViewPoint(centroid[0],centroid[1],centroid[2]);
    ne.compute(*cloud_normals);

    //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloud,*cloud_normals,*cloud_with_normals);

    //法向量一致
    //邻居统计
    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree_costs;
    kdtree_costs.setInputCloud(cloud_with_normals);
    double neighbor_radius = 0.01;
    std::vector<int> neighbors_idx;
    std::vector<float> neighbors_distance;

    std::vector< std::vector<int> > neighbors(cloud_with_normals->size());
   for(int i=0 ; i < cloud_with_normals->size() ; i++)
   {
       if( kdtree_costs.radiusSearch(cloud_with_normals->points[i],
                                      neighbor_radius, neighbors_idx,neighbors_distance ) > 0 )
       {
           neighbors[i].insert(neighbors[i].begin(), neighbors_idx.begin()+1, neighbors_idx.end());
       }
   }

   //成本计算 对每个点
   std::vector< std::vector<float> > costs(cloud_with_normals->size());
   bool cost_method = true;
   for(int i=0; i<costs.size(); i++)
   {
       costs[i].resize(neighbors[i].size() );
        //计算cost
        for(int j=0; j< costs[i].size();j++)
        {
            float f;

            if(cost_method == false)
            {
                //cost = | d.normal1 + d.normal2|    d为p->q的单位向量
                pcl::PointXYZRGBNormal cost_d;
                float x,y,z;
                x = cloud_with_normals->points[i].normal_x - cloud_with_normals->points[neighbors[i][j] ].normal_x;
                y = cloud_with_normals->points[i].normal_y - cloud_with_normals->points[neighbors[i][j] ].normal_y;
                z = cloud_with_normals->points[i].normal_z - cloud_with_normals->points[neighbors[i][j] ].normal_z;

                cost_d.normal_x = x/(fabs(x*x + y*y + z*z) );
                cost_d.normal_y = y/(fabs(x*x + y*y + z*z) );
                cost_d.normal_z = z/(fabs(x*x + y*y + z*z) );

                f = fabs(cost_d *  cloud_with_normals->points[i] + cost_d * cloud_with_normals->points[neighbors[i][j] ] );
            }
            else
                //cost = 1- | normal1 . normal2|
                f = 1.0 - fabs( cloud_with_normals->points[i] * cloud_with_normals->points[neighbors[i][j] ] );

            costs[i][j] = f;
        }
   }

   std::vector<int> is_visited(cloud_with_normals->size(), 0 );

   std::vector<int> nearby;
   int first = 0;
   is_visited[first] = 1;

   //预留空间，不添加元素
   nearby.reserve(neighbors[first].size());
  // nearby.insert(nearby.begin(),neighbors[first].begin(), neighbors[first].end());
   for(int i=0; i < neighbors[first].size(); i++)
   {
       nearby.push_back(neighbors[first][i]);
   }

   float cost;
   float lowest_cost;
   int cheapest_nearby = 0;//准备连接的权值最佳点
   int connected_visited = 0;//被连接的已访问点

   while(nearby.size() > 0 )
   {
       int point_nearby,point_neighbor;
       lowest_cost = 1.0e+100;
       //下一个连接的点选择，访问队列中每个点及其邻接点的成本
       for(int i = 0; i < nearby.size(); i++)
       {
           point_nearby = nearby[i];

           for(int j = 0;  j < neighbors[point_nearby].size(); j++)
           {
               point_neighbor = neighbors[point_nearby][j];
               //在已经被访问中的点选择连接
               if(is_visited[point_neighbor])
               {
                   cost = costs[point_nearby][j];
                   if(cost < lowest_cost)
                   {
                       lowest_cost = cost;
                       cheapest_nearby = point_nearby;
                       connected_visited = point_neighbor;
                       if(lowest_cost < 0.05)//两个阈值，直接连接
                       {
                           i=nearby.size();
                           break;
                       }
                   }
               }
           }
       }

       if( ( cloud_with_normals->points[cheapest_nearby]  *  cloud_with_normals->points[connected_visited] ) < 0 )
       {
           cloud_with_normals->points[cheapest_nearby].normal_x *= -1;
           cloud_with_normals->points[cheapest_nearby].normal_y *= -1;
           cloud_with_normals->points[cheapest_nearby].normal_z *= -1;
       }

       is_visited[cheapest_nearby] = 1;
       std::vector<int>::iterator iter;
       iter = find(nearby.begin(),nearby.end(),cheapest_nearby);
       nearby.erase(iter);

       //开始下一段，未被访问的点
       for(int j=0; j < neighbors[cheapest_nearby].size();j++)
       {
           point_neighbor = neighbors[cheapest_nearby][j];
           if(is_visited[point_neighbor] == 0)
           {
               std::vector<int>::iterator iter2;
               //在接下来的队列中没有 就加入
               iter2=find(nearby.begin(),nearby.end(),point_neighbor);
               //未找到
               if( iter2 == nearby.end() )
               {
                   nearby.push_back(point_neighbor);
               }
           }
       }
   }
   nearby.clear();
}


void gp::reconsruction::poisson(pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr cloud_with_normals, pcl::PolygonMesh& mesh)
{
  pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
  poisson.setInputCloud(cloud_with_normals);

  poisson.setDepth(depth);//
  poisson.setSolverDivide(solver_divide);
  poisson.setIsoDivide(iso_divide);
  
  poisson.setConfidence(false);
  poisson.setManifold(false);
  poisson.setOutputPolygons(false);
  
  poisson.reconstruct(mesh);
}

void gp::reconsruction::removeRedundantPolygon(pcl::PointCloud<PointT>::Ptr cloud_mls, pcl::PolygonMesh &mesh)
{
    //去除冗余曲面
    pcl::PointCloud<PointT> cloud_mesh;
    pcl::fromPCLPointCloud2(mesh.cloud,cloud_mesh);

    std::vector<pcl::Vertices>::iterator polygon_itr;
    std::vector<int> polygon_perimeter_radius(mesh.polygons.size(),0);//三角形周长

    for(polygon_itr = mesh.polygons.begin(); polygon_itr != mesh.polygons.end(); polygon_itr++)
    {
        float polygon_perimeter;
        float x,y,z;
        x = sqrt( pow(cloud_mesh.points[polygon_itr->vertices[0]].x - cloud_mesh.points[polygon_itr->vertices[1]].x , 2 ) +
                pow(cloud_mesh.points[polygon_itr->vertices[0]].y - cloud_mesh.points[polygon_itr->vertices[1]].y , 2 ) +
                pow(cloud_mesh.points[polygon_itr->vertices[0]].z - cloud_mesh.points[polygon_itr->vertices[1]].z , 2 )
                );
       y = sqrt( pow(cloud_mesh.points[polygon_itr->vertices[1]].x - cloud_mesh.points[polygon_itr->vertices[2]].x , 2 ) +
                pow(cloud_mesh.points[polygon_itr->vertices[1]].y - cloud_mesh.points[polygon_itr->vertices[2]].y , 2 ) +
                pow(cloud_mesh.points[polygon_itr->vertices[1]].z - cloud_mesh.points[polygon_itr->vertices[2]].z , 2 )
                );
        z = sqrt( pow(cloud_mesh.points[polygon_itr->vertices[2]].x - cloud_mesh.points[polygon_itr->vertices[0]].x , 2 ) +
                pow(cloud_mesh.points[polygon_itr->vertices[2]].y - cloud_mesh.points[polygon_itr->vertices[0]].y , 2 ) +
                pow(cloud_mesh.points[polygon_itr->vertices[2]].z - cloud_mesh.points[polygon_itr->vertices[0]].z , 2 )
                );
        polygon_perimeter = x+y+z;
        //优化 可以使用基于leafsize的自适应大小
        polygon_perimeter_radius[ (int)(polygon_perimeter*10000) ] ++;
    }

    std::vector<int> nice_polygon;
    //选取的范围
    for(int i=0;i<max_polygon;i++)
    {
        std::vector<int>::iterator poly_itr = std::max_element(polygon_perimeter_radius.begin(),polygon_perimeter_radius.end());
      //  nice_polygon[i]=std::distance(polygon_perimeter_radius.begin(), poly_itr);
        nice_polygon.push_back( std::distance(polygon_perimeter_radius.begin(), poly_itr) );
        polygon_perimeter_radius.erase(poly_itr);
    }

    std::vector<int> polygon_point;
    for(polygon_itr = mesh.polygons.begin(); polygon_itr != mesh.polygons.end(); polygon_itr++)
    {
        float polygon_perimeter;
        float x,y,z;
        x = sqrt( pow(cloud_mesh.points[polygon_itr->vertices[0]].x - cloud_mesh.points[polygon_itr->vertices[1]].x , 2 ) +
                pow(cloud_mesh.points[polygon_itr->vertices[0]].y - cloud_mesh.points[polygon_itr->vertices[1]].y , 2 ) +
                pow(cloud_mesh.points[polygon_itr->vertices[0]].z - cloud_mesh.points[polygon_itr->vertices[1]].z , 2 )
                );
       y = sqrt( pow(cloud_mesh.points[polygon_itr->vertices[1]].x - cloud_mesh.points[polygon_itr->vertices[2]].x , 2 ) +
                pow(cloud_mesh.points[polygon_itr->vertices[1]].y - cloud_mesh.points[polygon_itr->vertices[2]].y , 2 ) +
                pow(cloud_mesh.points[polygon_itr->vertices[1]].z - cloud_mesh.points[polygon_itr->vertices[2]].z , 2 )
                );
        z = sqrt( pow(cloud_mesh.points[polygon_itr->vertices[2]].x - cloud_mesh.points[polygon_itr->vertices[0]].x , 2 ) +
                pow(cloud_mesh.points[polygon_itr->vertices[2]].y - cloud_mesh.points[polygon_itr->vertices[0]].y , 2 ) +
                pow(cloud_mesh.points[polygon_itr->vertices[2]].z - cloud_mesh.points[polygon_itr->vertices[0]].z , 2 )
                );
        polygon_perimeter = x+y+z;
        //选取的点，记录
        if( find(nice_polygon.begin(),nice_polygon.end(), (int)(polygon_perimeter*10000) ) != nice_polygon.end() )
        {
            polygon_point.push_back(polygon_itr->vertices[0]);
            polygon_point.push_back(polygon_itr->vertices[1]);
            polygon_point.push_back(polygon_itr->vertices[2]);
        }
    }

    //所有点进行排序
    std::sort(polygon_point.begin(),polygon_point.end());

    //全局阈值
    float global_threshold=0;
    float full_theshold=0;
    int full_point_num=0;

    int d=(int)(cloud_mesh.size() / cloud_mls->size()) ;//公式

    pcl::KdTreeFLANN<PointT> polygon_tree_k;
    polygon_tree_k.setInputCloud(cloud_mls);
    int k = 3;//选取三角点最大距离
    std::vector<int> point_idx(k);
    std::vector<float> point_distance(k);
    for(int i=0;i<polygon_point.size();)
    {
        //选取最远点
        if(polygon_tree_k.nearestKSearch(cloud_mesh.points[ polygon_point[i] ],
                                     k,point_idx,point_distance)>0)
        {
            full_theshold +=point_distance[2];
            full_point_num++;
        }
        i += d ;
    }
    //最终全局阈值
    global_threshold = full_theshold/full_point_num;

    //标记不符合的点
    pcl::KdTreeFLANN<PointT> polygon_tree_radius;
    polygon_tree_radius.setInputCloud(cloud_mls);
    //int hash_cloud[cloud_mesh.size()] = {0};
    std::vector< int > hash_cloud(cloud_mesh.size(),0);
    for(int i = 0 ; i< cloud_mesh.size();i++)
    {
        //范围内没找到
        if(polygon_tree_k.nearestKSearch(cloud_mesh.points[ i ],
                                     1,point_idx,point_distance)>0)
        {
            if(point_distance[0]>=global_threshold)
                hash_cloud[i]=1;
        }
    }

    //去除冗余三角面片
    polygon_itr = mesh.polygons.begin();
    for(polygon_itr;polygon_itr != mesh.polygons.end();)
    {
        int flag = 0;
        for(int i=0;i<polygon_itr->vertices.size();i++)
        {
            if( (hash_cloud[polygon_itr->vertices[i] ] ) == 1)
            {
                mesh.polygons.erase(polygon_itr);
                flag=1;
                break;
            }
        }
        if(flag == 0)
            polygon_itr++;
    }

    //去除相关点
 /*   std::vector< int > indexs;
    for(int i = 0; i < hash_cloud.size(); i++)
    {
        if(hash_cloud[i] == 1)
        {
            indexs.push_back(i);
        }
    }
    pcl::PointCloud<PointT> cloud_remove_polygon;
    pcl::copyPointCloud(cloud_mesh,indexs,cloud_remove_polygon);
*/
    pcl::toPCLPointCloud2(cloud_mesh,mesh.cloud);
}

void gp::reconsruction::meshColoring(pcl::PolygonMesh& mesh, pcl::PointCloud< PointT >::Ptr cloud)
{
    pcl::PointCloud<PointT> cloud_color_mesh;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud_color_mesh);

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (cloud);
    // K 搜索的近邻点数
    int K = 5;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    for(int i=0;i<cloud_color_mesh.points.size();++i)
    {
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        int red = 0;
        int green = 0;
        int blue = 0;

        if ( kdtree.nearestKSearch (cloud_color_mesh.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (int j = 0; j < pointIdxNKNSearch.size (); ++j)
            {
                r = cloud->points[ pointIdxNKNSearch[j] ].r;
                g = cloud->points[ pointIdxNKNSearch[j] ].g;
                b = cloud->points[ pointIdxNKNSearch[j] ].b;

                red += int(r);
                green += int(g);
                blue += int(b);
            }
        }
        cloud_color_mesh.points[i].r = int(red/pointIdxNKNSearch.size ()+0.5);
        cloud_color_mesh.points[i].g = int(green/pointIdxNKNSearch.size ()+0.5);
        cloud_color_mesh.points[i].b = int(blue/pointIdxNKNSearch.size ()+0.5);
    }
    toPCLPointCloud2(cloud_color_mesh, mesh.cloud);
}

void gp::reconsruction::get_config()
{
    std::ifstream fin("project_config.json");
    rapidjson::IStreamWrapper isw(fin);
    
    rapidjson::Document document;
    document.ParseStream(isw);
    
    rapidjson::Value &rec = document["reconsruction"]; 
    leaf_size = rec["leaf_size"].GetFloat();

    mls_radius = rec["mls_radius"].GetFloat();
    
    depth= rec["depth"].GetInt();
    solver_divide = rec["solver_divide"].GetInt();
    iso_divide = rec["iso_divide"].GetInt();

    max_polygon = rec["max_polygon"].GetInt();
}

/*
float gp::operator *(pcl::PointXYZRGBNormal &p1,pcl::PointXYZRGBNormal &p2)
{
    float cost;
    cost = p1.normal_x * p2.normal_y +
            p1.normal_y * p2.normal_y +
            p1.normal_z * p2.normal_z;
    return cost;
}
*/
