//
// Created by nmohsin on 16/01/19.
//
#include "functions_kinect.h"
//#include <vector.hpp>



float func_kinect::distancePCL(pcl::PointXYZ a, pcl::PointXYZ b) {
    return sqrt((a.x-b.x)*(a.x-b.x)
                +(a.y-b.y)*(a.y-b.y)
                +(a.z-b.z)*(a.z-b.z));
}
float func_kinect::distance2D(func_kinect::Point2D a, func_kinect::Point2D b){
    return sqrt((a.x-b.x)*(a.x-b.x)
                +(a.y-b.y)*(a.y-b.y));
}
float func_kinect::nearest_point(cv::Mat A){
    /*std::vector<cv::Point> nz; // rather use Point, than Vec2i, see below
    cv::findNonZero(A,nz);
    //cerr << cv::Mat(nz);

    cv::Mat nonzeros;
    for (cv::Point p : nz) {
        nonzeros.push_back(A.at<uchar>(p)); // collect values
    }*/
    double min, max;
    cv::Mat mask = A>0;
    cv::minMaxLoc(A, &min, &max,NULL,NULL,mask);
    return static_cast<float>(min);

}

void func_kinect::foreground_segment(cv::Mat A){
    cv::Mat tempA=A/1.0f;
    float nearpt = nearest_point(tempA);
    cv::absdiff(tempA, nearpt, tempA);
    tempA.setTo(std::numeric_limits<int>::quiet_NaN(),tempA==nearpt);
    tempA.setTo(std::numeric_limits<int>::quiet_NaN(),tempA>FILT_DIST);
    A=tempA+nearpt;
    A.setTo(0,tempA==std::numeric_limits<int>::quiet_NaN());
    //cv::Mat mask= tempA>FILT_DIST;
    //cv::bitwise_and(A,A,A,mask/255);
    //A.setTo(0,tempA>1200.0f);

}


int func_kinect::convertToXYZPointCloud (cv::Mat depthData, const int& width,const int& height,const float& fx,
        const float& fy, const float& cx, const float& cy,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
    cloud->height = height;
    cloud->width = width;
    cloud->is_dense = false;

    cloud->points.resize (cloud->height * cloud->width);

    register float constant_x = 1.0f/fx;
    register float constant_y = 1.0f/fy ;
    register float centerX = cx;
    register float centerY = cy;
    register int validPoint =0;
    float bad_point = std::numeric_limits<float>::quiet_NaN ();

    register int depth_idx = 0;
    for (int v = 0; v < cloud->height; ++v)
    {
        for (register int u = 0; u < cloud->width; ++u, ++depth_idx)
        {
            pcl::PointXYZ& pt = cloud->points[depth_idx];
            // Check for invalid measurements
            if (depthData.at<float>(v,u) == 0)
            {
                // not valid
                pt.x = pt.y = pt.z = bad_point;
                continue;
            }
            else pt.z = depthData.at<float>(v,u)*0.001f;
            pt.x = (static_cast<float> (u) - centerX) * pt.z * constant_x;
            pt.y = (static_cast<float> (v) - centerY) * pt.z * constant_y;
            ++validPoint;
        }
    }
    /*cloud->sensor_origin_.setZero ();
    cloud->sensor_orientation_.w () = 0.0f;
    cloud->sensor_orientation_.x () = 0.0f;
    cloud->sensor_orientation_.y () = 0.0f;
    cloud->sensor_orientation_.z () = 0.0f;*/

        return validPoint;
}

cv::Mat func_kinect::convertXYZPointCloud2DepthImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    cv::Mat image = cv::Mat( cloud->height, cloud->width, CV_32FC1 );

    // pcl::PointCloud to cv::Mat
    #pragma omp parallel for
    for( int y = 0; y < image.rows; y++ ) {
        for( int x = 0; x < image.cols; x++ ) {
            pcl::PointXYZ point = cloud->at( x, y );

            image.at<float>( y, x ) = point.z;
        }
    }
    return image;

}


pcl::PointCloud<pcl::PointXYZ>::Ptr
    func_kinect::PtCloudPassFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cld (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setKeepOrganized (true);
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.5, 2.7);
//pass.setFilterLimitsNegative (true);
    pass.filter(*new_cld);
    return new_cld;


    /*pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZ>);
    // First test, the point's Z value must be greater than (GT) 0.
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.5)));
    // Second test, the point's Z value must be less than (LT) 2.
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 2.7)));

    condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -0.4)));

    condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 0.6)));
    // Checks available: GT, GE, LT, LE, EQ.

    // Filter object.
    pcl::ConditionalRemoval<pcl::PointXYZ> filter;
    filter.setCondition(condition);
    filter.setInputCloud(cloud);
    // If true, points that do not pass the filter will be set to a certain value (default NaN).
    // If false, they will be just removed, but that could break the structure of the cloud
    // (organized clouds are clouds taken from camera-like sensors that return a matrix-like image).
    filter.setKeepOrganized(true);
    // If keep organized was set true, points that failed the test will have their Z value set to this.
    filter.setUserFilterValue(0.0);


    filter.filter(*new_cld);
    return new_cld;*/
}

pcl::PolygonMesh func_kinect::SimpleMeshGen(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    pcl::OrganizedFastMesh<pcl::PointXYZ> fst_msh;
    pcl::PolygonMesh triangles;
    fst_msh.setMaxEdgeLength(THRES_DIST);
    fst_msh.setTrianglePixelSize(1);
    fst_msh.useDepthAsDistance(true);
    fst_msh.setAngleTolerance(5);
    fst_msh.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_ADAPTIVE_CUT);
    fst_msh.setInputCloud(cloud);
//fst_msh.setSearchMethod(tree);
    fst_msh.reconstruct(triangles);

    return triangles;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr func_kinect::FillNaN(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    // Do we want to copy everything?
    if (cloud_in->is_dense==true)
    {
        *cloud_out = *cloud_in;
        return cloud_out;
    }

    // Allocate enough space and copy the basics
    cloud_out->points.resize (cloud_in->points.size ());
    cloud_out->header   = cloud_in->header;
    cloud_out->width    = cloud_in->width;
    cloud_out->height   = cloud_in->height;
    cloud_out->is_dense = cloud_in->is_dense;
    cloud_out->sensor_orientation_ = cloud_in->sensor_orientation_;
    cloud_out->sensor_origin_ = cloud_in->sensor_origin_;
    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    // Iterate over each point
    for (size_t i = 0; i < cloud_in->points.size (); ++i) {
        pcl::PointXYZ &pt = cloud_in->points[i];
        pcl::PointXYZ &pt1 = cloud_out->points[i];
        if (pt.z == 0.0f) {
            pt1.x = pt1.y = pt1.z = bad_point;
            continue;
        }

        cloud_out->points[i] = cloud_in->points[i];
    }
    return cloud_out;
}

void func_kinect::BoundaryEdges(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
        std::vector<pcl::PointIndices> &label_indices,
        pcl::PointCloud<pcl::PointXYZ> &occluding_edges,
        pcl::PointCloud<pcl::PointXYZ> &occluded_edges,
        pcl::PointCloud<pcl::PointXYZ> &boundary_edges){

    pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label> oed;
    oed.setInputCloud(cloud_in);
    oed.setDepthDisconThreshold(0.02); // 2cm
    oed.setMaxSearchNeighbors(50);
    pcl::PointCloud<pcl::Label> labels;
    //std::vector<pcl::PointIndices> label_indices;
    oed.compute(labels, label_indices);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr high_curvature_edges(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*cloud_in, label_indices[0].indices, boundary_edges);
    pcl::copyPointCloud(*cloud_in, label_indices[1].indices, occluding_edges);
    pcl::copyPointCloud(*cloud_in, label_indices[2].indices, occluded_edges);
    return ;

}

void func_kinect::InterEdgeConnectivity(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
        const pcl::PointCloud<pcl::PointXYZ>:: Ptr boundary_edges,
        const std::vector<int> &boundary_indices,
        const int &update_index,
        std::vector<cv::Vec2i> &inter_connectivity,
        std::vector<float> &inter_weights){

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    int K = 2;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.setInputCloud (cloud_in);

    for (size_t i=0; i<boundary_indices.size(); ++i) {
        pcl::PointXYZ& searchPoint=boundary_edges->points[i];

        // K nearest neighbor search

        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            for (size_t j = 1; j < pointIdxNKNSearch.size(); ++j) {
                /*std::cout << "    " << cloud_in->points[pointIdxNKNSearch[j]].x
                          << " " << cloud_in->points[pointIdxNKNSearch[j]].y
                          << " " << cloud_in->points[pointIdxNKNSearch[j]].z
                          << " (squared distance: " << pointNKNSquaredDistance[j] << ")" << std::endl;*/

                if (pointNKNSquaredDistance[j]<=THRES_FUSION){

                    inter_connectivity.push_back(cv::Vec2i(boundary_indices[i],pointIdxNKNSearch[j]+update_index));
                    inter_weights.push_back(pointNKNSquaredDistance[j]);

                }
            }

        }
    }
    return ;
}

void func_kinect::IntraEdgeConnectivity(const pcl::PolygonMesh &mesh, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                                    std::vector<cv::Vec2i> &intra_connectivity,
                                    std::vector<float> &intra_weights){
    float dt_3D;
    for(int t = 0; t < mesh.polygons.size(); t++) {
        pcl::Vertices triangle = mesh.polygons[t];
        assert(triangle.vertices.size() == 3);
        int a = triangle.vertices[0], b = triangle.vertices[1], c = triangle.vertices[2];
        if(!(cloud_in->points[a].z)||!(cloud_in->points[b].z)||!(cloud_in->points[c].z)) continue;
// add each edge only once
        if(a < b)
        {
            dt_3D=distancePCL(cloud_in->points[a],cloud_in->points[b]);
            intra_connectivity.push_back(cv::Vec2i(a,b));
            intra_weights.push_back(dt_3D);
//std::cout<<"Edge Sucess: "<<tt.second<<std::endl;
        }

        if(b < c)
        {
            dt_3D=distancePCL(cloud_in->points[b],cloud_in->points[c]);
            intra_connectivity.push_back(cv::Vec2i(b,c));
            intra_weights.push_back(dt_3D);

        }
        if(c < a)
        {
            dt_3D=distancePCL(cloud_in->points[c],cloud_in->points[a]);
            intra_connectivity.push_back(cv::Vec2i(c,a));
            intra_weights.push_back(dt_3D);
        }
    }
    return ;
}

void func_kinect::GraphBuilding(std::vector<cv::Vec2i> &intra_connectivity,
        std::vector<float> &intra_weights,
        std::vector<cv::Vec2i> &inter_connectivity,
        std::vector<float> &inter_weights,
        Graph &graph)
{
    std::pair<Edge, bool> tt;
    for (size_t i=0; i<intra_weights.size(); ++i){
        tt=boost::add_edge(static_cast<unsigned long>(intra_connectivity[i][0]),static_cast<unsigned long>(intra_connectivity[i][1]), Weight(intra_weights[i]), graph);
        //std::cout<<"Edge Sucess: "<<tt.second<<std::endl;
    }

    for (size_t i=0; i<inter_weights.size(); ++i){
        tt=boost::add_edge(static_cast<unsigned long>(inter_connectivity[i][0]),static_cast<unsigned long>(inter_connectivity[i][1]), Weight(inter_weights[i]), graph);
        //std::cout<<"Edge Sucess: "<<tt.second<<std::endl;
    }

}

void func_kinect::FindNearestPt2CentroidPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, Eigen::Vector4f &centroid, unsigned long &cent_idx){


    pcl::PointIndices indices1; float x=0.0f,y=0.0f,z=0.0f;
    register int validNum=0;
    for(size_t i=0;i<cloud_in->points.size();++i){
        pcl::PointXYZ& P = cloud_in->points[i];
        if (!(isnan(double(P.z))) & P.z!=0.0f )
        {
            x+= P.x;y+=P.y;z+=P.z;
            indices1.indices.push_back(i);
            ++validNum;
        }

    }


    centroid[0] = x / validNum;
    centroid[1] = y / validNum;
    centroid[2] = z/ validNum;


    //pcl::compute3DCentroid 	(*cloud_in,indices1.indices,centroid);
    pcl::PointXYZ searchPoint;

    searchPoint.x=centroid[0];searchPoint.y=centroid[1];searchPoint.z=centroid[2];

  /*  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    int K = 2;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.setInputCloud (cloud_in);
        // K nearest neighbor search
        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {

            cent_idx = static_cast<unsigned long>(pointIdxNKNSearch[0]);

        }*/

    cent_idx=0;float cd_3D=distancePCL(searchPoint,cloud_in->points[cent_idx]),temp;
    for(size_t i=1;i<cloud_in->points.size();++i){
        if (std::isnan(cloud_in->points[i].z))continue;
        temp=distancePCL(searchPoint,cloud_in->points[i]);
        if (temp<cd_3D){
            cd_3D=temp;
            cent_idx=i;
        }

    }

}

void func_kinect::ADDGeoIntensity(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, const std::vector<float> geodist,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out){


    pcl::copyPointCloud (*cloud_in, *cloud_out);

    for (size_t i=0;i<cloud_out->points.size();i++)
    {
        cloud_out->points[i].intensity=geodist[i];
    }
}