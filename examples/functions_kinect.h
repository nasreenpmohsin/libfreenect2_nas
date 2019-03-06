//
// Created by nmohsin on 16/01/19.
//
#ifndef FUNCTIONS_KINECT_H
#define FUNCTIONS_KINECT_H

#ifndef LIBFREENECT2_FUNCTIONS_KINECT_H
#define LIBFREENECT2_FUNCTIONS_KINECT_H

#endif //LIBFREENECT2_FUNCTIONS_KINECT_H
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <signal.h>
#include <cstdio>
//#include <sys/time.h>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/config.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
//#include <opencv2/core/eigen.hpp>
#include <math.h>
#include <Eigen/Dense>
//PCL
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
//#include <chrono>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/organized_fast_mesh.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/features/organized_edge_detection.h>

//#include < opencv2/core/core.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>


#include <GL/glut.h>

#define FILT_DIST 1000.0f // maximum distantce  from nearest point on body
// max distance between any vertices on body surface
#define THRES_DIST 0.075f
#define THRES_DIST1  0.2
// max disatance between vertices in inter connectitivty edge
#define THRES_FUSION 0.2f
// No. of extrema points
#define K_EX 5

#define CMPNT_NODE_THRES 50 // minimum number of point in each connected set in subgraph of mapping function

typedef boost::property<boost::edge_weight_t, float> Weight;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, Weight> Graph;
typedef boost::graph_traits < Graph >::vertex_descriptor vertex_descriptor;
typedef Graph::edge_descriptor Edge;

class func_kinect {
private:
    struct Point2D {
        double x;
        double y;
    };


public:


    float distancePCL(pcl::PointXYZ a, pcl::PointXYZ b);

    float distance2D(Point2D a, Point2D b);

    void foreground_segment(cv::Mat A);

    float nearest_point(cv::Mat A);

    int convertToXYZPointCloud (cv::Mat depthData, const int& width,const int& height,const float& fx,
            const float& fy, const float& cx, const float& cy,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    cv::Mat convertXYZPointCloud2DepthImage(pcl::PointCloud<pcl::PointXYZ>::Ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    PtCloudPassFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    pcl::PolygonMesh SimpleMeshGen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr FillNaN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);

    void BoundaryEdges(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                                    std::vector<pcl::PointIndices> &label_indices,
                                    pcl::PointCloud<pcl::PointXYZ> &occluding_edges,
                                    pcl::PointCloud<pcl::PointXYZ> &occluded_edges,
                                    pcl::PointCloud<pcl::PointXYZ> &boundary_edges);

    void InterEdgeConnectivity(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                                            const pcl::PointCloud<pcl::PointXYZ>:: Ptr boundary_edges,
                                            const std::vector<int> &boundary_indices,
                                            const int &update_index,
                                            std::vector<cv::Vec2i> &inter_connectivity,
                                            std::vector<float> &inter_weights);

    void IntraEdgeConnectivity(const pcl::PolygonMesh &mesh, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                                        std::vector<cv::Vec2i> &intra_connectivity,
                                        std::vector<float> &intra_weights);

    void GraphBuilding(std::vector<cv::Vec2i> &intra_connectivity,
                                    std::vector<float> &intra_weights,
                                    std::vector<cv::Vec2i> &inter_connectivity,
                                    std::vector<float> &inter_weights,
                                    Graph &graph);

    void FindNearestPt2CentroidPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, Eigen::Vector4f &centroid, unsigned long &cent_idx);

    void ADDGeoIntensity(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, const std::vector<float> geodist,
                                      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out);

};



#endif // FUNCTIONS_KINECT_H
