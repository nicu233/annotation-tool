#pragma once

#include <QtWidgets/QMainWindow>
#include <QWidget>
#include <QTextBrowser>
#include <QFile>
#include <QtNetwork>
#include <QKeyEvent>
#include <QWidget>
#include <QTextStream>
#include <QScopedPointer>
#include <qvtkwidget.h>
#include "ui_pickingDemo.h"

#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>

#include <vector>
#include <iostream>
#include <string>


class pickingDemo : public QMainWindow
{
    Q_OBJECT
	typedef struct rgbInfo {
		double r = 1.0;
		double g = 1.0;
		double b = 1.0;
	} rgb;


public:
    pickingDemo(QWidget *parent = Q_NULLPTR);
	~pickingDemo();

	////ICON PATH
	//const std::string defaultEyetohandCalibIconFileName = "./img/eyetohandCalib.png";
	//const std::string defaultInitializationCameraIconFileName = "./img/initialCamera.png";
	//const std::string defaultCaptureIconFileName = "./img/capture.png";
	//const std::string defaultSegmentIconFileName = "./img/segment.png";
	//const std::string defaultMatchIconFileName = "./img/match.png";

	void On_import_pc_files();
	void On_export_pc_files();
	void On_import_label_class_file();

	void On_retrace();

	void On_preprocess();
	void On_postprocess();

	void read_default_class();
	//void keyPressEvent(QKeyEvent *event);


	static int num;
	static std::vector< int > pointmark;//点云分类标记
	static std::vector< int > idx_mapping;
	static std::vector< int > cur_class_idx;
	static pcl::PointCloud<pcl::PointXYZ> clicked_points_3d;
	static pcl::PointCloud<pcl::PointXYZ> cloud_all;
	static pcl::PointCloud<pcl::PointXYZ> cloud;
	static void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args);
	static std::vector< int > indices;

	static pcl::visualization::PCLVisualizer viewer;


	int cur_class_id;

	int status = 0; // 0 is move state, 1 is select state
	std::vector< int > points_class;
	std::vector < std::vector< int > > parts_idx_mapping;// pc files idx mapping single all_pc
	QFileInfoList list;

	

private:
    Ui::pickingDemoClass ui;
	QTranslator * AppTranslator;

	void initPointCloudVisualizationWidget();

	void addPointCloudDisplay(const pcl::PointCloud<pcl::PointNormal> cloud);
	void addPointCloudDisplayXYZ(const pcl::PointCloud<pcl::PointXYZ> cloud);
	void createMenuActions();

	//void importMatrixFromFile(const std::string fileName, cv::Mat &Matrix, int row = 4, int col = 4);
	//void exportFileFromMatrix(const std::string fileName, const cv::Mat &Matrix, int row = 4, int col = 4);

	void freshMatchTableView(const std::vector<int > &class_id, const std::vector<std::string > &class_name);

	QString fileOpen();
	//bool cameraStatus = false;
	//pcl::console::TicToc pclTime;
	//QTimer	*robotMoveLoopTimer;
	//RobotMoveLoop   *robotMoveLoop;
	//RobotServer *robotServer;
	
	static QVTKWidget *qvtkWidget;

	QDockWidget *infoDock;
	QTextBrowser *msgBrowser;
	
	//RobPose curHomePose;

	//the result of matching pc
	//std::vector<cv::Mat > resultMat;
	//std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr > resultpcPtr;

	//pcp::pointCloudSegment::segmentMethod selectedSegmentMethod = pcp::pointCloudSegment::euclideanClustering;
	//pcp::pointCloudMatch::matchMethod selectedMatchMethod = pcp::pointCloudMatch::ppf;
	//pcp::pointCloudMatch::refineMethod selectedRefineMethod = pcp::pointCloudMatch::no_refine;
	
	//
	rgb pcVisualizationBackground = { .85, .85, .85 };
	rgb pcVisualizationModel = { 175.0,0,0 };
	rgb pcVisualizationSelectedModel = { 255.0,175.,0 };

	//these parameter will be save at project file.
	//parameter file path
	std::string curProjectFilePath;
	std::string curModelFilePath;
	std::string curHashTableFilePath;
	std::string cureyetohandCalibFilePath;
	std::string curGrabCalibFilePath;
	std::string curVisenTOPFilePath;


signals:
	//void pickAndPlacePoseSignal(RobPose, RobPose);
	//void placeTendingSignal(RobPose, int, int, double, double, double);
	//void pickPoseSignal(RobPose);
	void OnResetPlaceTending();
	void chineseVisualization();
	void englishVisualization();
	//void setHomePoseSignal(RobPose);

private slots:
	
	//cloud button
	void OnSegmentButtonClick();
	
};
