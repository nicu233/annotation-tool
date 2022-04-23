#include "anno.h"
#include <QTextBrowser>
#include <fstream>
#include <QDockWidget>
#include <QHBoxLayout >
#include <QPushButton>
#include <QStandardItemModel>
#include <QComboBox>
#include <QCheckBox>
#include <QFileDialog>
#include <QMessageBox >
#include <QTextCodec>
#include <QSettings>
//#define withoutCameraTest
#include <windows.h>


int pickingDemo::num=0;
//unsigned char* pickingDemo::pointmark;//点云分类标记
std::vector< int > pickingDemo::pointmark;
std::vector< int > pickingDemo::idx_mapping; //view pc idx mapping all pc idx
std::vector< int > pickingDemo::cur_class_idx;
pcl::PointCloud<pcl::PointXYZ> pickingDemo::clicked_points_3d;
pcl::PointCloud<pcl::PointXYZ> pickingDemo::cloud;
pcl::PointCloud<pcl::PointXYZ> pickingDemo::cloud_all;
pcl::visualization::PCLVisualizer pickingDemo::viewer;
std::vector< int > pickingDemo::indices;
QVTKWidget *pickingDemo::qvtkWidget;



pickingDemo::pickingDemo(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

	initPointCloudVisualizationWidget();
	// init cloud
	//clicked_points_3d = new pcl::PointCloud<pcl::PointXYZ>();
	/*if (pcl::io::loadPLYFile("E:\\project\\vs\\cloud_bin_0.ply", cloud))
	{
		std::cerr << "ERROR: Cannot open file " << std::endl;
		return;
	}*/

	//this->grabKeyboard();

	read_default_class();

	cloud_all = cloud;

	pointmark.resize(cloud.points.size(), 0);
	// 默认类别为杂物
	points_class.resize(cloud.points.size(), 91);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = cloud_all.makeShared();
	viewer.registerAreaPickingCallback(pp_callback, (void*)&cloudPtr);//注册事件


	addPointCloudDisplayXYZ(cloud);



	//message dockwidget initialization
	infoDock = new QDockWidget(tr("message"), this);
	infoDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

	QWidget *infoWidget = new QWidget();

	QGridLayout *msgGridLayout = new QGridLayout(infoWidget);
	msgGridLayout->setSpacing(6);
	msgGridLayout->setContentsMargins(11, 11, 11, 11);
	
	
	msgBrowser = new QTextBrowser(infoWidget);
	msgGridLayout->addWidget(msgBrowser);


	infoDock->setWidget(infoWidget);
	addDockWidget(Qt::RightDockWidgetArea, infoDock);

	//information dockwidget 
	ui.matchtableView->setEditTriggers(QAbstractItemView::NoEditTriggers);
	connect(ui.matchtableView, &QTableView::clicked, this, [this](const QModelIndex & index) {
		//msgBrowser->append("matchtableView clicked");
		QItemSelectionModel *select = ui.matchtableView->selectionModel();

		////restore original color
		//for (auto i = 0; i < resultpcPtr.size(); ++i) {
		//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> selectedPCColor(resultpcPtr[i], pcVisualizationModel.r, pcVisualizationModel.g, pcVisualizationModel.b);
		//	viewer.updatePointCloud(resultpcPtr[i], selectedPCColor, "matched" + std::to_string(i));
		//}

		if (select->hasSelection()) {

			cur_class_id = index.row();
			msgBrowser->append("cur_class_id " + QString::number(cur_class_id));

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> selectedPCColor(cloud.makeShared(), pcVisualizationSelectedModel.r, pcVisualizationSelectedModel.g, pcVisualizationSelectedModel.b);
			viewer.updatePointCloud(cloud.makeShared(), selectedPCColor, "matched" + std::to_string(cur_class_id));
			
			qvtkWidget->update();

		}
		else {
			//msgBrowser->append("not select->hasSelection()");
		}
		});
	
	createMenuActions();

}


pickingDemo::~pickingDemo() {

}


void pickingDemo::initPointCloudVisualizationWidget() {


	//viewer initialization
	viewer.setWindowName("show");
	viewer.setBackgroundColor(pcVisualizationBackground.r, pcVisualizationBackground.g, pcVisualizationBackground.b);
	viewer.addCoordinateSystem(100.0, "reference", 0);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> pcColor(cur_cloud.makeShared(), 0, 255, 0);
	//viewer.addPointCloud(cur_cloud.makeShared(), pcColor, "cloud");

	//Eigen::Affine3f tm_inv_aff(pcp::loadModelScene::_pcp_camHmodel[0].inverse());
	//viewer.addCoordinateSystem(100.0, tm_inv_aff, "obj", 0);

	qvtkWidget = new QVTKWidget();
	qvtkWidget->SetRenderWindow(viewer.getRenderWindow());

	//用vtk点云显示铺满空间，不适用dockwidget
	setCentralWidget(qvtkWidget);
}

void pickingDemo::createMenuActions()
{

	QMenu *FileMenu = menuBar()->addMenu(tr("&File"));

	QAction *import_pc_action = new QAction(tr("import pc"), this);
	import_pc_action->setStatusTip(tr("select input folder of pc files "));
	connect(import_pc_action, &QAction::triggered, this, &pickingDemo::On_import_pc_files);
	FileMenu->addAction(import_pc_action);

	QAction *export_pc_action = new QAction(tr("export pc"), this);
	export_pc_action->setStatusTip(tr("select ouput folder of pc files "));
	connect(export_pc_action, &QAction::triggered, this, &pickingDemo::On_export_pc_files);
	FileMenu->addAction(export_pc_action);

	QAction *import_label_class_action= new QAction(tr("import label"), this);
	import_label_class_action->setStatusTip(tr("select label file "));
	connect(import_label_class_action, &QAction::triggered, this, &pickingDemo::On_import_label_class_file);
	FileMenu->addAction(import_label_class_action);

	//QAction *preprocess_action = new QAction(tr("preprocess"), this);
	//preprocess_action->setStatusTip(tr("preprocess"));
	//connect(preprocess_action, &QAction::triggered, this, &pickingDemo::On_preprocess);
	//FileMenu->addAction(preprocess_action);

	//QAction *postprocess_action = new QAction(tr("postprocess"), this);
	//postprocess_action->setStatusTip(tr("postprocess"));
	//connect(postprocess_action, &QAction::triggered, this, &pickingDemo::On_postprocess);
	//FileMenu->addAction(postprocess_action);

	QMenu *ToolMenu = menuBar()->addMenu(tr("&Tools"));

	QAction *preprocess_action = new QAction(tr("preprocess"), this);
	preprocess_action->setStatusTip(tr("preprocess"));
	connect(preprocess_action, &QAction::triggered, this, &pickingDemo::On_preprocess);
	ToolMenu->addAction(preprocess_action);

	QAction *postprocess_action = new QAction(tr("postprocess"), this);
	postprocess_action->setStatusTip(tr("postprocess"));
	connect(postprocess_action, &QAction::triggered, this, &pickingDemo::On_postprocess);
	ToolMenu->addAction(postprocess_action);

	QAction *retrace_action = new QAction(tr("retrace"), this);
	retrace_action->setStatusTip(tr("retrace"));
	retrace_action->setShortcut(tr("ctrl+z"));
	connect(retrace_action, &QAction::triggered, this, &pickingDemo::On_retrace);
	ToolMenu->addAction(retrace_action);


	//view
	QMenu *viewMenu = menuBar()->addMenu(tr("&View"));
	viewMenu->addAction(ui.informationDock->toggleViewAction());
	viewMenu->addAction(infoDock->toggleViewAction());

	//AppTranslator = new QTranslator;
	//AppTranslator->load(":/qmFile/pickingdemo_zh.qm");

	QSettings *reg = new QSettings("HKEY_CURRENT_USER\\SOFTWARE\\VisenPick", QSettings::NativeFormat);
	//reg->setValue("VisenTopPath", fileName);

	QAction *chineseVisualAction = new QAction(tr("chinese"), this);
	chineseVisualAction->setStatusTip(tr("set application show chinese"));
	connect(chineseVisualAction, &QAction::triggered, this, [reg]() {
		//qApp->removeTranslator(AppTranslator);
		//qApp->installTranslator(AppTranslator); 
		QSettings *appConfigFile = new QSettings("./config.ini", QSettings::IniFormat);
		appConfigFile->beginGroup("form");
		appConfigFile->setValue("language", "chinese");
		appConfigFile->endGroup();

		reg->setValue("Language", "chinese");

		QMessageBox msgBox;
		msgBox.setWindowTitle("message");
		msgBox.setText("Please reset this application.");
		msgBox.setInformativeText("This application visualization will be changed to chinese.");
		msgBox.setStandardButtons(QMessageBox::Ok);
		msgBox.exec();
		
		});
	viewMenu->addAction(chineseVisualAction);

	QAction *englishVisualAction = new QAction(tr("english"), this);
	englishVisualAction->setStatusTip(tr("change application's langurage to english"));
	connect(englishVisualAction, &QAction::triggered, this, [this, reg]() {
		QSettings *appConfigFile = new QSettings("./config.ini", QSettings::IniFormat);
		appConfigFile->beginGroup("form");
		appConfigFile->setValue("language", "english");
		appConfigFile->endGroup();

		reg->setValue("Language", "english");

		QMessageBox msgBox;
		msgBox.setWindowTitle("message");
		msgBox.setText("Please reset this application.");
		msgBox.setInformativeText("This application visualization will be changed to english.");
		msgBox.setStandardButtons(QMessageBox::Ok);
		msgBox.exec();
		});
	viewMenu->addAction(englishVisualAction);


	ui.mainToolBar->setIconSize(QSize(40,40));

	
	

	//const QIcon segmentIcon = QIcon::fromTheme("bin_picking", QIcon(QString::fromStdString(defaultSegmentIconFileName)));
	const QIcon segmentIcon = QIcon::fromTheme("bin_picking", QIcon(QString::fromStdString(":/ManuIcon/segment1.png")));
	QAction *segmentPC = new QAction(segmentIcon, tr("segment"), this);
	segmentPC->setStatusTip(tr("segment point cloud"));
	segmentPC->setShortcut(Qt::Key_Space);
	connect(segmentPC, &QAction::triggered, this, &pickingDemo::OnSegmentButtonClick);
	//HandEyeRelMenu->addAction(capturePC);
	ui.mainToolBar->addAction(segmentPC);

	

	
}

void pickingDemo::addPointCloudDisplay(const pcl::PointCloud<pcl::PointNormal> cloud) {
	viewer.removePointCloud("cloud");
	viewer.removePointCloud("matched");
	viewer.removeAllCoordinateSystems();
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> pcColor(cloud.makeShared(), 0, 125, 0);
	viewer.addPointCloud(cloud.makeShared(), pcColor, "cloud");
	qvtkWidget->update();
}

void pickingDemo::addPointCloudDisplayXYZ(const pcl::PointCloud<pcl::PointXYZ> cloud) {
	//viewer.removePointCloud("cloud");
	//viewer.removePointCloud("matched");
	viewer.removeAllPointClouds();
	viewer.removeAllCoordinateSystems();

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pcColor(cloud.makeShared(), 0, 125, 0);
	viewer.addPointCloud(cloud.makeShared(), pcColor, "cloud");
	qvtkWidget->update();
}

void pickingDemo::OnSegmentButtonClick() {


	idx_mapping.clear();
	for (int i = 0; i < pointmark.size(); ++i)
	{
		//msgBrowser->append("idx_mapping size is " + QString::fromStdString(std::to_string(pointmark[i])));

		if (pointmark[i] == 0)
		{
			//msgBrowser->append("idx_mapping size is " + QString::fromStdString(std::to_string(i)));
			idx_mapping.push_back(i);
		}
	}


	//update points_class
	for (int i = 0; i < cur_class_idx.size(); ++i) {
		points_class[cur_class_idx[i]] = cur_class_id;
	}
	cur_class_idx.clear();

	//update cloud
	cloud.clear();
	for (int i = 0; i < idx_mapping.size(); ++i) {
		cloud.push_back(cloud_all.points[idx_mapping[i]]);
	}

	msgBrowser->append("cloud size is " + QString::fromStdString(std::to_string(cloud.points.size())));

	addPointCloudDisplayXYZ(cloud);

	viewer.removeText3D("number");
	viewer.addText(std::to_string(cloud.size()), 20, 60, 16, 0,0,125,"number");
	qvtkWidget->update();

	//// quit X mode
	//QWidget *receiver = QApplication::focusWidget();
	//QKeyEvent keyPress(QEvent::KeyPress, Qt::Key_X, Qt::NoModifier);
	//QKeyEvent keyRelease(QEvent::KeyRelease, Qt::Key_X, Qt::NoModifier);
	//QApplication::sendEvent(receiver, &keyPress);
	//QApplication::sendEvent(receiver, &keyRelease);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = cloud_all.makeShared();
	//viewer.registerAreaPickingCallback(pp_callback, (void*)&cloudPtr);//注册事件

	

}

//void pickingDemo::importMatrixFromFile(const std::string fileName, cv::Mat &EyeToHandCalibMatrix, int row, int col) {
//	ifstream detectorFile(fileName);
//
//	cv::Mat tempEyeToHandCalibMatrix(row, col, CV_64FC1);
//
//	CV_Assert(detectorFile.good());
//
//	{
//		char buffer[256];
//		int rows = 0;
//
//		while (!detectorFile.eof())
//		{	
//			detectorFile.getline(buffer, 100);
//			QString str(buffer);
//			str.simplified().remove(' ');
//			//qDebug() << str << endl;
//
//			QStringList strlist = str.split(",");
//
//			//qDebug() << "the cols is " << strlist.count() << endl;
//			CV_Assert(strlist.count() == col+1 );
//			CV_Assert(rows <= row);
//
//			for (int cols = 0; cols < col; ++cols) {
//				tempEyeToHandCalibMatrix.at<double>(rows, cols) = strlist.at(cols).toDouble();
//			}
//			++rows;
//
//			if (rows+1 > row)break;
//		}
//		EyeToHandCalibMatrix = tempEyeToHandCalibMatrix;
//	}
//}

//void pickingDemo::exportFileFromMatrix(const std::string fileName, const cv::Mat &Matrix, int row, int col) {
//	ofstream out;
//	out.open(fileName);
//	out.clear();
//
//	CV_Assert(Matrix.rows == row && Matrix.cols == col);
//
//	for (auto rows = 0; rows < Matrix.rows; ++rows) {
//		for (auto cols = 0; cols < Matrix.cols; ++cols) {
//			out << std::to_string(Matrix.at<double>(rows, cols)) << ",";
//		}
//
//		out << endl;
//	}
//}


void pickingDemo::freshMatchTableView(const std::vector<int > &class_id, const std::vector<std::string > &class_name) {
	QStandardItemModel *standardModel = new QStandardItemModel();

	QStringList list;
	list << "ID"  << "Name";

	standardModel->setHorizontalHeaderLabels(list);
	
	if (class_id.size() == class_name.size()) {
		for (int row = 0; row < class_id.size(); ++row) {
			QStandardItem *newItemID = new QStandardItem(QString::number(class_id[row]));
			standardModel->setItem(row, 0, newItemID);

			//QStandardItem *newItemSize = new QStandardItem(QString::number(size[row]));
			//standardModel->setItem(row, 1, newItemSize);

			QStandardItem *newItemName = new QStandardItem(QString::fromStdString(class_name[row]));
			standardModel->setItem(row, 1, newItemName);
		}
		ui.matchtableView->setModel(standardModel);
	}
	else {
		//传入的数据错误
	}

}

QString pickingDemo::fileOpen() {
	QString fileName = QFileDialog::getOpenFileName(this);
	return fileName;
}

void pickingDemo::On_retrace() {

	//恢复pointmark 未框选
	for (auto i = 0; i < cur_class_idx.size(); i++) {
		pointmark[cur_class_idx[i]] = 0;
	}
	cur_class_idx.clear();

	msgBrowser->append("retrace cloud size is " + QString::fromStdString(std::to_string(cloud.points.size())));
	if (cloud.points.size() > 0) {
		addPointCloudDisplayXYZ(cloud);
	}
	

	viewer.removeText3D("number");
	viewer.addText(std::to_string(cloud.size()), 20, 60, 16, 0, 0, 125, "number");
	qvtkWidget->update();

}

void pickingDemo::pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{

	qDebug() << "there pp call";

	//如果没有剩下的点云了就不框选了
	if (cloud.size() == 0) {
		qDebug() << "cloud.size() == 0";
		//如果在选中状态下分割再选中空白处就会闪退，但是如果每次选中后关闭选中状态，再选中空白处就不会报错。所以每次框选后自动关闭框选状态
		keybd_event(88, 0, 0, 0);
		keybd_event(88, 0, KEYEVENTF_KEYUP, 0);
		return;
	}

	if (event.getPointsIndices(indices) == -1) {
		qDebug() << "event.getPointsIndices(indices) == -1";
		//如果在选中状态下分割再选中空白处就会闪退，但是如果每次选中后关闭选中状态，再选中空白处就不会报错。所以每次框选后自动关闭框选状态
		keybd_event(88, 0, 0, 0);
		keybd_event(88, 0, KEYEVENTF_KEYUP, 0);
		return;
	}
	else {
		qDebug() << "indices.size() :"<<indices.size();
	}

	//如果上一次框选的点没有分割，不可再框选。
	if (cur_class_idx.size() != 0) {
		qDebug() << "wait for seg";
		//如果在选中状态下分割再选中空白处就会闪退，但是如果每次选中后关闭选中状态，再选中空白处就不会报错。所以每次框选后自动关闭框选状态
		keybd_event(88, 0, 0, 0);
		keybd_event(88, 0, KEYEVENTF_KEYUP, 0);
		return;
	}
		

	//std::vector< int > indices;
	clicked_points_3d.clear();
	//cur_class_idx.clear();
	
	//添加点云（去掉重复的点）
	int addcount = 0;//添加点计数
	long bt = clock();//开始时间

	qDebug() << "idx_mapping.size(): " << idx_mapping.size();
	
	for (int i = 0; i < indices.size(); ++i)
	{
		if (idx_mapping.size() == 0) {
			//qDebug() << "pointmark[indices[i]]" << pointmark[indices[i]];
			if (pointmark[indices[i]] == 0)
			{

				//pcl::PointXYZ p = cloud.points.at(indices[i]);
				clicked_points_3d.points.push_back(cloud.points.at(indices[i]));
				addcount++;
				pointmark[indices[i]] = 1;//将点云标记为已圈选状态
				cur_class_idx.push_back(indices[i]);
			}
			
			

		}
		else {
			//qDebug() << "pointmark[idx_mapping[indices[i]]]" << pointmark[idx_mapping[indices[i]]];
			if (pointmark[idx_mapping[indices[i]]] == 0)
			{

				//pcl::PointXYZ p = cloud.points.at(indices[i]);
				clicked_points_3d.points.push_back(cloud.points.at(indices[i]));
				addcount++;
				pointmark[idx_mapping[indices[i]]] = 1;//将点云标记为已圈选状态
				cur_class_idx.push_back(idx_mapping[indices[i]]);
			}
			
			
		}


		
	}
	long et = clock();//开始时间
	//std::cout << "花费时间:" << et - bt << "ms" << endl;
	//std::cout << "添加点数:" << addcount << endl;

	clicked_points_3d.height = 1;
	clicked_points_3d.width += addcount;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d.makeShared(), 255, 0, 0);
	//每添加一次点云都要取一次别名，不然只能选择一次
	std::stringstream ss;
	std::string cloudName;
	ss << num++;
	ss >> cloudName;
	cloudName += "_cloudName";
	//显示点云
	viewer.addPointCloud(clicked_points_3d.makeShared(), red, cloudName);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloudName);
	// 保存点云
	//pcl::PCDWriter writer;
	//writer.write<pcl::PointXYZ>("plane.pcd", clicked_points_3d);
	qvtkWidget->update();
	indices.clear();

	//如果在选中状态下分割再选中空白处就会闪退，但是如果每次选中后关闭选中状态，再选中空白处就不会报错。所以每次框选后自动关闭框选状态
	keybd_event(88, 0, 0, 0);
	keybd_event(88, 0, KEYEVENTF_KEYUP, 0);
}

void pickingDemo::On_import_pc_files() {

	//init 
	idx_mapping.clear();

	msgBrowser->append("idx_mapping size is "+ QString::number(idx_mapping.size()));


	QString folder_name = QFileDialog::getExistingDirectory(this);
	msgBrowser->append("current foler is" + folder_name);
	if (!folder_name.isEmpty()) {
		/*pcp::loadModelScene::loadModel(pcp::loadModelScene::ply, fileName.toLatin1().toStdString());
		cur_cloud = pcp::loadModelScene::getModel().back();*/

		auto GetSpecifiedFormatFiles = [this](
			const QString & dstDir,
			//const QString & targetName,
			QFileInfoList & list,
			QString suffix = "csv")
		{
			// 获取目录文件列表
			QDir dir(dstDir);
			dir.setFilter(QDir::Files | QDir::Hidden | QDir::NoSymLinks);
			dir.setSorting(QDir::Name | QDir::Reversed);

			QStringList filters;
			filters << QString("*.%1").arg(suffix);
			dir.setNameFilters(filters);

			list = dir.entryInfoList();

		};

		
		GetSpecifiedFormatFiles(folder_name, list);


		auto ReadCSVtoPC = [this](const QString & fileName, pcl::PointCloud<pcl::PointXYZ> & save_pc, /*int & cnt,*/ std::vector< int > & part_idx_mapping) {
			/*save_pc.clear();*/

			QFile file(fileName);
			if (file.open(QIODevice::ReadOnly | QIODevice::Text))
			{
				while (!file.atEnd())
				{
					QByteArray line = file.readLine();
					QString str(line);
					// the least line maybe has nothing
					if (str.size() == 0) { break; }
					

					QStringList list = str.split(" ");
					std::string a, b, c;
					a = list.at(0).toStdString();
					b = list.at(1).toStdString();
					c = list.at(2).toStdString();
					double x, y, z;
					x = (float)atof(a.c_str());
					y = (float)atof(b.c_str());
					z = (float)atof(c.c_str());
					pcl::PointXYZ p;
					p.x = x;
					p.y = y;
					p.z = z;
					//msgBrowser->append("x:"+list.at(0)+ " y:"+ list.at(1) + " z:"+ list.at(2));
					save_pc.push_back(p);
					part_idx_mapping.push_back(save_pc.size()-1);
					//msgBrowser->append("id:" + QString::number(save_pc.size() - 1));
					//cnt++;
				}
				file.close();

			}

		};
		
		//int point_cnt = 0;
		cloud.clear();
		for (int i = 0; i < list.size(); ++i) {
			QString fileName = list.at(i).fileName();
			std::vector< int > part_idx_mapping;
			fileName = folder_name + '/' + fileName;
			msgBrowser->append(fileName);
			ReadCSVtoPC(fileName, cloud,/* point_cnt,*/ part_idx_mapping);
			parts_idx_mapping.push_back(part_idx_mapping);
		}
		pointmark.clear();
		pointmark.resize(cloud.points.size(), 0);
		// 默认类别为杂物

		points_class.clear();
		points_class.resize(cloud.points.size(), 91);

		cloud_all.clear();
		cloud_all = cloud;
		addPointCloudDisplayXYZ(cloud);

		////重新注册
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = cloud_all.makeShared();
		//viewer.registerAreaPickingCallback(pp_callback, (void*)&cloudPtr);//注册事件

		viewer.removeText3D("number");
		viewer.addText(std::to_string(cloud.size()), 20, 60, 16, 0,0,125,"number");
		qvtkWidget->update();

		if (cloud.points.size() == 0) {
			QMessageBox msgBox;
			msgBox.setWindowTitle("ERROR");
			msgBox.setText("unvaild model file");
			msgBox.setInformativeText("Please check path is right.");
			msgBox.setStandardButtons(QMessageBox::Ok);
			msgBox.exec();
		}
		//msgBrowser->append("cur_cloud points is" + QString::fromStdString(std::to_string(cur_cloud.points.size())));
		/*viewer.removeAllPointClouds();
		addPointCloudDisplayXYZ(cloud);*/
		//curModelFilePath = fileName.toLatin1().toStdString();

		/*if (!curProjectFilePath.empty()) {
			OnSaveCurProjectClick();
		}*/
	}
	else {
		QMessageBox msgBox;
		msgBox.setWindowTitle("ERROR");
		msgBox.setText("fail to open the path");
		msgBox.setInformativeText("Please check path is right.");
		msgBox.setStandardButtons(QMessageBox::Ok);
		msgBox.exec();
	}
}

void pickingDemo::On_export_pc_files() {
	QString folder_name = QFileDialog::getExistingDirectory(this);

	auto write_pc_file= [](const QString & file_name, const pcl::PointCloud<pcl::PointXYZ> & save_pc, std::vector< int > &points_class, const std::vector< int > &part_idx_mapping)
	{

		QFile file(file_name);
		if (file.open(QIODevice::ReadWrite | QIODevice::Text))
		{
			QTextStream stream(&file);
			stream.seek(file.size());
			for(auto i = 0; i < part_idx_mapping.size(); i++)
			{
				QString qs;
				qs.append(QString::number(save_pc.points[part_idx_mapping[i]].x));
				qs.append(" ");
				qs.append(QString::number(save_pc.points[part_idx_mapping[i]].y));
				qs.append(" ");
				qs.append(QString::number(save_pc.points[part_idx_mapping[i]].z));
				qs.append(" ");
				qs.append(QString::number(points_class[part_idx_mapping[i]]));
				qs.remove("\n");
				stream << qs << "\n";
			}
			file.close();
		}
	};

	for (auto i = 0; i < list.size(); ++i) {
		QString file_name = folder_name + "/" + list.at(i).fileName();
		msgBrowser->append("ouput file name: "+ file_name);
		std::vector< int > part_idx_mapping = parts_idx_mapping[i];
		write_pc_file(file_name, cloud_all, points_class, part_idx_mapping);
	}
	
	QDir dir_all(folder_name);
	dir_all.cdUp();
	QString file_name = dir_all.absolutePath() +"/all_label.csv";

	msgBrowser->append("all csv in " + file_name);

	QFile file(file_name);
	if (file.open(QIODevice::ReadWrite | QIODevice::Text))
	{
		QTextStream stream(&file);
		stream.seek(file.size());
		for (auto i = 0; i < cloud_all.size(); i++)
		{
			QString qs;
			qs.append(QString::number(cloud_all.points[i].x));
			qs.append(" ");
			qs.append(QString::number(cloud_all.points[i].y));
			qs.append(" ");
			qs.append(QString::number(cloud_all.points[i].z));
			qs.append(" ");
			qs.append(QString::number(points_class[i]));
			qs.remove("\n");
			stream << qs << "\n";
		}
		file.close();
	}

}

void pickingDemo::On_import_label_class_file() {

	QString fileName = QFileDialog::getOpenFileName(this);

	std::vector<int > class_ids;
	std::vector<std::string > class_names;

	QFile file(fileName);
	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		while (!file.atEnd())
		{
			QByteArray line = file.readLine();
			QString str(line);
			// the least line maybe has nothing
			if (str.size() == 0) { break; }


			str.remove("\n");
			QStringList list = str.split(",");
			int class_id = list.at(0).toInt();

			QTextCodec *code = QTextCodec::codecForName("utf-8");//解决中文路径问题
			std::string class_name = code->fromUnicode(list.at(1)).data();
			//std::string class_name = list.at(1).toStdString();
			class_ids.push_back(class_id);
			class_names.push_back(class_name);
		}
	}

	freshMatchTableView(class_ids, class_names);

	QFile::copy(fileName, ".\default_class.txt");
}


void pickingDemo::read_default_class() {

	QString fileName = ".\default_class.txt";


	if (!QFile::exists(fileName)) {
		return ;
	}

	std::vector<int > class_ids;
	std::vector<std::string > class_names;

	QFile file(fileName);
	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		while (!file.atEnd())
		{
			QByteArray line = file.readLine();
			QString str(line);
			// the least line maybe has nothing
			if (str.size() == 0) { break; }


			str.remove("\n");
			QStringList list = str.split(",");
			int class_id = list.at(0).toInt();

			QTextCodec *code = QTextCodec::codecForName("utf-8");//解决中文路径问题
			std::string class_name = code->fromUnicode(list.at(1)).data();
			//std::string class_name = list.at(1).toStdString();
			class_ids.push_back(class_id);
			class_names.push_back(class_name);
		}
	}

	freshMatchTableView(class_ids, class_names);
}

//void pickingDemo::keyPressEvent(QKeyEvent *event) {
//	if (event->key() == Qt::Key_X)
//	{
//		status = (status == 0) ? 1 : 0;
//		std::string s = (status == 0) ? "move" : "select";
//		msgBrowser->append(QString::fromStdString(s));
//		viewer.removeText3D("status");
//		viewer.addText(s, 20, 90, 16, 0, 0, 125, "status");
//		qvtkWidget->update();
//	}
//}

void pickingDemo::On_preprocess() {
	QString folder_name = QFileDialog::getExistingDirectory(this);
	char command[128];
	sprintf(command, ".\\rerange.exe --mode %s --f %s", "pre", folder_name.toStdString().c_str());
	msgBrowser->append("command :");
	msgBrowser->append(command);
	std::system(command);
	msgBrowser->append("command finish");
}

void pickingDemo::On_postprocess() {
	QString folder_name = QFileDialog::getExistingDirectory(this);
	QString pose_folder_name= folder_name+"/poses";
	QString pc_folder_name = folder_name+"/label";

	//msgBrowser->append(pose_folder_name);
	//msgBrowser->append(pc_folder_name);

	if (!QFile::exists(pose_folder_name)) {
		msgBrowser->append("cont find "+pose_folder_name);
		return;
	}

	if (!QFile::exists(pc_folder_name)) {
		msgBrowser->append("cont find " + pc_folder_name);
		return;
	}
	
	QString all = ".\\rerange.exe --mode inverse --pose_path " + pose_folder_name + " --pc_path " + pc_folder_name;

	char command[256];
	sprintf(command, all.toStdString().c_str());
	//command = all.toStdString().c_str();

	//sprintf(command, ".\\rerange.exe --mode %s --pose_path %s --pc_path %s", "inverse", pose_folder_name.toStdString().c_str(), pc_folder_name.toStdString().c_str());
	msgBrowser->append("command :");
	msgBrowser->append(command);
	std::system(command);

	msgBrowser->append("command finish");
}