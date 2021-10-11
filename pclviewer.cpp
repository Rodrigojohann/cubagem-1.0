#include "pclviewer.h"
#include "ui_pclviewer.h"

using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer)
////
{
    ui->setupUi (this);
    this->setWindowTitle ("Cubagem");

    connect (ui->pushButton_2, SIGNAL(clicked()), this, SLOT(ConnectDevice()));
    connect (ui->pushButton, SIGNAL(clicked()), this, SLOT(Clean()));
    connect (this, SIGNAL(value(bool)), this, SLOT(MainFrame()));

    // Set up the QVTK window
    viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    viewer_->setBackgroundColor(0, 0, 0);
    viewer_->setShowFPS(false);
    ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void PCLViewer::MainFrame(){
  ////////
    Controller c;
    Sensor s;

    for (size_t counter = 0; counter < 10; ++counter)
    {
        cloudnew.reset(new pcl::PointCloud<pcl::PointXYZ>);
        coloredinput.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

        cloudnew = s.CamStream(IP, PORT);
        coloredinput->points.resize(cloudnew->points.size());

        if (cloudnew->points.size() > 100)
        {

            for (size_t i = 0; i < coloredinput->points.size(); i++)
            {
                coloredinput->points[i].x = (*cloudnew)[i].x;
                coloredinput->points[i].y = (*cloudnew)[i].y;
                coloredinput->points[i].z = (*cloudnew)[i].z;
                coloredinput->points[i].r = 255;
                coloredinput->points[i].g = 255;
                coloredinput->points[i].b = 255;
                coloredinput->points[i].a = 200;
            }

            filteredcloud = c.FilterCloud(cloudnew);
            std::tie(unsortedclusters, clustersize) = c.CloudSegmentation(filteredcloud);

            notorientedclusters = c.SortClusters(unsortedclusters, clustersize);
            clusters = c.RemoveInclined(filteredcloud, notorientedclusters);

            viewer_->updatePointCloud(coloredinput, "inputcloud");

            if (clusters.size() > 5)
            {
                limitcluster = 5;
            }
            else
            {
                limitcluster = clusters.size();
            }

            totalvolume = 0;
            objvolume = 0;

            coloredcloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

            viewer_->updatePointCloud (coloredcloud, to_string(0));
            viewer_->updatePointCloud (coloredcloud, to_string(1));
            viewer_->updatePointCloud (coloredcloud, to_string(2));
            viewer_->updatePointCloud (coloredcloud, to_string(3));
            viewer_->updatePointCloud (coloredcloud, to_string(4));

            for (int number=0; number<limitcluster; ++number)
            {
                segmented_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
                coloredcloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
                coloredcloud->points.resize(clusters[number].indices.size());
                segmented_cloud->points.resize(clusters[number].indices.size());

                for(size_t i=0; i<clusters[number].indices.size(); ++i)
                {
                    segmented_cloud->points[i].x = (*filteredcloud)[clusters[number].indices[i]].x;
                    segmented_cloud->points[i].y = (*filteredcloud)[clusters[number].indices[i]].y;
                    segmented_cloud->points[i].z = (*filteredcloud)[clusters[number].indices[i]].z;

                    coloredcloud->points[i].x = (*filteredcloud)[clusters[number].indices[i]].x;
                    coloredcloud->points[i].y = (*filteredcloud)[clusters[number].indices[i]].y;
                    coloredcloud->points[i].z = (*filteredcloud)[clusters[number].indices[i]].z;

                    coloredcloud->points[i].r = cloudcolor[number][0];
                    coloredcloud->points[i].g = cloudcolor[number][1];
                    coloredcloud->points[i].b = cloudcolor[number][2];
                    coloredcloud->points[i].a = 255;
                }

                hullarea = c.SurfaceArea(segmented_cloud);
                std::tie(dimensionX, dimensionY, dimensionZ) = c.CalculateDimensions(segmented_cloud);

                objvolume = hullarea*dimensionZ/100;
                totalvolume += objvolume;

                VolStr = to_string(objvolume).substr(0,5)+" m³";

                if (number == 0)
                {
                    ui->label_1->setAutoFillBackground(true);
                    QPalette palette = ui->label_1->palette();
                    palette.setColor(QPalette::WindowText, QColor(cloudcolor[number][0],cloudcolor[number][1],cloudcolor[number][2]));
                    ui->label_1->setPalette(palette);
                    mean1 += objvolume;
                }
                else if (number == 1)
                {
                    ui->label_2->setAutoFillBackground(true);
                    QPalette palette = ui->label_2->palette();
                    palette.setColor(QPalette::WindowText, QColor(cloudcolor[number][0],cloudcolor[number][1],cloudcolor[number][2]));
                    ui->label_2->setPalette(palette);
                    mean2 += objvolume;
                }
                else if (number == 2)
                {
                    ui->label_3->setAutoFillBackground(true);
                    QPalette palette = ui->label_3->palette();
                    palette.setColor(QPalette::WindowText, QColor(cloudcolor[number][0],cloudcolor[number][1],cloudcolor[number][2]));
                    ui->label_3->setPalette(palette);
                    mean3 += objvolume;
                }
                else if (number == 3)
                {
                    ui->label_4->setAutoFillBackground(true);
                    QPalette palette = ui->label_4->palette();
                    palette.setColor(QPalette::WindowText, QColor(cloudcolor[number][0],cloudcolor[number][1],cloudcolor[number][2]));
                    ui->label_4->setPalette(palette);
                    mean4 += objvolume;
                }
                else if (number == 4)
                {
                    ui->label_5->setAutoFillBackground(true);
                    QPalette palette = ui->label_5->palette();
                    palette.setColor(QPalette::WindowText, QColor(cloudcolor[number][0],cloudcolor[number][1],cloudcolor[number][2]));
                    ui->label_5->setPalette(palette);
                    mean5 += objvolume;
                }

                viewer_->updatePointCloud (coloredcloud, to_string(number));
            }
        }
        volumemean += totalvolume;
    }

    Vol1 = to_string(mean1/10).substr(0,5)+" m³";
    Vol2 = to_string(mean2/10).substr(0,5)+" m³";
    Vol3 = to_string(mean3/10).substr(0,5)+" m³";
    Vol4 = to_string(mean4/10).substr(0,5)+" m³";
    Vol5 = to_string(mean5/10).substr(0,5)+" m³";
    TotalStr = to_string(volumemean/10).substr(0,5)+" m³";

    ui->label_1->setText(QString::fromStdString(Vol1));
    ui->label_2->setText(QString::fromStdString(Vol2));
    ui->label_3->setText(QString::fromStdString(Vol3));
    ui->label_4->setText(QString::fromStdString(Vol4));
    ui->label_5->setText(QString::fromStdString(Vol5));

    ui->label_6->setAutoFillBackground(true);
    ui->label_6->setText("Total: "+QString::fromStdString(TotalStr));
    ui->label_6->setStyleSheet("font-weight: bold");

    ui->qvtkWidget->update();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::Clean()
{
    viewer_->removeAllPointClouds();

    coloredinput.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    coloredcloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

    viewer_->resetCamera();
    viewer_->setCameraPosition(0, 0, -3, 0, -1.3, -1);

    viewer_->addPointCloud(coloredinput, "inputcloud");
    viewer_->addPointCloud(coloredcloud, to_string(0));
    viewer_->addPointCloud(coloredcloud, to_string(1));
    viewer_->addPointCloud(coloredcloud, to_string(2));
    viewer_->addPointCloud(coloredcloud, to_string(3));
    viewer_->addPointCloud(coloredcloud, to_string(4));

    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "inputcloud");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, to_string(0));
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, to_string(1));
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, to_string(2));
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, to_string(3));
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, to_string(4));

    ui->label_1->setAutoFillBackground(true);
    QPalette palette = ui->label_1->palette();
    palette.setColor(QPalette::WindowText, QColor(0, 0, 0));
    ui->label_1->setPalette(palette);
    ui->label_1->setText(QString::fromStdString("0.000 m³"));

    ui->label_2->setAutoFillBackground(true);
    ui->label_2->setPalette(palette);
    ui->label_2->setText(QString::fromStdString("0.000 m³"));

    ui->label_3->setAutoFillBackground(true);
    ui->label_3->setPalette(palette);
    ui->label_3->setText(QString::fromStdString("0.000 m³"));

    ui->label_4->setAutoFillBackground(true);
    ui->label_4->setPalette(palette);
    ui->label_4->setText(QString::fromStdString("0.000 m³"));

    ui->label_5->setAutoFillBackground(true);
    ui->label_5->setPalette(palette);
    ui->label_5->setText(QString::fromStdString("0.000 m³"));

    volumemean = 0.0;
    mean1 = 0.0;
    mean2 = 0.0;
    mean3 = 0.0;
    mean4 = 0.0;
    mean5 = 0.0;

    ui->qvtkWidget->update();

    emit value(true);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::ConnectDevice()
{
    cout << "test";


}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PCLViewer::~PCLViewer()
{
  delete ui;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
