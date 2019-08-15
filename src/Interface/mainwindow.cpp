#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    /*Reference Qt Demo : imageviewer.cpp*/

    imageLabel = new QLabel;
    imageLabel->setBackgroundRole(QPalette::Base);
    //imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    //imageLabel->setScaledContents(true);

    scrollArea = ui->scrollArea;
    scrollArea->setBackgroundRole(QPalette::Dark);
    scrollArea->setWidget(imageLabel);

    /*Loads Image a la Qt*/
//    image = QImage("/home/robst/Dropbox/AirplaneMCL/IKONOS_venice.jpg");



    /*cv::Mat to QImage references:
     *http://stackoverflow.com/questions/5026965/how-to-convert-an-opencv-cvmat-to-qimage
     *
    */

    /*Loads Image with openCV*/
    currentView=cv::imread("/home/robst/Dropbox/AirplaneMCL/IKONOS_venice.jpg",CV_LOAD_IMAGE_COLOR);
    /*Transforms cv::Mat to QImage */
    image = QImage((uchar*) currentView.data, currentView.cols, currentView.rows, currentView.step, QImage::Format_RGB888);
    image = image.rgbSwapped();/*This is required because openCV stores in BGR format and Qt in RGB*/

    imageLabel->setPixmap(QPixmap::fromImage(image));

    this->showMaximized();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_btnExit_clicked()
{
    this->close();
}

/*Somewhere in the application we will need to do repainting
 *
 *
 *  mLabel->setPixmap(QPixmap::fromImage(mImage)); // mImage is a QImage containing the new frame
    mLabel->update();
    //or
    mLabel->repaint();
    QApplication::processEvents();


Updates the widget unless updates are disabled or the widget is hidden.

This function does not cause an immediate repaint; instead it schedules a paint event for processing
when Qt returns to the main event loop. This permits Qt to optimize for more speed and less flicker
than a call to repaint() does.


  http://qt-project.org/doc/qt-4.8/qwidget.html#update-2
  http://qt-project.org/doc/qt-4.8/qwidget.html#repaint

*/
