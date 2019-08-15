#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtGui>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
private slots:
    void on_btnExit_clicked();

private:
    /*The main window of the application*/
    Ui::MainWindow *ui;

    QLabel *imageLabel;         /*The label used to display the main image*/
    QScrollArea *scrollArea;    /*The scrollArea used to display the image*/
    int nParticles;             //The quantity of particles for the filter
    QImage image;               //The image used to handle the cv::Mat image
    cv::Mat currentView;        //The current representation of the map and filter
};

#endif // MAINWINDOW_H
