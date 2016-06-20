#ifndef CONTROLPANEL_H
#define CONTROLPANEL_H

#include <QMainWindow>
#include "../../flightControl/FlightController.h"
#include <QPalette>
#include <QTimer>

namespace Ui {
class ControlPanel;
}

class ControlPanel : public QMainWindow
{
    Q_OBJECT

public:
    explicit ControlPanel(QWidget *parent = 0);
    void setValues(Nav *nav, FlightController *newController, ros::NodeHandle *n, int countdownSeconds);
    ~ControlPanel(void);

private slots:
    void on_pushButton_Start_clicked(void);
    void on_pushButton_Reset_clicked(void);
    void on_pushButton_Stop_clicked(void);
    void on_pushButton_shutdown_clicked(void);
    void on_pushButton_Test1_clicked();
    void on_pushButton_Test2_clicked();
    void on_pushButton_Test3_clicked();
    void on_pushButton_Test4_clicked();
    void on_pushButton_Test5_clicked();
    void on_pushButton_Test6_clicked();
    void updatePanel(void);
    void updatePosition(void);

private:
    bool started;
    ros::NodeHandle *node;
    void callback(const ardrone_autonomy::NavdataConstPtr &msg);
    float battery;
    ros::Subscriber sub_navdata;
    int secondsLeft;
    int secondsleftConst;
    Ui::ControlPanel *ui;
    Nav *navData;
    FlightController *controller;
    QPalette paletteRed;
    QPalette paletteOrg;
    QTimer * timer;
    QTimer * timerPos;
};

#endif // CONTROLPANEL_H
