#include <QtCore/QString>
#include <QtCore>
#include <QApplication>
#include <QtGui/qevent.h>
#include <QtGui/QtGui>
#include "controlpanel.h"
#include "ui_controlpanel.h"
#include <QLCDNumber>
#include <QtGui/QStyleOption>

ControlPanel::ControlPanel(QWidget *parent) : QMainWindow(parent), ui(new Ui::ControlPanel) {
    ui->setupUi(this);
    paletteOrg.setColor(paletteOrg.WindowText, Qt::black);
    paletteRed.setColor(paletteRed.WindowText, Qt::red);
    ui->lcdNumber_Right->display("00");
    ui->lcdNumber_Left->display("00:00");
    ui->lcdNumber_Position->display("000:000:000");
    timer = new QTimer(this);
    timerPos = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updatePanel()));
    connect(timerPos, SIGNAL(timeout()), this, SLOT(updatePosition()));
    timer->start(1000);
    timerPos->start(50);
}

ControlPanel::~ControlPanel(void){
    delete(ui);
    delete(timer);
    delete(timerPos);
}


void ControlPanel::setValues(Nav *nav, FlightController *newController, ros::NodeHandle *n, int countdownSeconds){
    node = n;
    navData = nav;
    secondsLeft = secondsleftConst = countdownSeconds;
    controller = newController;
    sub_navdata = n->subscribe<ardrone_autonomy::Navdata>("ardrone/navdata", 10, &ControlPanel::callback, this);
}

void ControlPanel::callback(const ardrone_autonomy::NavdataConstPtr &msg) {
    battery =  msg->batteryPercent;
}

void ControlPanel::on_pushButton_Start_clicked(void)
{
    controller->startProgram();
    started = true;
}

void ControlPanel::on_pushButton_Reset_clicked(void)
{
    started = false;
    secondsLeft = secondsleftConst;
    controller->resetProgram();
}

void ControlPanel::on_pushButton_Stop_clicked(void)
{
    started = false;
    secondsLeft = secondsleftConst;
    controller->abortProgram();
}

void ControlPanel::updatePanel(void) {

    if (started && --secondsLeft > -600) {
        string timeStr;

        int mins = secondsLeft / 60;
        int secs = secondsLeft % 60;

        if (mins != 0) {
            timeStr = to_string(mins);
            timeStr.append(":");
        }

        if (secondsLeft < 0 && mins != 0)
            secs = -secs;

        timeStr.append(to_string(secs));

        if (secondsLeft < 30)
            ui->lcdNumber_Left->setPalette(paletteRed);
        else
            ui->lcdNumber_Left->setPalette(paletteOrg);

        ui->lcdNumber_Left->display(QString::fromStdString(timeStr));
    }
    if (battery < 30.00)
        ui->lcdNumber_Right->setPalette(paletteRed);
    else
        ui->lcdNumber_Right->setPalette(paletteOrg);

    ui->lcdNumber_Right->display(battery);
}

void ControlPanel::updatePosition(void) {
    string posStr;
    posStr.append(to_string((int)navData->position.x));
    posStr.append(":");
    posStr.append(to_string((int)navData->position.y));
    posStr.append(":");
    posStr.append(to_string((int) (navData->position.z/10)));
    posStr.append(":");
    posStr.append(to_string((int)navData->getRotation()));
    ui->lcdNumber_Position->display(QString::fromStdString(posStr));
}

void ControlPanel::on_pushButton_shutdown_clicked(void)
{
    const char* command = "kill $(ps aux | grep ros | grep -v grep | awk '{print $2}')";
    controller->resetProgram();
    system(command);
    exit (EXIT_SUCCESS);
}

