#include <QtCore/QString>
#include <QtCore>
#include <QApplication>
#include <QtGui/qevent.h>
#include "controlpanel.h"
#include "ui_controlpanel.h"

ControlPanel::ControlPanel(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ControlPanel)
{
    ui->setupUi(this);
    paletteOrg.setColor(paletteOrg.WindowText, Qt::black);
    paletteRed.setColor(paletteRed.WindowText, Qt::red);
    ui->lcdNumber_Right->setPalette(paletteOrg);
    ui->lcdNumber_Left->setPalette(paletteOrg);
    ui->lcdNumber_Right->display("00");
    ui->lcdNumber_Left->display("00:00");
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updatePanel()));
    timer->start(1000);
}

ControlPanel::~ControlPanel(void)
{
    delete(ui);
    delete(timer);
}


void ControlPanel::setValues(FlightController *newController, ros::NodeHandle *n, int countdownSeconds){
    node = n;
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

void ControlPanel::on_pushButton_Test_clicked(void)
{
    controller->testProgram();
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

void ControlPanel::on_pushButton_shutdown_clicked(void)
{
    const char* command = "kill $(ps aux | grep ros | grep -v grep | awk '{print $2}')";
    controller->resetProgram();
    system(command);
    exit (EXIT_SUCCESS);
}

