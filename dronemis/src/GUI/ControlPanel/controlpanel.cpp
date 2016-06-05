#include "controlpanel.h"
#include "ui_controlpanel.h"



ControlPanel::ControlPanel(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ControlPanel)
{
    ui->setupUi(this);
}

ControlPanel::~ControlPanel()
{
    delete ui;
}

void ControlPanel::on_pushButton_Start_clicked()
{
    blindFlight::startProgram();
}

void ControlPanel::on_pushButton_Reset_clicked()
{
    blindFlight::resetProgram();
}

void ControlPanel::on_pushButton_Stop_clicked()
{
    blindFlight::abortProgram();
}

void ControlPanel::on_pushButton_shutdown_clicked()
{
    const char* command = "kill $(ps aux | grep ros | grep -v grep | awk '{print $2}')";
    blindFlight::abortProgram();
    system(command);
    exit (EXIT_SUCCESS);
}

