#include "controlpanel.h"
#include "ui_controlpanel.h"


ControlPanel::ControlPanel(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ControlPanel)
{
    ui->setupUi(this);
}

ControlPanel::~ControlPanel(void)
{
    delete ui;
}

void ControlPanel::setController(FlightController *newController){
    controller = newController;
}

void ControlPanel::on_pushButton_Start_clicked(void)
{
    controller->startProgram();
}

void ControlPanel::on_pushButton_Reset_clicked(void)
{
    controller->resetProgram();
}

void ControlPanel::on_pushButton_Stop_clicked(void)
{
    controller->abortProgram();
}

void ControlPanel::on_pushButton_shutdown_clicked(void)
{
    const char* command = "kill $(ps aux | grep ros | grep -v grep | awk '{print $2}')";
    controller->resetProgram();
    system(command);
    exit (EXIT_SUCCESS);
}

