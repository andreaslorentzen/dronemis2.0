#ifndef CONTROLPANEL_H
#define CONTROLPANEL_H

#include <QMainWindow>
#include "../../flightControl/blindFlight.h"
namespace Ui {
class ControlPanel;
}

class ControlPanel : public QMainWindow
{
    Q_OBJECT

public:
    explicit ControlPanel(QWidget *parent = 0);
    ~ControlPanel();


private slots:
    void on_pushButton_Start_clicked();

    void on_pushButton_Reset_clicked();

    void on_pushButton_Stop_clicked();

    void on_pushButton_shutdown_clicked();

private:
    Ui::ControlPanel *ui;
};

#endif // CONTROLPANEL_H
