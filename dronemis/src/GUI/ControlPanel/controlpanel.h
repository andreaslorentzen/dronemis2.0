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
    ~ControlPanel(void);


private slots:
    void on_pushButton_Start_clicked(void);

    void on_pushButton_Reset_clicked(void);

    void on_pushButton_Stop_clicked(void);

    void on_pushButton_shutdown_clicked(void);

private:
    Ui::ControlPanel *ui;
};

#endif // CONTROLPANEL_H
