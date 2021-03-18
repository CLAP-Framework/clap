#include "ui_Shougang.h"
#include <QWidget>
#include <QtGui>

class RoutingWindow : public QWidget
{
    Q_OBJECT
    public:
        RoutingWindow(float* destination_x, float* destination_y, QWidget *parent = 0);
    private:
        Ui::shougang ui;
        float* exp_x;
        float* exp_y;
    private slots:
        void slot_destination1();
        void slot_destination2();
        void slot_destination3();
        void slot_destination4();
        void slot_destination5();
        void slot_destination6();
        void slot_destination7();
        void slot_destination8();
};