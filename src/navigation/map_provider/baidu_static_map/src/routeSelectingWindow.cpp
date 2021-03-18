#include "routeSelectingWindow.h"

RoutingWindow::RoutingWindow(float* destination_x, float* destination_y, QWidget *parent) : QWidget(parent)
{
    ui.setupUi(this);
    connect(ui.destination1, SIGNAL(clicked()), this, SLOT(slot_destination1()));
    connect(ui.destination2, SIGNAL(clicked()), this, SLOT(slot_destination2()));
    connect(ui.destination3, SIGNAL(clicked()), this, SLOT(slot_destination3()));
    connect(ui.destination4, SIGNAL(clicked()), this, SLOT(slot_destination4()));
    connect(ui.destination5, SIGNAL(clicked()), this, SLOT(slot_destination5()));
    connect(ui.destination6, SIGNAL(clicked()), this, SLOT(slot_destination6()));
    connect(ui.destination7, SIGNAL(clicked()), this, SLOT(slot_destination7()));
    connect(ui.destination8, SIGNAL(clicked()), this, SLOT(slot_destination8()));

    ui.destination1->setText("冬奥纪念品店");
    ui.destination2->setText("星巴克");
    ui.destination3->setText("奥组委");
    ui.destination4->setText("三高炉广场");
    ui.destination5->setText("冬训中心");
    ui.destination6->setText("四高炉");
    ui.destination7->setText("群明湖");
    ui.destination8->setText("老起点");
    exp_x = destination_x;
    exp_y = destination_y;
}

void RoutingWindow::slot_destination1()
{
    *exp_x = -842.01 + 428191;
    *exp_y = 1510.74 + 4417667;
    this->close();
}

void RoutingWindow::slot_destination2()
{
    *exp_x = -918.76 + 428191;
    *exp_y = 1582.27 + 4417667;
    this->close();
}

void RoutingWindow::slot_destination3()
{
    *exp_x = -1224.93 + 428191;
    *exp_y = 1575.76 + 4417667;
    this->close();
}

void RoutingWindow::slot_destination4()
{
    *exp_x = -891.4 + 428191;
    *exp_y = 1355.25 + 4417667;
    this->close();
}

void RoutingWindow::slot_destination5()
{
    *exp_x = -1138.8 + 428191;
    *exp_y = 965.60 + 4417667;
    this->close();
}

void RoutingWindow::slot_destination6()
{
    *exp_x = -725.0 + 428191;
    *exp_y = 991.5 + 4417667;
    this->close();
}

void RoutingWindow::slot_destination7()
{
    *exp_x = -693.3 + 428191;
    *exp_y = 666.91 + 4417667;
    this->close();
}

void RoutingWindow::slot_destination8()
{
    *exp_x = -695.25 + 428191;
    *exp_y = 339.65 + 4417667;
    this->close();
}

