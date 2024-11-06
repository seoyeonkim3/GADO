#ifndef VIZLIB_TEST_H
#define VIZLIB_TEST_H

#include <QWidget>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include "ui_vizlib_test.h"  // UI Çì´õ ÆÄÀÏÀ» ¿Ã¹Ù¸£°Ô Æ÷ÇÔ½ÃÅµ´Ï´Ù.

namespace Ui {
class VizlibTest;
}

class VizlibTest : public QWidget
{
    Q_OBJECT

public:
    explicit VizlibTest(int argc, char **argv, QWidget *parent = 0);
    ~VizlibTest();

    void init_ros(int argc, char **argv);

private slots:
    void slot_btn_display();
    void slot_btn_quit();

private:
    rviz::VisualizationManager *manager_;
    rviz::RenderPanel * render_panel_;
    Ui::VizlibTest *ui;
    ros::Publisher goal_pub;
};

#endif // VIZLIB_TEST_H
