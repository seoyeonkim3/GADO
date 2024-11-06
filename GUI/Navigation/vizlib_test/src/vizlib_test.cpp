#include "vizlib_test.h"
#include <QString>
#include <QDebug>
#include <QMessageBox>
#include <geometry_msgs/PoseStamped.h>

VizlibTest::VizlibTest(int argc, char **argv, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::VizlibTest)
{
    ui->setupUi(this);
    init_ros(argc, argv);

    // RViz 렌더 패널 초기화
    render_panel_ = new rviz::RenderPanel;
    manager_ = new rviz::VisualizationManager(render_panel_);
    render_panel_->initialize(manager_->getSceneManager(), manager_);
    manager_->initialize();

    // RViz 렌더 패널을 UI에 추가
    ui->verticalLayout->addWidget(render_panel_);

    // 버튼 클릭 시 연결
    QObject::connect(ui->pushButton_start, SIGNAL(clicked()), this, SLOT(slot_btn_display()));
    QObject::connect(ui->pushButton_quit, SIGNAL(clicked()), this, SLOT(slot_btn_quit()));
}

VizlibTest::~VizlibTest()
{
    delete ui;
}

void VizlibTest::init_ros(int argc, char **argv)
{
    ros::init(argc, argv, "vizlib_test", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10); // goal_pub 초기화
}

void VizlibTest::slot_btn_display()
{
    static bool display_enabled = false;

    if (!display_enabled)
    {
        manager_->removeAllDisplays();

        // Map 및 LaserScan 디스플레이 설정
        rviz::Display* map_display = manager_->createDisplay("rviz/Map", "adjustable map", true);
        map_display->subProp("Topic")->setValue("/map");

        rviz::Display* laser_display = manager_->createDisplay("rviz/LaserScan", "adjustable scan", true);
        laser_display->subProp("Topic")->setValue("/scan");

        manager_->startUpdate();
        ui->pushButton_start->setText("Hide Display");

        // 목표 설정
        QString goal_x_str = ui->lineEdit_goal_x->text();
        QString goal_y_str = ui->lineEdit_goal_y->text();

        // 입력 값이 비어있는지 확인
        if (goal_x_str.isEmpty() || goal_y_str.isEmpty())
        {
            QMessageBox::warning(this, "Input Error", "Please enter both Goal X and Goal Y.");
            return;
        }

        double goal_x = goal_x_str.toDouble();
        double goal_y = goal_y_str.toDouble();
        double goal_w = 1.0; // 방향 설정

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = goal_x;
        goal.pose.position.y = goal_y;
        goal.pose.orientation.w = goal_w;

        goal_pub.publish(goal); // 목표 위치 발행
    }
    else
    {
        // Display 숨기기
        manager_->stopUpdate();
        ui->pushButton_start->setText("Show Display");
    }

    display_enabled = !display_enabled;
}

void VizlibTest::slot_btn_quit()
{
    this->close();
}

