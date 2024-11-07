#include "vizlib_test.h"
#include <QDebug>
#include <geometry_msgs/PoseStamped.h>

// 사전 정의된 목표 좌표 설정
const double predefined_goal_x = 2.0;  // X 좌표
const double predefined_goal_y = 3.0;  // Y 좌표
const double predefined_goal_w = 1.0;  // Orientation

VizlibTest::VizlibTest(int argc, char **argv, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::VizlibTest)
{
    ui->setupUi(this);

    // QLineEdit 필드를 사전 정의된 좌표로 초기화하고 비활성화
    ui->lineEdit_goal_x->setText(QString::number(predefined_goal_x));
    ui->lineEdit_goal_y->setText(QString::number(predefined_goal_y));
    ui->lineEdit_goal_x->setEnabled(false);
    ui->lineEdit_goal_y->setEnabled(false);

    // 버튼에 슬롯 연결
    connect(ui->pushButton_start, SIGNAL(clicked()), this, SLOT(slot_btn_display()));
    connect(ui->pushButton_quit, SIGNAL(clicked()), this, SLOT(slot_btn_quit()));

    init_ros(argc, argv);  // ROS 초기화
}

void VizlibTest::init_ros(int argc, char **argv) {
    ros::NodeHandle nh;
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    // 목표 메시지 설정
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = predefined_goal_x;
    goal.pose.position.y = predefined_goal_y;
    goal.pose.orientation.w = predefined_goal_w;

    // 목표 좌표 발행
    goal_pub.publish(goal);
}

// Display 버튼 클릭 시 호출되는 슬롯 함수
void VizlibTest::slot_btn_display() {
    qDebug() << "Display button clicked. Starting goal publishing.";

    // 목표 좌표를 발행
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = predefined_goal_x;
    goal.pose.position.y = predefined_goal_y;
    goal.pose.orientation.w = predefined_goal_w;
    goal_pub.publish(goal);
}

// Quit 버튼 클릭 시 호출되는 슬롯 함수
void VizlibTest::slot_btn_quit() {
    qDebug() << "Quit button clicked. Exiting application.";
    close();  // GUI 종료
}

VizlibTest::~VizlibTest()
{
    delete ui;
}
