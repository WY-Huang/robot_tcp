#include "tcp_client.h"
#include "ui_tcp_client.h"

tcp_client::tcp_client(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::tcp_client)
{
    ui->setupUi(this);

    // 设置默认地址和端口
    ui->ip_edit->setText("127.0.0.1");
    ui->port_edit->setText(QString::number(1502));

    client = new QTcpSocket(this);  // 创建客户端

    // 参数初始化
    link_state = 0;
    link_mode = LINK_MODBUS_TCP;
    robot_model = ROBOT_ZHICHANG;
    laser_state = 0;
    camera_state = 0;

    ui->robot_box->setCurrentIndex(robot_model);
    ui->modbus_tcp_rbtn->setChecked(1);
    ui->send_btn->setEnabled(false);
    ui->read_register_btn->setEnabled(false);
    ui->move_to_pos->setEnabled(false);
    ui->get_pos_btn->setEnabled(false);

    // 机器人位置初值
    ui->x_edit->setText("66.666");
    ui->y_edit->setText("55.555");
    ui->z_edit->setText("44.444");
    ui->rx_edit->setText("33.333");
    ui->ry_edit->setText("22.222");
    ui->rz_edit->setText("11.111");
    ui->sp_edit->setText("99.999");

    // modbus按钮
    connect(ui->modbus_tcp_rbtn, &QRadioButton::clicked, [=](){
        link_mode = LINK_MODBUS_TCP;
    });

    // 川崎按钮
    connect(ui->kawasaki_json_rbtn, &QRadioButton::clicked, [=](){
        link_mode = LINK_KAWASAKI;
    });

    // 连接服务器
    connect(ui->connect_server_btn, &QPushButton::clicked, [=](){
        if(link_mode == LINK_KAWASAKI)
        {
            if(link_state == 0)
            {
                link_state = 1;
                client->connectToHost(ui->ip_edit->text(), ui->port_edit->text().toInt());
                ui->connect_server_btn->setText("断开服务器");
                ui->kawasaki_json_rbtn->setEnabled(false);
                ui->send_btn->setEnabled(true);
                ui->read_register_btn->setEnabled(true);
                ui->move_to_pos->setEnabled(true);
                ui->get_pos_btn->setEnabled(true);

            }
            else
            {
                link_state = 0;
                client->disconnectFromHost();
                ui->connect_server_btn->setText("连接服务器");
                ui->kawasaki_json_rbtn->setEnabled(true);
                ui->send_btn->setEnabled(false);
                ui->read_register_btn->setEnabled(false);
                ui->move_to_pos->setEnabled(false);
                ui->get_pos_btn->setEnabled(false);

            }
        }
        else if(link_mode == LINK_MODBUS_TCP)
        {
            if(link_state == 0)
            {
                QString server_ip = ui->ip_edit->text();
                QString server_port = ui->port_edit->text();
                ctx = modbus_new_tcp(server_ip.toUtf8(), server_port.toInt());
                if(modbus_connect(ctx) == -1)
                {
                    fprintf(stderr, "Connection failed: %s\n",modbus_strerror(errno));
                    modbus_free(ctx);
                }
                else
                {
                    link_state = 1;
                    ui->connect_server_btn->setText("断开服务器");
                    ui->kawasaki_json_rbtn->setEnabled(false);
                    ui->send_btn->setEnabled(true);
                    ui->read_register_btn->setEnabled(true);
                    ui->move_to_pos->setEnabled(true);
                    ui->get_pos_btn->setEnabled(true);
                }
            }
            else
            {
                link_state = 0;
                modbus_close(ctx);
                modbus_free(ctx);
                ui->connect_server_btn->setText("连接服务器");
                ui->kawasaki_json_rbtn->setEnabled(true);
                ui->send_btn->setEnabled(false);
                ui->read_register_btn->setEnabled(false);
                ui->move_to_pos->setEnabled(false);
                ui->get_pos_btn->setEnabled(false);
            }
        }
    });

    // 获取机器人位置坐标
    connect(ui->get_pos_btn, &QPushButton::clicked, [=](){
        if(link_mode == LINK_MODBUS_TCP)
        {
            int nb_points = 12;
            uint16_t *tab_reg = new uint16_t[nb_points];    // 创建数组保存读取寄存器的结果
            int rc = modbus_read_registers(ctx, MODBUS_ADD_POS, nb_points, tab_reg);
            if(rc!=nb_points)
            {
                fprintf(stderr, "read pose failed\n", NULL);
            }
            else
            {
                float *f_tab_reg=(float*)tab_reg;   // uint16_t转float
                float posX=f_tab_reg[0];
                float posY=f_tab_reg[1];
                float posZ=f_tab_reg[2];
                float posRX=f_tab_reg[3];
                float posRY=f_tab_reg[4];
                float posRZ=f_tab_reg[5];
                QString s_pos;
                s_pos="("+QString::number(posX)+", "+QString::number(posY)+", "+QString::number(posZ)+", "
                        +QString::number(posRX)+", "+QString::number(posRY)+", "+QString::number(posRZ)+")";
                ui->pos_label->setText(s_pos);
            }
            delete[] tab_reg;
        }
        else if(link_mode==LINK_KAWASAKI)
        {
            QJsonObject json;
         // json.insert(ASK_POS2_KEY_KAWASAKI, ASK_POS2_ONCE_KAWASAKI);
            json.insert(ASK_POS6_KEY_KAWASAKI, ASK_POS6_ONCE_KAWASAKI);
            QString msg=json_to_qstring(json);
        #ifdef USE_PARENTHESES_INSTEAD_QUOTATION
            int time_s=0;
            for(unsigned int n=0;n<msg.size();n++)
            {
                if(msg[n]=='"')   //"
                {
                    if(time_s==0)
                    {
                        msg[n]='('; //(
                        time_s=1;
                    }
                    else if(time_s==1)
                    {
                        msg[n]=')'; //)
                        time_s=0;
                    }
                }
            }
        #endif
            client->write(msg.toUtf8());
            ui->record_tb->append("发送:" + msg); // 将数据显示到记录框
        }
    });

    // 激光控制按钮
    connect(ui->laser_btn, &QPushButton::clicked, [=](){
        if(laser_state == 0)
        {
            if(link_mode==LINK_MODBUS_TCP)
            {
                uint16_t tab_reg[1];

                if(robot_model==ROBOT_ZHICHANG)
                {
                    tab_reg[0]=0xff;
                    int rc = modbus_write_registers(ctx, MODBUS_ADD_LASER, 1, tab_reg);
                    if(rc!=1)
                    {
                        ui->record_tb->append("激光器启动设置失败");
                    }
                    else
                    {
                        ui->record_tb->append("激光器启动设置成功");
                        laser_state = 1;
                        ui->laser_btn->setText("关闭激光");
                    }
                }
            }
        }
        else
        {
            if(link_mode==LINK_MODBUS_TCP)
            {
                uint16_t tab_reg[1];
                tab_reg[0] = 0;
                if(robot_model == ROBOT_ZHICHANG)
                {
                    int rc=modbus_write_registers(ctx, MODBUS_ADD_LASER, 1, tab_reg);
                    if(rc!=1)
                    {
                        ui->record_tb->append("激光器关闭设置失败");
                    }
                    else
                    {
                        ui->record_tb->append("激光器关闭设置成功");
                        laser_state = 0;
                        ui->laser_btn->setText("打开激光");
                    }
                }
            }
        }
    });

    // 任务号设置
    connect(ui->write_task_btn, &QPushButton::clicked, [=](){
        if(link_mode==LINK_MODBUS_TCP)
        {
            uint16_t tab_reg[1];
            tab_reg[0] = ui->task_edit->text().toInt();
            if(robot_model == ROBOT_ZHICHANG)
            {
                int rc = modbus_write_registers(ctx, MODBUS_ADD_TASKNUM, 1, tab_reg);
                if(rc != 1)
                {
                    ui->record_tb->append("任务号设置失败");
                }
                else
                {
                    ui->record_tb->append("任务号设置成功");
                }
            }
        }
    });

    // 获取焊缝信息
    connect(ui->get_weld_btn, &QPushButton::clicked, [=](){
        if(link_mode == LINK_MODBUS_TCP)
        {
            int real_readnum;
            if(robot_model == ROBOT_ZHICHANG)
            {
                uint16_t tab_reg[3];
                real_readnum = modbus_read_registers(ctx, MODBUS_ADD_SEARCHSTAT, 3, tab_reg);
                if(real_readnum<0)
                {
                    ui->record_tb->append("坐标位置获取失败");
                }
                else
                {
                    QString msg;
                    if(tab_reg[0] == 0)
                    {
                        msg="无效(";
                    }
                    else if(tab_reg[0]==0xff)
                    {
                        msg="有效(";
                    }
                    float y = (int16_t)tab_reg[1]/100.0;
                    float z = (int16_t)tab_reg[2]/100.0;
                    msg=msg+QString::number(y)+", "+QString::number(z)+")";
                    ui->weld_label->setText(msg);
                }
            }
        }
    });
}

tcp_client::~tcp_client()
{
    delete ui;
}

QJsonObject tcp_client::qstring_to_json(QString jsonString)
{
    QJsonDocument jsonDocument = QJsonDocument::fromJson(jsonString.toLocal8Bit().data());
    if(jsonDocument.isNull())
    {
        qDebug()<< "String NULL"<< jsonString.toLocal8Bit().data();
    }
    QJsonObject jsonObject = jsonDocument.object();
    return jsonObject;
}

QString tcp_client::json_to_qstring(QJsonObject jsonObject)
{
    return QString(QJsonDocument(jsonObject).toJson());
}
