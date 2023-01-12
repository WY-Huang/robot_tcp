#include "tcp_server.h"
#include "ui_tcp_server.h"

Tcp_Server::Tcp_Server(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tcp_Server)
{
    ui->setupUi(this);

    // 创建子线程modbus
    thread1 = new modbustcpThread(this);
    connect(thread1, SIGNAL(Send_show_registers_list()), this, SLOT(init_show_registers_list()));// 子线程modbus收到数据后触发主线程更新寄存器数据

    // 参数初始化
    robot_model = ROBOT_ZHICHANG;
    ui->robot_model_box->setCurrentIndex(robot_model);

    link_mode = LINK_MODBUS_TCP;
    link_state = 0;

    ui->modbus_tcp_rbtn->setChecked(1);

    ui->send_btn->setEnabled(false);

    // 设置默认地址和端口
    ui->ip_edit->setText("127.0.0.1");
    ui->port_edit->setText(QString::number(1502));

    // 机器人位置初值
    ui->x_edit->setText("66.666");
    ui->y_edit->setText("55.555");
    ui->z_edit->setText("44.444");
    ui->rx_edit->setText("33.333");
    ui->ry_edit->setText("22.222");
    ui->rz_edit->setText("11.111");

    // 焊缝信息
    ui->delay_edit->setText("100");
    ui->search_state_edit->setText("0xff");
    ui->weld_y_edit->setText("3003");
    ui->weld_z_edit->setText("4004");
    ui->weld_w_edit->setText("5005");
    ui->weld_h_edit->setText("6006");

    connect(ui->modbus_tcp_rbtn, &QRadioButton::clicked, [=](){
        link_mode = LINK_MODBUS_TCP;
    });

    connect(ui->kawasaki_json_rbtn, &QRadioButton::clicked, [=](){
        link_mode = LINK_KAWASAKI;
    });

    // 初始化从站地址的映射
    mb_mapping = modbus_mapping_new(0, 0, MODBUS_REGISTERS_MAXNUM, 0);
    memset(mod_registers, 0, sizeof(u_int16_t)*MODBUS_REGISTERS_MAXNUM);

    init_show_registers_list();

    // 创建监听套接字
    connect(ui->start_server_btn, &QPushButton::clicked, [=](){
        if(link_mode == LINK_KAWASAKI)
        {
            if(link_state == 0)
            {
                link_state = 1;
                // 设置监听
                listen = new QTcpServer(this);
                listen->listen(QHostAddress(ui->ip_edit->text()), ui->port_edit->text().toInt());

                // 监听新连接
                connect(listen, &QTcpServer::newConnection, [=](){
                    connect_s = listen->nextPendingConnection();

//                    ui->record->append("有新的连接了");
                    // 接收数据
                    connect(connect_s, &QTcpSocket::readyRead, [=](){
                        if(link_mode == LINK_KAWASAKI)
                        {
                            QByteArray artemp = "接收：";
                            QByteArray array = connect_s->readAll();
                            QByteArray out_array;
//                            ui->record->append(artemp+array);
//                            ReceiveMsg(array, &out_array);
                        }
//                        else if(link_mode == LINK_NORMAL_RTU)
//                        {}
                    });
                });
                ui->start_server_btn->setText("关闭服务器");
                ui->modbus_tcp_rbtn->setEnabled(false);
                ui->kawasaki_json_rbtn->setEnabled(false);
                ui->send_btn->setEnabled(false);
            }
            else
            {
                link_state=0;
                listen->close();
                delete listen;
                ui->start_server_btn->setText("开启服务器");
                ui->modbus_tcp_rbtn->setEnabled(true);
                ui->kawasaki_json_rbtn->setEnabled(true);
                ui->send_btn->setEnabled(false);
            }
        }
        else if(link_mode==LINK_MODBUS_TCP)
        {
            if(link_state==0)
            {
                link_state=1;
                thread1->start();
                ui->start_server_btn->setText("关闭服务器");
                ui->modbus_tcp_rbtn->setEnabled(false);
                ui->kawasaki_json_rbtn->setEnabled(false);
                ui->send_btn->setEnabled(false);
            }
            else
            {
                link_state=0;
                ui->start_server_btn->setText("开启服务器");
                ui->modbus_tcp_rbtn->setEnabled(true);
                ui->kawasaki_json_rbtn->setEnabled(true);
                ui->send_btn->setEnabled(false);
            }
        }
    });

    // 写入机器人位置信息
    connect(ui->write_pos_btn, &QPushButton::clicked, [=](){
        float PosX = ui->x_edit->text().toFloat();
        float PosY = ui->y_edit->text().toFloat();
        float PosZ = ui->z_edit->text().toFloat();
        float PosRX = ui->rx_edit->text().toFloat();
        float PosRY = ui->ry_edit->text().toFloat();
        float PosRZ = ui->rz_edit->text().toFloat();

        QString msg;
        msg="("+ui->x_edit->text()+","+ui->y_edit->text()+","+ui->z_edit->text()+","+
                ui->rx_edit->text()+","+ui->ry_edit->text()+","+ui->rz_edit->text()+")";
        ui->record_tb->append("Pos:" + msg); // 将数据显示到记录框

        float *freg=(float *)&mod_registers[MODBUS_ADD_POS];
        freg[0]=PosX;
        freg[1]=PosY;
        freg[2]=PosZ;
        freg[3]=PosRX;
        freg[4]=PosRY;
        freg[5]=PosRZ;
        init_show_registers_list();

        if(link_mode==LINK_KAWASAKI)
        {

        }
        else if(link_mode==LINK_MODBUS_TCP)
        {
            float *f_tab_reg=(float *)&mb_mapping->tab_registers[MODBUS_ADD_POS];
            f_tab_reg[0]=PosX;
            f_tab_reg[1]=PosY;
            f_tab_reg[2]=PosZ;
            f_tab_reg[3]=PosRX;
            f_tab_reg[4]=PosRY;
            f_tab_reg[5]=PosRZ;
        }
    });

    // 写入焊缝信息
    connect(ui->write_weld_btn, &QPushButton::clicked, [=](){
        std::string str_data;
        u_int16_t data_delay, data_search_state, data_weld_y, data_weld_z, data_weld_w, data_weld_h;
        u_int16_t * u16_reg = (u_int16_t *)mod_registers;
        QString msg;

        // str转int
        str_data = ui->delay_edit->text().toStdString();
        data_delay = std::stoi(str_data, NULL, 0);
        str_data = ui->search_state_edit->text().toStdString();
        data_search_state = std::stoi(str_data, NULL, 0);
        str_data = ui->weld_y_edit->text().toStdString();
        data_weld_y = std::stoi(str_data, NULL, 0);
        str_data = ui->weld_z_edit->text().toStdString();
        data_weld_z = std::stoi(str_data, NULL, 0);
        str_data = ui->weld_w_edit->text().toStdString();
        data_weld_w = std::stoi(str_data, NULL, 0);
        str_data = ui->weld_h_edit->text().toStdString();
        data_weld_h = std::stoi(str_data, NULL, 0);

        if(robot_model==ROBOT_ZHICHANG)
        {
            u16_reg[MODBUS_ADD_DELAY] = data_delay;
            u16_reg[MODBUS_ADD_SEARCHSTAT] = data_search_state;
            u16_reg[MODBUS_ADD_WELD_Y_POS] = data_weld_y;
            u16_reg[MODBUS_ADD_WELD_Z_POS] = data_weld_z;
            u16_reg[MODBUS_ADD_WELD_W_SIZE] = data_weld_w;
            u16_reg[MODBUS_ADD_WELD_H_SIZE] = data_weld_h;

            mb_mapping->tab_registers[MODBUS_ADD_DELAY] = data_delay;
            mb_mapping->tab_registers[MODBUS_ADD_SEARCHSTAT] = data_search_state;
            mb_mapping->tab_registers[MODBUS_ADD_WELD_Y_POS] = data_weld_y;
            mb_mapping->tab_registers[MODBUS_ADD_WELD_Z_POS] = data_weld_z;
            mb_mapping->tab_registers[MODBUS_ADD_WELD_W_SIZE] = data_weld_w;
            mb_mapping->tab_registers[MODBUS_ADD_WELD_H_SIZE] = data_weld_h;
        }

        msg="设置:\n延迟:"+QString::number(data_delay)+"ms\n搜索状态:"
                +QString::number(data_search_state)+"\n焊缝坐标Y:"
                +QString::number(data_weld_y/100.0)+"mm\n焊缝坐标Z:"
                +QString::number(data_weld_z/100.0)+"mm\n焊缝宽度:"
                +QString::number(data_weld_w/100.0)+"mm\n焊缝高度:"
                +QString::number(data_weld_h/100.0)+"mm";
        ui->record_tb->append(msg); // 将数据显示到记录框
        init_show_registers_list();
    });

    // 手动写入寄存器数据
    connect(ui->write_reg_btn, &QPushButton::clicked, [=](){

        std::string stradd = ui->reg_addr_edit->text().toStdString();
        int addr = std::stoi(stradd, NULL, 0);
        std::string strdata = ui->reg_value_edit->text().toStdString();
        uint16_t data = std::stoi(strdata, NULL, 0);

        if(addr>=0&&addr<MODBUS_REGISTERS_MAXNUM)
        {
            mod_registers[addr] = data;
        }
        mb_mapping->tab_registers[addr] = data;
        init_show_registers_list();
    });

    // 发送数据

}

Tcp_Server::~Tcp_Server()
{
    if(link_state == 1)
    {
        link_state = 0;
        thread1->quit();
    }
    delete ui;
}

// 初始化寄存器数组中的数据
void Tcp_Server::init_show_registers_list()
{
    ui->register_tb->clear();
    for(int n=0; n<MODBUS_REGISTERS_MAXNUM; n++)
    {
        QString msg = QString::number(n, 16); // int地址转16进制
        QString data = QString::number(mod_registers[n], 16);   // uint16_t值转16进制
        ui->register_tb->append("[0x"+msg+"]\t0x"+data);
    }
}

#if 0
// 接收数据
void Tcp_Server::ReceiveMsg(QByteArray array, QByteArray *sent_array)
{
    QByteArray outarray;

    if(link_mode==LINK_KAWASAKI)
    {
        /***********************/
        //这里写数据
    #ifdef USE_PARENTHESES_INSTEAD_QUOTATION
        for(unsigned int n=0;n<array.size();n++)
        {
            if(array[n]=='('||array[n]==')')
            {
               array[n]='"';
            }
        }
    #endif
        QJsonObject json=QstringToJson(array);
        QJsonObject::Iterator it;

        QJsonObject anser;
        for(it=json.begin();it!=json.end();it++)//遍历Key
        {
            QString keyString=it.key();
            if(keyString==ASK_LASER_KEY_KAWASAKI)//激光
            {
                QString valueString=it.value().toString();
                if(valueString==ASK_LASER_OPEN_KAWASAKI)//请求开激光
                {
                    if(robot_mod==ROBOT_ZHICHANG)
                    {
                        mod_registers[MODBUS_ADD_LASER]=1;
                    }
                    else
                    {
                        mod_registers[MODBUS_NABO_LASER]=1;
                    }
                    anser.insert(ASE_LASER_KEY_KAWASAKI, ASE_LASER_OPENOK_KAWASAKI);
                    init_show_registers_list();
                    ui->record->append("解析:请求开激光"); // 将数据显示到记录框
                }
                else if(valueString==ASK_LASER_CLOSE_KAWASAKI)//请求关激光
                {
                    if(robot_mod==ROBOT_ZHICHANG)
                    {
                        mod_registers[MODBUS_ADD_LASER]=0;
                    }
                    else
                    {
                        mod_registers[MODBUS_NABO_LASER]=0;
                    }
                    anser.insert(ASE_LASER_KEY_KAWASAKI, ASE_LASER_CLOSEOK_KAWASAKI);
                    init_show_registers_list();
                    ui->record->append("解析:请求关激光");
                }
            }
            else if(keyString==ASK_CAMERA_KEY_KAWASAKI)//相机
            {
                QString valueString=it.value().toString();
                if(valueString==ASK_CAMERA_OPEN_KAWASAKI)//请求开相机
                {
                    if(robot_mod==ROBOT_ZHICHANG)
                    {
                        mod_registers[MODBUS_ADD_LASER]=1;
                    }
                    else
                    {
                        mod_registers[MODBUS_NABO_WELDING]=1;
                    }
                    anser.insert(ASE_CAMERA_KEY_KAWASAKI, ASE_CAMERA_OPENOK_KAWASAKI);
                    init_show_registers_list();
                    ui->record->append("解析:请求开相机"); // 将数据显示到记录框
                }
                else if(valueString==ASK_CAMERA_CLOSE_KAWASAKI)//请求关相机
                {
                    if(robot_mod==ROBOT_ZHICHANG)
                    {
                        mod_registers[MODBUS_ADD_LASER]=0;
                    }
                    else
                    {
                        mod_registers[MODBUS_NABO_WELDING]=0;
                    }
                    anser.insert(ASE_CAMERA_KEY_KAWASAKI, ASE_CAMERA_CLOSEOK_KAWASAKI);
                    init_show_registers_list();
                    ui->record->append("解析:请求关相机");
                }
            }
            else if(keyString==ASK_TASKNUM_KEY_KAWASAKI)//任务号
            {
                QString valueString=it.value().toString();
                int ID=valueString.toInt();
                if(robot_mod==ROBOT_ZHICHANG)
                {
                    mod_registers[MODBUS_ADD_TASKNUM]=ID;
                }
                else
                {
                    mod_registers[MODBUS_NABO_TASKNUM]=ID;
                }
                anser.insert(ASE_TASKNUM_KEY_KAWASAKI, valueString+ASE_TASKNUM_SETOK_KAWASAKI);
                init_show_registers_list();
                ui->record->append("解析:设置任务号:"+valueString);
            }
            else if(keyString==ASK_DELAY_KEY_KAWASAKI)//获取焊缝延迟
            {
                switch(it->type())
                {
                    case QJsonValue::String:
                    {
                        QString valueString=it.value().toString();
                        if(valueString==ASK_DELAY_ONCE_KAWASAKI)//获取1次
                        {
                            u_int16_t u16_data=mod_registers[MODBUS_ADD_DELAY];
                            QString msg;
                            if(ui->checkBox_daley->isChecked()==true)
                            {
                                msg=ASE_DELAY_FAILED_KAWASAKI;
                            }
                            else
                            {
                                msg=QString::number(u16_data);
                            }
                            anser.insert(ASE_DELAY_KEY_KAWASAKI, msg);
                            ui->record->append("解析:请求获取1次延迟");
                        }
                    }
                    break;
                    default:
                    break;
                 }
            }
            else if(keyString==ASK_SEARCHSTAT_KEY_KAWASAKI)//获取搜索状态
            {
                switch(it->type())
                {
                    case QJsonValue::String:
                    {
                        QString valueString=it.value().toString();
                        if(valueString==ASK_SEARCHSTAT_ONCE_KAWASAKI)//获取1次
                        {
                            u_int16_t u16_data=mod_registers[MODBUS_ADD_SEARCHSTAT];
                            if(robot_mod==ROBOT_ZHICHANG)
                            {
                                u16_data=mod_registers[MODBUS_ADD_SEARCHSTAT];
                            }
                            else if(robot_mod==ROBOT_NABOTE)
                            {
                                u16_data=mod_registers[MODBUS_NABO_SEARCHSTAT];
                            }
                            QString msg="ng";
                            if(ui->checkBox_searchstat->isChecked()==true)
                            {
                                msg=ASE_SEARCHSTAT_FAILED_KAWASAKI;
                            }
                            else
                            {
                                if(u16_data==0)
                                {
                                    msg="ng";
                                }
                                else if(u16_data==255)
                                {
                                    msg="ok";
                                }
                            }
                            anser.insert(ASE_SEARCHSTAT_KEY_KAWASAKI, msg);
                            ui->record->append("解析:请求获取1次搜索状态");
                        }
                    }
                    break;
                    default:
                    break;
                 }
            }
            else if(keyString==ASK_POS2_KEY_KAWASAKI)//获取2维坐标
            {
                switch(it->type())
                {
                    case QJsonValue::String:
                    {
                        QString valueString=it.value().toString();
                        if(valueString==ASK_POS2_ONCE_KAWASAKI)//获取1次
                        {
                            int16_t i16_y=mod_registers[MODBUS_ADD_WELD_Y_POS],i16_z=mod_registers[MODBUS_ADD_WELD_Z_POS];
                            if(robot_mod==ROBOT_ZHICHANG)
                            {
                                i16_y=mod_registers[MODBUS_ADD_WELD_Y_POS];
                                i16_z=mod_registers[MODBUS_ADD_WELD_Z_POS];
                            }
                            else if(robot_mod==ROBOT_NABOTE)
                            {
                                i16_y=mod_registers[MODBUS_NABO_WELD_Y_POS];
                                i16_z=mod_registers[MODBUS_NABO_WELD_Z_POS];
                            }

                            float f16_weld_Y=i16_y/100.0;
                            float f16_weld_Z=i16_z/100.0;
                            QJsonArray versionArray;
                            if(ui->checkBox_pos2->isChecked()==true)
                            {
                                anser.insert(ASE_POS2_KEY_KAWASAKI, ASE_POS2_FAILED_KAWASAKI);
                            }
                            else
                            {
                                versionArray.append(f16_weld_Y);
                                versionArray.append(f16_weld_Z);
                                anser.insert(ASE_POS2_KEY_KAWASAKI, QJsonValue(versionArray));
                            }
                            ui->record->append("解析:请求获取1次2维坐标");
                        }
                    }
                    break;
                    default:
                    break;
                 }
            }
            else if(keyString==ASK_SIZE2_KEY_KAWASAKI)//获取焊缝宽高
            {
                switch(it->type())
                {
                    case QJsonValue::String:
                    {
                        QString valueString=it.value().toString();
                        if(valueString==ASK_SIZE2_ONCE_KAWASAKI)//获取1次
                        {
                            float f16_weld_W=mod_registers[MODBUS_ADD_WELD_W_SIZE]/100.0;
                            float f16_weld_H=mod_registers[MODBUS_ADD_WELD_H_SIZE]/100.0;
                            QJsonArray versionArray;
                            if(ui->checkBox_size2->isChecked()==true)
                            {
                                 anser.insert(ASE_SIZE2_KEY_KAWASAKI, ASE_SIZE2_FAILED_KAWASAKI);
                            }
                            else
                            {
                                versionArray.append(f16_weld_W);
                                versionArray.append(f16_weld_H);
                                anser.insert(ASE_SIZE2_KEY_KAWASAKI, QJsonValue(versionArray));
                            }
                            ui->record->append("解析:请求获取1次2维尺寸");
                        }
                    }
                    break;
                    default:
                    break;
                 }
            }
            else if(keyString==ASK_POS6_KEY_KAWASAKI)//获取6维坐标
            {
                switch(it->type())
                {
                    case QJsonValue::String:
                    {
                        QString valueString=it.value().toString();
                        if(valueString==ASK_POS6_ONCE_KAWASAKI)//获取1次
                        {
                            float *freg=(float *)&mod_registers[MODBUS_ADD_POS];
                            QJsonArray versionArray;
                            for(int i=0;i<6;i++)
                            {
                                versionArray.append(freg[i]);
                            }
                            anser.insert(ASE_POS6_KEY_KAWASAKI, QJsonValue(versionArray));
                            ui->record->append("解析:请求获取1次6维坐标");
                        }
                    }
                    break;
                    default:
                    break;
                 }
            }
        }
        if(anser.size()!=0)
        {
            QString msg=JsonToQstring(anser);
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
        #ifdef DEL_SPACE_AND_LINEN
            QString msg2;
            for(unsigned int n=0;n<msg.size();n++)
            {
                if(msg[n]!='\n')
                {
                    msg2.push_back(msg[n]);
                }
            }
            msg=msg2;
        #endif
            conn->write(msg.toUtf8());
            ui->record->append("发送:" + msg); // 将数据显示到记录框
        }
    }


    *sent_array=outarray;
}
#endif

// 子线程modbus
modbustcpThread::modbustcpThread(Tcp_Server *statci_p)
{
    _p = statci_p;  // 传入主线程窗口对象的指针，来操作窗口对象，否则子线程无权对窗口进行任何更改（也可采用信号槽完成线程间的通信）
}

void modbustcpThread::run()
{
    while(_p->link_state==1)
    {
        QString server_ip=_p->ui->ip_edit->text();
        QString server_port=_p->ui->port_edit->text();
   //   _p->ctx = modbus_new_tcp(server_ip.toUtf8(), server_port.toInt());
        _p->ctx = modbus_new_tcp(NULL, server_port.toInt());    // NULL 值可用于在服务器模式下侦听任何地址
        _p->sock = modbus_tcp_listen(_p->ctx, 1);   // 最大监听1路
        modbus_tcp_accept(_p->ctx, &_p->sock);      // 提取第一个连接，并返回一个新的套接字

        while(_p->link_state==1)
        {
            int rc;
            uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
            rc = modbus_receive(_p->ctx, query);    // 接收请求，存在query中
            if (rc > 0) {
              modbus_reply(_p->ctx, query, rc, _p->mb_mapping); // 响应请求, 读取或写入寄存器
              for(int n=0;n<MODBUS_REGISTERS_MAXNUM;n++)
              {
                if(_p->mb_mapping->tab_registers[n]!=_p->mod_registers[n])
                {
                    _p->mod_registers[n]=_p->mb_mapping->tab_registers[n];
                    QString msg=QString::number(n,16);
                    QString data=QString::number(_p->mb_mapping->tab_registers[n],16);
                    _p->ui->record_tb->append("ChangeRegister:[0x"+msg+"]: "+data);
                }
              }
              emit Send_show_registers_list();
            }
            else if (rc == -1) {
              /* Connection closed by the client or error */
              break;
            }

        }

        close(_p->sock);
        modbus_close(_p->ctx);
        modbus_free(_p->ctx);
    }
}
