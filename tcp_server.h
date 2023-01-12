#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <QWidget>
#include <modbus/modbus.h>
#include <QTcpServer>
#include <QTcpSocket>
#include <QThread>
#include "global.h"
#include <unistd.h>
//#include <stdio.h>

namespace Ui {
class Tcp_Server;
}

class modbustcpThread;

class Tcp_Server : public QWidget
{
    Q_OBJECT

public:
    explicit Tcp_Server(QWidget *parent = nullptr);
    ~Tcp_Server();

public:
    Ui::Tcp_Server *ui;

    uint8_t link_state;     // 连接状态
    uint8_t link_mode;      // 连接模式
    u_int16_t robot_model;  // 机器人型号

    modbus_mapping_t * mb_mapping;
    modbustcpThread * thread1;
    modbus_t * ctx;

    QTcpServer * listen;
    QTcpSocket * connect_s;
    int sock;

    uint16_t mod_registers[MODBUS_REGISTERS_MAXNUM];    // 寄存器数组

    void ReceiveMsg(QByteArray array, QByteArray *sent_array);
    QString json_to_QString(QJsonObject json_object);   // json转QString
    QJsonObject qstring_to_json(QString json_string);   //QString转json

    void Round(float f_in, float *f_out, int decimalplace);   //保留小数位数

private:


private slots:
    void init_show_registers_list();

};

// 子线程modbus
class modbustcpThread : public QThread
{
    Q_OBJECT

public:
    modbustcpThread(Tcp_Server *statci_p);
protected:
    void run();
private:
    Tcp_Server *_p;

signals:
    // 自定义信号
    void Send_show_registers_list();

};

#endif // TCP_SERVER_H
