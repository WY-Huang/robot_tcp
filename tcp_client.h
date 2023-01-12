#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include <QWidget>
#include <QTcpSocket>
#include <modbus/modbus.h>
#include "global.h"

#include <QJsonParseError>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

QT_BEGIN_NAMESPACE
namespace Ui { class tcp_client; }
QT_END_NAMESPACE

class tcp_client : public QWidget
{
    Q_OBJECT

public:
    tcp_client(QWidget *parent = nullptr);
    ~tcp_client();

public:
    QTcpSocket *client; // tcpSocket
    modbus_t *ctx;      // modbus

    u_int16_t robot_model;  // 机器人型号

private:
    Ui::tcp_client *ui;

    uint8_t link_state; // 服务连接状态
    uint8_t link_mode;  // 服务连接模式

    uint8_t laser_state; // 激光状态
    uint8_t camera_state; // 相机状态

    void receive_msg(QByteArray msg);   // 接收msg

    QString json_to_qstring(QJsonObject json_object);   // json转QString
    QJsonObject qstring_to_json(QString json_string);   //QString转json
};
#endif // TCP_CLIENT_H
