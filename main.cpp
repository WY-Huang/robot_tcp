#include "tcp_client.h"
#include "tcp_server.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    tcp_client w;
    w.setWindowTitle("客户端");
    w.show();

    Tcp_Server s;
    s.setWindowTitle("服务端");
    s.show();
    return a.exec();
}
