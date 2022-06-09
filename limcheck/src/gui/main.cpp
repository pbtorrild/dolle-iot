#include <QApplication>
#include <QQmlApplicationEngine>
#include <QFontDatabase>
//#include <ros_functions.h>


int main(int argc, char *argv[])
{
    qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));
    //qmlRegisterType<ROS_QT_BRIDGE>("ROS_QT_BRIDGE", 1, 0, "RosQtBridge");

    #if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
        QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    #endif
    QApplication app(argc, argv);

    QQmlApplicationEngine engine;

    const QUrl url(QStringLiteral("qrc:/main.qml"));

    engine.load(url);
    if(engine.rootObjects().isEmpty()){
        return -1;
    }

    return app.exec();
}

