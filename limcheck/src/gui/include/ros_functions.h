#ifndef ROS_FUNCTIONS_H
#define ROS_FUNCTIONS_H

class MyObject : public QObject{
   Q_OBJECT
public:
    explicit MyObject (QObject* parent = 0) : QObject(parent) {}
    Q_INVOKABLE int reken_tijden_uit(){
    return 1;
    }
};

#endif // ROS_FUNCTIONS_H