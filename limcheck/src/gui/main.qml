import QtQuick 2.12
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.12
import "include/DolleBranding.js" as Dolle
import Ros2 1.0

ApplicationWindow {
    id: main
    width: 1280
    height: 800
    visible: true
    visibility: "FullScreen"
    title: qsTr("Limcheck")
    
    color: "#ffffff"
    Connections {
        target: Ros2
        onShutdown: Qt.quit()
    }
    // Arguments are: Topic, Message Type, Queue Size
    property var timePublisher: Ros2.createPublisher("/stigemaskine2/gui/limkontrol", "limcheck/msg/CleaningStamp", 10)
    
    ColumnLayout{
        anchors.fill: parent
        spacing: 5
        Header{
            id:header
            Layout.leftMargin: Dolle.logo.height
            Layout.rightMargin: Dolle.logo.height
            Layout.topMargin: Dolle.logo.height
            Layout.preferredHeight: Dolle.logo.height
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignTop
        }
        Content{
            id: content
            Layout.leftMargin: Dolle.logo.height
            Layout.rightMargin:Dolle.logo.height
            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.maximumHeight: main.height-header.height-footer.height
            Layout.alignment: Qt.AlignLeft

        }
        Footer{
            id:footer
            Layout.rightMargin: Dolle.logo.height
            Layout.fillWidth: true
            Layout.preferredHeight:9
            Layout.alignment: Qt.AlignBottom
        }


    }
    Component.onCompleted: {
        // Initialize ROS with the given name. The command line args are passed by the plugin
        // Optionally, you can call init with a string list ["arg1", "arg2"] after the name to use those
        // args instead of the ones supplied by the command line.
        Ros2.init("stigemaskine2_gui")
    } 
}
