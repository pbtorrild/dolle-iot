import QtQuick 2.12
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.12

ApplicationWindow {
    id: window
    width: 1280
    height: 800
    visible: true
    visibility: "FullScreen"
    title: qsTr("Limcheck")
    
    color: "#ffffff"
       

    ColumnLayout{
        anchors.fill: parent
        spacing: 5
        Header{
            id:header
            Layout.leftMargin: 85
            Layout.rightMargin: 85
            Layout.topMargin: 85
            Layout.preferredHeight: 85
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignTop
        }
        Content{
            id: content
            Layout.leftMargin: 85
            Layout.rightMargin:85
            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.maximumHeight: window.height-header.height-footer.height

        }
        Footer{
            id:footer
            Layout.rightMargin: 85
            Layout.fillWidth: true
            Layout.preferredHeight:9
            Layout.alignment: Qt.AlignBottom
        }


    }
}
