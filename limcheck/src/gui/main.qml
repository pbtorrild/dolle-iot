import QtQuick 2.12
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.12
import "include/DolleBranding.js" as Dolle

ApplicationWindow {
    id: main
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
    
}
