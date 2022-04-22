import QtQuick 2.0
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "include/timer.js" as Countdown
Rectangle{
    FontLoader{
        id: dolle_font
        source: "fonts/regular"
    }
    RowLayout{
        anchors.fill: parent
        spacing: 0
        
        Rectangle{
            Layout.fillWidth: true
            Layout.minimumWidth: 50
            Layout.preferredWidth: 100
            Layout.maximumWidth: 300
            Layout.minimumHeight: 100
            Layout.rightMargin: 25
            GlueTimer{}
        }/*
        Rectangle{
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.topMargin: 40
            Layout.bottomMargin: 40
            PieCharts{}
        }*/
        

    }
}
