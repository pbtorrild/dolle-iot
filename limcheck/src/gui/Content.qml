import QtQuick 2.0
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "include/timer.js" as Countdown
import "include/DolleBranding.js" as Dolle
Rectangle{
    FontLoader{
        id: dolle_font
        source: "fonts/regular"
    }
    RowLayout{
        anchors.fill: parent
        spacing: 0
        
        Rectangle{
            id:timer_area
            height:Dolle.logo.height*2
            width:Dolle.logo.width*2
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
