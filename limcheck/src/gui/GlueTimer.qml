import QtQuick 2.0
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.12
import "include/timer.js" as Countdown
import "include/DolleBranding.js" as Dolle

ColumnLayout{
    id:glue_timer
    anchors.fill: timer_area

    FontLoader{
        id: dolle_font
        source: "fonts/regular"
    }
    spacing: 0
    Timer {
        interval: 1000; running: true; repeat: true
        onTriggered: {time.text = Countdown.getTime();time.color = Countdown.timer_color;}
        }

    Text{
        id:time
        height: glue_timer.height/2
        width: glue_timer.width
        Layout.alignment: Qt.AlignCenter
        font.family: dolle_font.name
        horizontalAlignment: Text.AlignHCenter
        fontSizeMode: Text.Fit
        minimumPixelSize: 10
        font.pixelSize: 72 
    }
    Button {
        id:control
        Layout.alignment: Qt.AlignCenter
        width: 150
        height: Dolle.logo.height
        text: qsTr("KVITTER FOR RENGØRING")
        contentItem:Text{
            text: control.text
            anchors.centerIn: parent
            anchors.fill: parent
            color:"#FFFFFF"
            font.family: dolle_font.name
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 25 
        }
        background: Rectangle {
                implicitWidth: glue_timer.width
                implicitHeight: glue_timer.height/2
                color: Dolle.colors.red
                radius: 15

        }
        hoverEnabled: false
        onPressed: {
            background.color=Dolle.colors.dark_red
        }
        onReleased: {
            Countdown.resetTimer(),
            background.color=Dolle.colors.red
        }
    }
}
    
