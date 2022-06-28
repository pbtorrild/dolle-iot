import QtQuick 2.0
import QtQuick.Layouts 1.12
import "include/DolleBranding.js" as Dolle

RowLayout {
    id:header
    FontLoader{
        id: header_font
        source: "fonts/regular"
    }
    layoutDirection: "LeftToRight"
    Rectangle{
        id: header_text
        Layout.preferredHeight: (11/13)*logo.height
        Layout.fillWidth: true
        Text {
            id: title
            text: "LIMCHECK"
            height: parent.height*0.7
            width: parent.width
            font.family: header_font.name
            font.pixelSize: height
            anchors.top: header_text.top
            anchors.left: header_text.left
            anchors.right:header_text.left
            anchors.bottom: subtitle.top

            color: "#000000"
        }
        Text {
            id: subtitle
            text: "DOLLE IOT"
            height: parent.height*0.25
            width: parent.width
            font.family: header_font.name
            font.pixelSize: height
            anchors.left:header_text.left
            anchors.bottom: header_text.bottom
            anchors.bottomMargin: header_text.height*0.05

            color: Dolle.colors.red
        }
    }
    Image {
        id: logo
        Layout.preferredWidth: Dolle.logo.width
        Layout.preferredHeight: Dolle.logo.height
        source: Dolle.logo.source

        MouseArea {
            anchors.fill: parent
            
            onPressAndHold: {main.visibility="Windowed"}
            onDoubleClicked: {main.visibility="FullScreen"}
        }

    }
}
