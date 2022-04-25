import QtQuick 2.0
import "include/DolleBranding.js" as Dolle
Rectangle {
    id: footer
    Rectangle{
        id: bottom_detail
        anchors.bottom: parent.BottomLeft
        width: footer.width
        height: 9
        color: Dolle.colors.red
    }
}
