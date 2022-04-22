import QtQuick 2.0
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.12
import QtCharts 2.3

ColumnLayout{anchors.fill: parent
    id:charts
    FontLoader{
        id: dolle_font
        source: "fonts/regular"
    }
    spacing: 0
    Text{
        height: 25
        Layout.fillWidth: true
        id:title
        text:"HØJRE SIDE"
        font.family: dolle_font.name
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 12
    }
    RowLayout{
        spacing: 25
        Layout.alignment: Qt.AlignCenter
        height: content.height/2-2*title.height
        width:charts.width
        ChartView {
            antialiasing: true
            backgroundColor: "#EEE8E4"

            backgroundRoundness: 25
            Layout.alignment: Qt.AlignLeft
            Layout.preferredWidth: parent.width/3-backgroundRoundness
            Layout.preferredHeight: parent.height

            PieSeries {
                PieSlice { value: 94.9; color: "#5C5851"; label: value; }
                PieSlice { value: 5.1 ; color: "#D7102D"; exploded: true; label: value; }
            }
         }
        ChartView {
            antialiasing: true
            backgroundColor: "#EEE8E4"

            backgroundRoundness: 25
            Layout.alignment: Qt.AlignCenter
            Layout.preferredWidth: parent.width/3-backgroundRoundness
            Layout.preferredHeight: parent.height
            PieSeries {
                PieSlice { value: 94.9; color: "#5C5851"; label: value; }
                PieSlice { value: 5.1 ; color: "#D7102D"; exploded: true; label: value; }
            }
         }
        ChartView {
            antialiasing: true
            backgroundColor: "#EEE8E4"

            backgroundRoundness: 25
            Layout.alignment: Qt.AlignRight
            Layout.preferredWidth: parent.width/3-backgroundRoundness
            Layout.preferredHeight: parent.height
            PieSeries {
                PieSlice { value: 93.8; color: "#5C5851"; label: value; }
                PieSlice { value: 6.2 ; color: "#D7102D"; exploded: true; label: value; }
            }
         }
    }

    Text{
        height: 25
        Layout.fillWidth: true
        text:"VENSTRE SIDE"
        font.family: dolle_font.name
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 12
    }
    RowLayout{
        spacing: 25
        Layout.alignment: Qt.AlignCenter
        height: content.height/2-2*title.height
        width:charts.width
        ChartView {
            antialiasing: true
            backgroundColor: "#EEE8E4"

            backgroundRoundness: 25
            Layout.alignment: Qt.AlignLeft
            Layout.preferredWidth: parent.width/3-backgroundRoundness
            Layout.preferredHeight: parent.height
            PieSeries {
                PieSlice { value: 91; color: "#5C5851"; label: value; }
                PieSlice { value: 9 ; color: "#D7102D"; exploded: true; label: value; }
            }
         }
        ChartView {
            antialiasing: true
            backgroundColor: "#EEE8E4"

            backgroundRoundness: 25
            Layout.alignment: Qt.AlignCenter
            Layout.preferredWidth: parent.width/3-backgroundRoundness
            Layout.preferredHeight: parent.height
            PieSeries {
                PieSlice { value: 92.3; color: "#5C5851"; label: value; }
                PieSlice { value: 7.7 ; color: "#D7102D"; exploded: true; label: value; }
            }
         }
        ChartView {
            antialiasing: true
            backgroundColor: "#EEE8E4"

            backgroundRoundness: 25
            Layout.alignment: Qt.AlignRight
            Layout.preferredWidth: parent.width/3-backgroundRoundness
            Layout.preferredHeight: parent.height
            PieSeries {
                PieSlice { value: 97.8; color: "#5C5851"; label: value; }
                PieSlice { value: 2.2; color: "#D7102D"; exploded: true; label: value; }
            }
         }
    }
}
