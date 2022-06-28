import QtQuick 2.0
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import QtMultimedia 5.4
import Ros2 1.0

import "include/timer.js" as Countdown
import "include/DolleBranding.js" as Dolle
Rectangle{
    FontLoader{
        id: dolle_font
        source: "fonts/regular"
    }
    RowLayout{
        anchors.fill: parent
        spacing: Dolle.logo.height
        
        Rectangle{
            id:timer_area
            height:Dolle.logo.height*2
            width:Dolle.logo.width*2
            GlueTimer{}
        }
        Item {
            width: 640
            height: 420

            ImageTransportSubscription {
                id: imageSubscriber
                topic: "/stigemaskine2/limcheck/cam0"
                throttleRate: 0.2 // 1 frame every 5 seconds
            }

            VideoOutput {
                source: imageSubscriber
            }
        }
        

    }
}
