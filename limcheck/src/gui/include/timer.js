//Note: you only need '.pragma library' if you are planning to
//change this variable from multiple qml files
.pragma library
//TimerFunctions
var tick= 20*60;
var timer_color="#5C5851";
var timer_output= "20:00";

function resetTimer(){
    tick=20*60;
    timer_color = "#5C5851";
    timer_output = getTime();
    tick--;
}
function regular(){
    let min =Math.floor(tick/60).toString();;
    let sec = (tick%60).toString();
    if(min.length<2){
        min='0'+min;
    }
    if(sec.length<2){
        sec='0'+sec;
    }
    timer_output = min.toString()+':'+sec.toString();
}

function timesUp(){
    let local_tick=tick*-1;
    let min =Math.floor(local_tick/60).toString();
    let sec = (local_tick%60).toString();
    if(min.length<2){
        min='0'+min;
    }
    if(sec.length<2){
        sec='0'+sec;
    }
    timer_output = '-'+min+':'+sec;

}
function getTime(){
    if(tick>0){
        regular();
        tick--;
        return timer_output
    }
    if(tick==0){
        timer_color = "#D7102D";
        tick--;
        return "00:00"
    }

    timesUp();
    tick--;
    return timer_output
}