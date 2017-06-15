
//The Little FLO simulation program 


//Set all values in terms of inches
var perimX = 5.3;
var perimY = 4.8;
var eyeSide = 0.75;
var eyeBetween = .115;
var eyeDiam = .085;
var mouthHorz = 2.5;
var mouthVert = mouthHorz / 2;
var mouthBetween = 0.185;
var mouthDiam = 0.15;
var mag = 150;

//Using magnification, approximately convert inches to pixels in html
var perimXMag = perimX * mag;
var perimYMag = perimY * mag;
var eyeSideMag = eyeSide * mag;
var eyeBetweenMag = eyeBetween * mag;
var eyeDiamMag = eyeDiam * mag;
var mouthHorzMag = mouthHorz * mag;
var mouthVertMag = mouthHorzMag / 2;
var mouthBetweenMag = mouthBetween * mag;
var mouthDiamMag = mouthDiam * mag;

//Declare location variables, in terms of pixels
var leftEyeLocX = 650;
var leftEyeLocY = 250;
var rightEyeLocX = 1035;
var rightEyeLocY = 250;
var mouthLocX = 715;
var mouthLocY = 500;

for (i = 0; i < 9 ; i++) {



}

//Paramaterize Flo's dimensions
function setValues() {
    document.getElementById("facePerim").setAttribute("rx", perimXMag / 2);
    document.getElementById("facePerim").setAttribute("ry", perimYMag / 2);
    document.getElementById("leftEye").setAttribute("height", eyeSideMag);
    document.getElementById("leftEye").setAttribute("width", eyeSideMag);
    document.getElementById("leftEye").setAttribute("x", leftEyeLocX);
    document.getElementById("leftEye").setAttribute("y", leftEyeLocY);
    document.getElementById("rightEye").setAttribute("height", eyeSideMag);
    document.getElementById("rightEye").setAttribute("width", eyeSideMag);
    document.getElementById("mouth").setAttribute("width", mouthHorzMag);
    document.getElementById("rightEye").setAttribute("x", rightEyeLocX);
    document.getElementById("rightEye").setAttribute("y", rightEyeLocY);
    document.getElementById("mouth").setAttribute("height", mouthVertMag);
    document.getElementById("mouth").setAttribute("x", mouthLocX);
    document.getElementById("mouth").setAttribute("y", mouthLocY);
}

